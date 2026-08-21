// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

#include "hal/Errors.h"

namespace hal::vmx {

struct VMXWatchdogBackend final {
  std::function<bool(uint16_t, int32_t&)> setTimeout;
  std::function<bool(bool, bool, bool, int32_t&)> setManagedOutputs;
  std::function<bool(bool, int32_t&)> setEnabled;
  std::function<bool(int32_t&)> feed;
  std::function<bool(int32_t&)> expireNow;
  std::function<bool(bool&, int32_t&)> getEnabled;
  std::function<bool(bool&, int32_t&)> getExpired;
};

using VMXWatchdogClock = std::function<uint64_t()>;
using VMXWatchdogSafetyPredicate = std::function<bool(uint64_t)>;

/**
 * Host-testable watchdog lifecycle and feed scheduler.  The SDK is called only
 * through VMXWatchdogBackend; no fake output state is exposed as hardware.
 */
class VMXHardwareWatchdog final {
 public:
  VMXHardwareWatchdog(VMXWatchdogBackend backend,
                      VMXWatchdogSafetyPredicate safety,
                      VMXWatchdogClock clock = {})
      : m_backend{std::move(backend)},
        m_safety{std::move(safety)},
        m_clock{std::move(clock)} {
    if (!m_clock) {
      m_clock = [] {
        return static_cast<uint64_t>(std::chrono::duration_cast<
                                         std::chrono::microseconds>(
                                         std::chrono::steady_clock::now()
                                             .time_since_epoch())
                                         .count());
      };
    }
  }

  VMXHardwareWatchdog(const VMXHardwareWatchdog&) = delete;
  VMXHardwareWatchdog& operator=(const VMXHardwareWatchdog&) = delete;

  ~VMXHardwareWatchdog() { Shutdown(); }

  bool Configure(uint16_t timeoutMs, bool manageFlexDio = true,
                 bool manageHighCurrentDio = true,
                 bool manageCommDio = false, bool startWorker = true) noexcept {
    if (timeoutMs == 0 || !m_backend.setTimeout ||
        !m_backend.setManagedOutputs || !m_backend.setEnabled) {
      SetError(PARAMETER_OUT_OF_RANGE);
      return false;
    }
    {
      std::scoped_lock lock{m_mutex};
      if (m_worker.joinable()) {
        m_lastError = INCOMPATIBLE_STATE;
        return false;
      }
    }
    int32_t status = HAL_SUCCESS;
    try {
      if (!m_backend.setTimeout(timeoutMs, status) ||
          !m_backend.setManagedOutputs(manageFlexDio, manageHighCurrentDio,
                                       manageCommDio, status) ||
          !m_backend.setEnabled(true, status)) {
        SetError(status);
        return false;
      }
    } catch (...) {
      SetError(INCOMPATIBLE_STATE);
      return false;
    }
    {
      std::scoped_lock lock{m_mutex};
      m_configured = true;
      m_enabled = true;
      m_shutdown = false;
      m_expireIssued = false;
      m_lastFeed = 0;
      m_timeoutMs = timeoutMs;
    }
    if (startWorker) {
      try {
        std::scoped_lock lock{m_mutex};
        m_stopRequested = false;
        m_worker = std::thread{[this] { Run(); }};
      } catch (...) {
        SetError(INCOMPATIBLE_STATE);
        ForceExpire();
        return false;
      }
    }
    return true;
  }

  void NotifyStateChanged() noexcept { m_cv.notify_all(); }

  bool Tick(uint64_t now = 0) noexcept {
    if (now == 0) {
      now = m_clock();
    }
    bool configured = false;
    bool shutdown = false;
    {
      std::scoped_lock lock{m_mutex};
      configured = m_configured && m_enabled;
      shutdown = m_shutdown;
    }
    if (!configured || shutdown) {
      return false;
    }
    bool safe = false;
    try {
      safe = m_safety && m_safety(now);
    } catch (...) {
      safe = false;
    }
    if (safe) {
      int32_t status = HAL_SUCCESS;
      bool fed = false;
      try {
        fed = m_backend.feed && m_backend.feed(status);
      } catch (...) {
        status = INCOMPATIBLE_STATE;
      }
      if (!fed) {
        SetError(status);
        ForceExpire();
        return false;
      }
      std::scoped_lock lock{m_mutex};
      m_lastFeed = now;
      m_expireIssued = false;
      return true;
    }
    ForceExpire();
    return false;
  }

  void ForceExpire() noexcept {
    bool shouldExpire = false;
    {
      std::scoped_lock lock{m_mutex};
      if (m_configured && m_enabled && !m_expireIssued) {
        m_expireIssued = true;
        shouldExpire = true;
      }
    }
    if (shouldExpire && m_backend.expireNow) {
      int32_t status = HAL_SUCCESS;
      bool expired = false;
      try {
        expired = m_backend.expireNow(status);
      } catch (...) {
        status = INCOMPATIBLE_STATE;
      }
      if (!expired) {
        SetError(status);
      }
    }
  }

  void Shutdown() noexcept {
    std::thread worker;
    {
      std::scoped_lock lock{m_mutex};
      if (m_shutdown && !m_worker.joinable()) {
        return;
      }
      m_shutdown = true;
      m_stopRequested = true;
      worker = std::move(m_worker);
    }
    m_cv.notify_all();
    if (worker.joinable()) {
      worker.join();
    }
    ForceExpire();
    bool disable = false;
    {
      std::scoped_lock lock{m_mutex};
      disable = m_enabled;
      m_enabled = false;
    }
    if (disable && m_backend.setEnabled) {
      int32_t status = HAL_SUCCESS;
      bool disabled = false;
      try {
        disabled = m_backend.setEnabled(false, status);
      } catch (...) {
        status = INCOMPATIBLE_STATE;
      }
      if (!disabled) {
        SetError(status);
      }
    }
  }

  bool IsConfigured() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_configured;
  }

  bool IsExpired() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_expireIssued;
  }

  uint64_t LastFeed() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_lastFeed;
  }

  int32_t LastError() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_lastError;
  }

 private:
  void SetError(int32_t status) noexcept {
    std::scoped_lock lock{m_mutex};
    m_lastError = status == HAL_SUCCESS ? INCOMPATIBLE_STATE : status;
  }

  void Run() noexcept {
    while (true) {
      {
        std::unique_lock lock{m_mutex};
        if (m_stopRequested) {
          return;
        }
      }
      Tick();
      std::unique_lock lock{m_mutex};
      if (m_stopRequested) {
        return;
      }
      m_cv.wait_for(lock, std::chrono::milliseconds{20});
    }
  }

  VMXWatchdogBackend m_backend;
  VMXWatchdogSafetyPredicate m_safety;
  VMXWatchdogClock m_clock;
  mutable std::mutex m_mutex;
  std::condition_variable m_cv;
  std::thread m_worker;
  uint16_t m_timeoutMs = 0;
  uint64_t m_lastFeed = 0;
  int32_t m_lastError = HAL_SUCCESS;
  bool m_configured = false;
  bool m_enabled = false;
  bool m_shutdown = false;
  bool m_stopRequested = false;
  bool m_expireIssued = false;
};

bool InitializeHardwareWatchdog() noexcept;
void ShutdownHardwareWatchdog() noexcept;
void NotifyWatchdogStateChanged() noexcept;

}  // namespace hal::vmx
