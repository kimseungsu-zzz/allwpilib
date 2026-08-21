// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VMXWatchdogInternal.h"

#include <memory>
#include <mutex>

#include "DriverStationInternal.h"
#include "VMXErrors.h"
#include "VMXPi.h"
#include "VMXRuntime.h"

namespace hal::vmx {
namespace {

class VMXWatchdogRuntime final {
 public:
  bool Initialize() noexcept {
    try {
      return InitializeImpl();
    } catch (...) {
      std::scoped_lock lock{m_mutex};
      m_watchdog.reset();
      return false;
    }
  }

 private:
  bool InitializeImpl() {
    std::scoped_lock lock{m_mutex};
    if (m_watchdog) {
      return true;
    }
    auto context = GetRuntimeContext();
    if (!context) {
      return false;
    }
    auto sdkMutex = std::make_shared<std::mutex>();
    VMXWatchdogBackend backend;
    backend.setTimeout = [context, sdkMutex](uint16_t timeout, int32_t& status) {
      std::scoped_lock sdkLock{*sdkMutex};
      VMXErrorCode error = 0;
      const bool ok = context->getIO().SetWatchdogTimeoutPeriodMS(timeout, &error);
      status = ok ? HAL_SUCCESS : INCOMPATIBLE_STATE;
      return ok;
    };
    backend.setManagedOutputs =
        [context, sdkMutex](bool flex, bool highCurrent, bool comm,
                            int32_t& status) {
          std::scoped_lock sdkLock{*sdkMutex};
          VMXErrorCode error = 0;
          const bool ok = context->getIO().SetWatchdogManagedOutputs(
              flex, highCurrent, comm, &error);
          status = ok ? HAL_SUCCESS : INCOMPATIBLE_STATE;
          return ok;
        };
    backend.setEnabled = [context, sdkMutex](bool enabled, int32_t& status) {
      std::scoped_lock sdkLock{*sdkMutex};
      VMXErrorCode error = 0;
      const bool ok = context->getIO().SetWatchdogEnabled(enabled, &error);
      status = ok ? HAL_SUCCESS : INCOMPATIBLE_STATE;
      return ok;
    };
    backend.feed = [context, sdkMutex](int32_t& status) {
      std::scoped_lock sdkLock{*sdkMutex};
      VMXErrorCode error = 0;
      const bool ok = context->getIO().FeedWatchdog(&error);
      status = ok ? HAL_SUCCESS : INCOMPATIBLE_STATE;
      return ok;
    };
    backend.expireNow = [context, sdkMutex](int32_t& status) {
      std::scoped_lock sdkLock{*sdkMutex};
      VMXErrorCode error = 0;
      const bool ok = context->getIO().ExpireWatchdogNow(&error);
      status = ok ? HAL_SUCCESS : INCOMPATIBLE_STATE;
      return ok;
    };
    backend.getEnabled = [context, sdkMutex](bool& enabled, int32_t& status) {
      std::scoped_lock sdkLock{*sdkMutex};
      VMXErrorCode error = 0;
      const bool ok = context->getIO().GetWatchdogEnabled(enabled, &error);
      status = ok ? HAL_SUCCESS : INCOMPATIBLE_STATE;
      return ok;
    };
    backend.getExpired = [context, sdkMutex](bool& expired, int32_t& status) {
      std::scoped_lock sdkLock{*sdkMutex};
      VMXErrorCode error = 0;
      const bool ok = context->getIO().GetWatchdogExpired(expired, &error);
      status = ok ? HAL_SUCCESS : INCOMPATIBLE_STATE;
      return ok;
    };
    try {
      m_watchdog = std::make_unique<VMXHardwareWatchdog>(
          std::move(backend), [](uint64_t now) {
            auto& state = GetDriverStationState();
            return IsRuntimeInitialized() && state.IsFresh(now) &&
                   state.IsProgramHeartbeatFresh(now) && !state.IsEStop();
          });
      if (!m_watchdog->Configure(kWatchdogTimeoutMs, true, true, false, true)) {
        m_watchdog.reset();
        return false;
      }
      return true;
    } catch (...) {
      m_watchdog.reset();
      return false;
    }
  }

 public:
  void Shutdown() noexcept {
    std::unique_ptr<VMXHardwareWatchdog> watchdog;
    {
      std::scoped_lock lock{m_mutex};
      watchdog = std::move(m_watchdog);
    }
    if (watchdog) {
      watchdog->Shutdown();
    }
  }

  void Notify() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_watchdog) {
      m_watchdog->NotifyStateChanged();
    }
  }

 private:
  static constexpr uint16_t kWatchdogTimeoutMs = 100;
  std::mutex m_mutex;
  std::unique_ptr<VMXHardwareWatchdog> m_watchdog;
};

VMXWatchdogRuntime& GetRuntime() {
  static VMXWatchdogRuntime runtime;
  return runtime;
}

}  // namespace

bool InitializeHardwareWatchdog() noexcept { return GetRuntime().Initialize(); }

void ShutdownHardwareWatchdog() noexcept { GetRuntime().Shutdown(); }

void NotifyWatchdogStateChanged() noexcept { GetRuntime().Notify(); }

}  // namespace hal::vmx
