// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <cmath>
#include <climits>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "SPIInternal.h"
#include "hal/SPI.h"

namespace hal::vmx {

enum class SPIAutoResult {
  kOk,
  kPortOutOfRange,
  kAlreadyAllocated,
  kResourceConflict,
  kNotInitialized,
  kInvalidSize,
  kNullPointer,
  kInvalidPeriod,
  kInvalidTrigger,
  kUnsupportedSource,
  kHardwareFailure,
  kUnsupportedStall,
};

class SPIAutoWaiter {
 public:
  virtual ~SPIAutoWaiter() = default;

  // Waits for an absolute VMX-time deadline and returns the timestamp at
  // which the scheduler woke. Stop() must release a blocked wait.
  virtual bool WaitUntil(uint64_t deadline, uint64_t& firedAt) noexcept = 0;
  virtual void Stop() noexcept = 0;
};

class SPIAutoTriggerWaiter {
 public:
  virtual ~SPIAutoTriggerWaiter() = default;

  // Waits for one or more configured DIO edges. Stop() must release a blocked
  // wait without invoking a SPI transaction.
  virtual bool Wait() noexcept = 0;
  virtual void Stop() noexcept = 0;
};

using SPIAutoTransfer =
    std::function<bool(const uint8_t*, uint8_t*, uint16_t)>;
using SPIAutoClock = std::function<uint64_t()>;
using SPIAutoWaiterFactory = std::function<std::unique_ptr<SPIAutoWaiter>()>;

class SPIAutoEngine final {
 public:
  SPIAutoEngine(SPIAutoTransfer transfer, SPIAutoClock clock,
                SPIAutoWaiterFactory waiterFactory)
      : m_transfer{std::move(transfer)},
        m_clock{std::move(clock)},
        m_waiterFactory{std::move(waiterFactory)} {}

  ~SPIAutoEngine() { Free(); }

  SPIAutoEngine(const SPIAutoEngine&) = delete;
  SPIAutoEngine& operator=(const SPIAutoEngine&) = delete;

  SPIAutoResult Initialize(int32_t bufferSize) noexcept {
    if (bufferSize <= 0) {
      return SPIAutoResult::kInvalidSize;
    }
    std::scoped_lock lock{m_mutex};
    if (m_initialized) {
      return SPIAutoResult::kAlreadyAllocated;
    }
    try {
      m_capacity = static_cast<size_t>(bufferSize);
      m_ring.clear();
      m_ring.resize(0);
      m_initialized = true;
      m_closed = false;
      m_transmit.clear();
      m_zeroSize = 0;
      m_transmitConfigured = false;
      m_dropped = 0;
      return SPIAutoResult::kOk;
    } catch (...) {
      m_capacity = 0;
      return SPIAutoResult::kHardwareFailure;
    }
  }

  SPIAutoResult Free() noexcept {
    Stop();
    std::scoped_lock lock{m_mutex};
    if (!m_initialized) {
      return SPIAutoResult::kOk;
    }
    m_closed = true;
    m_initialized = false;
    m_transmit.clear();
    m_ring.clear();
    m_capacity = 0;
    m_dropped = 0;
    m_transmitConfigured = false;
    m_readCv.notify_all();
    return SPIAutoResult::kOk;
  }

  SPIAutoResult SetTransmitData(const uint8_t* data, int32_t dataSize,
                                int32_t zeroSize) noexcept {
    if (dataSize < 0 || dataSize > 32 || zeroSize < 0 || zeroSize > 127) {
      return SPIAutoResult::kInvalidSize;
    }
    if (dataSize > 0 && data == nullptr) {
      return SPIAutoResult::kNullPointer;
    }
    std::vector<uint8_t> transmit;
    try {
      if (dataSize > 0) {
        transmit.assign(data, data + dataSize);
      }
    } catch (...) {
      return SPIAutoResult::kHardwareFailure;
    }
    std::scoped_lock lock{m_mutex};
    if (!m_initialized) {
      return SPIAutoResult::kNotInitialized;
    }
    m_transmit = std::move(transmit);
    m_zeroSize = static_cast<size_t>(zeroSize);
    m_transmitConfigured = true;
    return SPIAutoResult::kOk;
  }

  SPIAutoResult StartRate(double periodSeconds) noexcept {
    if (!(periodSeconds > 0.0) || !std::isfinite(periodSeconds)) {
      return SPIAutoResult::kInvalidPeriod;
    }
    const auto period = static_cast<uint64_t>(periodSeconds * 1'000'000.0);
    if (period == 0) {
      return SPIAutoResult::kInvalidPeriod;
    }
    std::unique_ptr<SPIAutoWaiter> waiter;
    try {
      waiter = m_waiterFactory ? m_waiterFactory() : nullptr;
    } catch (...) {
      waiter.reset();
    }
    if (!waiter) {
      return SPIAutoResult::kHardwareFailure;
    }
    Stop();
    {
      std::scoped_lock lock{m_mutex};
      if (!m_initialized) {
        return SPIAutoResult::kNotInitialized;
      }
      if (!m_transmitConfigured) {
        return SPIAutoResult::kInvalidSize;
      }
      try {
        m_waiter = std::shared_ptr<SPIAutoWaiter>{std::move(waiter)};
        m_running = true;
        m_stopRequested = false;
        m_mode = RunMode::kRate;
        const auto sharedWaiter = m_waiter;
        m_thread = std::thread([this, sharedWaiter, period] {
          RunRate(sharedWaiter, period);
        });
      } catch (...) {
        m_waiter.reset();
        m_running = false;
        m_mode = RunMode::kStopped;
        return SPIAutoResult::kHardwareFailure;
      }
    }
    return SPIAutoResult::kOk;
  }

  SPIAutoResult StartTrigger(std::unique_ptr<SPIAutoTriggerWaiter> trigger,
                             bool rising, bool falling) noexcept {
    if (!rising && !falling) {
      return SPIAutoResult::kInvalidTrigger;
    }
    if (!trigger) {
      return SPIAutoResult::kHardwareFailure;
    }
    Stop();
    {
      std::scoped_lock lock{m_mutex};
      if (!m_initialized) {
        return SPIAutoResult::kNotInitialized;
      }
      if (!m_transmitConfigured) {
        return SPIAutoResult::kInvalidSize;
      }
      try {
        m_trigger = std::shared_ptr<SPIAutoTriggerWaiter>{std::move(trigger)};
        m_running = true;
        m_stopRequested = false;
        m_mode = RunMode::kTrigger;
        const auto sharedTrigger = m_trigger;
        m_thread = std::thread([this, sharedTrigger] {
          RunTrigger(sharedTrigger);
        });
      } catch (...) {
        m_trigger.reset();
        m_running = false;
        m_mode = RunMode::kStopped;
        return SPIAutoResult::kHardwareFailure;
      }
    }
    return SPIAutoResult::kOk;
  }

  SPIAutoResult Stop() noexcept {
    std::shared_ptr<SPIAutoWaiter> waiter;
    std::shared_ptr<SPIAutoTriggerWaiter> trigger;
    std::thread thread;
    {
      std::scoped_lock lock{m_mutex};
      if (!m_running && !m_thread.joinable()) {
        return m_initialized ? SPIAutoResult::kOk
                             : SPIAutoResult::kNotInitialized;
      }
      m_running = false;
      m_stopRequested = true;
      waiter = m_waiter;
      trigger = m_trigger;
      thread = std::move(m_thread);
    }
    if (waiter) {
      waiter->Stop();
    }
    if (trigger) {
      trigger->Stop();
    }
    m_readCv.notify_all();
    if (thread.joinable()) {
      thread.join();
    }
    {
      std::scoped_lock lock{m_mutex};
      m_waiter.reset();
      m_trigger.reset();
      m_mode = RunMode::kStopped;
      m_stopRequested = false;
    }
    return SPIAutoResult::kOk;
  }

  SPIAutoResult ForceRead() noexcept {
    if (!IsReady()) {
      return SPIAutoResult::kNotInitialized;
    }
    return PerformTransfer() ? SPIAutoResult::kOk
                             : SPIAutoResult::kHardwareFailure;
  }

  int32_t Read(uint32_t* buffer, int32_t numToRead, double timeout,
               SPIAutoResult& result) noexcept {
    result = SPIAutoResult::kOk;
    if (numToRead < 0) {
      result = SPIAutoResult::kInvalidSize;
      return 0;
    }
    if (numToRead > 0 && buffer == nullptr) {
      result = SPIAutoResult::kNullPointer;
      return 0;
    }
    if (!(timeout >= 0.0) || std::isnan(timeout)) {
      result = SPIAutoResult::kInvalidPeriod;
      return 0;
    }
    if (!IsReady()) {
      result = SPIAutoResult::kNotInitialized;
      return 0;
    }

    std::unique_lock lock{m_mutex};
    if (numToRead == 0) {
      return static_cast<int32_t>(m_ring.size());
    }
    auto ready = [this, numToRead] {
      return m_ring.size() >= static_cast<size_t>(numToRead) ||
             m_closed || !m_initialized || !m_running;
    };
    if (m_ring.size() < static_cast<size_t>(numToRead) && m_running) {
      if (timeout == std::numeric_limits<double>::infinity()) {
        m_readCv.wait(lock, ready);
      } else if (timeout > 0.0) {
        m_readCv.wait_for(lock, std::chrono::duration<double>{timeout}, ready);
      }
    }
    const auto count = (std::min)(static_cast<size_t>(numToRead),
                                  m_ring.size());
    for (size_t i = 0; i < count; ++i) {
      buffer[i] = m_ring.front();
      m_ring.pop_front();
    }
    return static_cast<int32_t>(m_ring.size());
  }

  int32_t GetDropped(SPIAutoResult& result) const noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_initialized) {
      result = SPIAutoResult::kNotInitialized;
      return 0;
    }
    result = SPIAutoResult::kOk;
    return m_dropped > static_cast<uint64_t>(INT32_MAX)
               ? INT32_MAX
               : static_cast<int32_t>(m_dropped);
  }

  bool IsInitialized() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_initialized;
  }

 private:
  enum class RunMode { kStopped, kRate, kTrigger };

  bool IsReady() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_initialized && m_transmitConfigured && static_cast<bool>(m_transfer);
  }

  bool PerformTransfer() noexcept {
    try {
      std::vector<uint8_t> transmit;
      std::vector<uint8_t> received;
      size_t zeroSize = 0;
      {
        std::scoped_lock lock{m_mutex};
        if (!m_initialized || !m_transmitConfigured || !m_transfer ||
            m_stopRequested) {
          return false;
        }
        transmit = m_transmit;
        zeroSize = m_zeroSize;
        received.resize(transmit.size() + zeroSize);
        transmit.resize(received.size(), 0);
      }

      bool success = false;
      {
        std::scoped_lock transferLock{m_transferMutex};
        try {
          success = m_transfer(transmit.data(), received.data(),
                               static_cast<uint16_t>(received.size()));
        } catch (...) {
          success = false;
        }
      }
      if (!success) {
        return false;
      }

      uint64_t timestamp = 0;
      try {
        timestamp = m_clock ? m_clock() : 0;
      } catch (...) {
        return false;
      }

      std::scoped_lock lock{m_mutex};
      if (!m_initialized || m_stopRequested) {
        return false;
      }
      const size_t sampleSize = received.size() + 1;
      if (sampleSize > m_capacity || m_ring.size() + sampleSize > m_capacity) {
        ++m_dropped;
        return true;
      }
      m_ring.push_back(static_cast<uint32_t>(timestamp));
      for (auto byte : received) {
        m_ring.push_back(static_cast<uint32_t>(byte));
      }
      m_readCv.notify_all();
      return true;
    } catch (...) {
      return false;
    }
  }

  void RunRate(const std::shared_ptr<SPIAutoWaiter>& waiter,
               uint64_t period) noexcept {
    uint64_t deadline = 0;
    try {
      deadline = (m_clock ? m_clock() : 0) + period;
    } catch (...) {
      WorkerFinished();
      return;
    }
    while (true) {
      uint64_t firedAt = 0;
      if (!waiter->WaitUntil(deadline, firedAt)) {
        WorkerFinished();
        return;
      }
      {
        std::scoped_lock lock{m_mutex};
        if (!m_running || m_stopRequested) {
          WorkerFinished();
          return;
        }
      }
      PerformTransfer();
      deadline = firedAt + period;
    }
  }

  void RunTrigger(const std::shared_ptr<SPIAutoTriggerWaiter>& trigger) noexcept {
    while (true) {
      if (!trigger->Wait()) {
        WorkerFinished();
        return;
      }
      {
        std::scoped_lock lock{m_mutex};
        if (!m_running || m_stopRequested) {
          WorkerFinished();
          return;
        }
      }
      PerformTransfer();
    }
  }

  void WorkerFinished() noexcept {
    std::scoped_lock lock{m_mutex};
    m_running = false;
    m_readCv.notify_all();
  }

  SPIAutoTransfer m_transfer;
  SPIAutoClock m_clock;
  SPIAutoWaiterFactory m_waiterFactory;
  mutable std::mutex m_mutex;
  std::mutex m_transferMutex;
  std::condition_variable m_readCv;
  std::thread m_thread;
  std::shared_ptr<SPIAutoWaiter> m_waiter;
  std::shared_ptr<SPIAutoTriggerWaiter> m_trigger;
  std::deque<uint32_t> m_ring;
  std::vector<uint8_t> m_transmit;
  size_t m_capacity = 0;
  size_t m_zeroSize = 0;
  uint64_t m_dropped = 0;
  std::atomic_bool m_initialized = false;
  bool m_closed = false;
  bool m_transmitConfigured = false;
  bool m_running = false;
  bool m_stopRequested = false;
  RunMode m_mode = RunMode::kStopped;
};

using SPIAutoTriggerFactory = std::function<std::unique_ptr<SPIAutoTriggerWaiter>(
    int32_t, bool, bool)>;

class SPIAutoManager final {
 public:
  SPIAutoManager(SPIManager& spi, SPIAutoClock clock,
                 SPIAutoWaiterFactory waiterFactory,
                 SPIAutoTriggerFactory triggerFactory)
      : m_spi{spi},
        m_port{-1},
        m_engine{
            [this](const uint8_t* send, uint8_t* receive, uint16_t size) {
              int32_t transferred = -1;
              const auto port = static_cast<HAL_SPIPort>(m_port.load());
              return m_spi.Transaction(port, send, receive, size, transferred) ==
                     SPIResult::kOk;
            },
            std::move(clock), std::move(waiterFactory)},
        m_triggerFactory{std::move(triggerFactory)} {}

  SPIAutoResult Initialize(HAL_SPIPort port, int32_t bufferSize) noexcept {
    if (ValidateSPIPort(port) != SPIResult::kOk) {
      return SPIAutoResult::kPortOutOfRange;
    }
    std::scoped_lock lock{m_mutex};
    if (m_initialized) {
      return SPIAutoResult::kAlreadyAllocated;
    }
    const auto spiResult = m_spi.Initialize(port);
    if (spiResult != SPIResult::kOk) {
      return spiResult == SPIResult::kResourceConflict
                 ? SPIAutoResult::kResourceConflict
                 : SPIAutoResult::kNotInitialized;
    }
    m_port.store(static_cast<int32_t>(port));
    const auto result = m_engine.Initialize(bufferSize);
    if (result != SPIAutoResult::kOk) {
      m_spi.Close(port);
      m_port.store(-1);
      return result;
    }
    m_initialized = true;
    return SPIAutoResult::kOk;
  }

  SPIAutoResult Free(HAL_SPIPort port) noexcept {
    if (ValidateSPIPort(port) != SPIResult::kOk) {
      return SPIAutoResult::kPortOutOfRange;
    }
    std::scoped_lock lock{m_mutex};
    if (!m_initialized) {
      return SPIAutoResult::kOk;
    }
    const auto result = m_engine.Free();
    m_spi.Close(static_cast<HAL_SPIPort>(m_port.load()));
    m_port.store(-1);
    m_initialized = false;
    return result;
  }

  SPIAutoResult SetTransmitData(HAL_SPIPort port, const uint8_t* data,
                                int32_t dataSize, int32_t zeroSize) noexcept {
    return WithPort(port, [this, data, dataSize, zeroSize] {
      return m_engine.SetTransmitData(data, dataSize, zeroSize);
    });
  }

  SPIAutoResult StartRate(HAL_SPIPort port, double period) noexcept {
    return WithPort(port, [this, period] { return m_engine.StartRate(period); });
  }

  SPIAutoResult StartTrigger(HAL_SPIPort port, HAL_Handle source,
                             HAL_AnalogTriggerType triggerType, bool rising,
                             bool falling) noexcept {
    if (!rising && !falling) {
      return SPIAutoResult::kInvalidTrigger;
    }
    int32_t channel = -1;
    const auto validation = ValidateTriggerSource(source, triggerType, channel);
    if (validation != SPIAutoResult::kOk) {
      return validation;
    }
    return WithPort(port, [this, channel, rising, falling] {
      std::unique_ptr<SPIAutoTriggerWaiter> trigger;
      try {
        trigger = m_triggerFactory ? m_triggerFactory(channel, rising, falling)
                                   : nullptr;
      } catch (...) {
        trigger.reset();
      }
      return m_engine.StartTrigger(std::move(trigger), rising, falling);
    });
  }

  SPIAutoResult Stop(HAL_SPIPort port) noexcept {
    return WithPort(port, [this] { return m_engine.Stop(); });
  }

  SPIAutoResult SetForceRead(HAL_SPIPort port) noexcept {
    return WithPort(port, [this] { return m_engine.ForceRead(); });
  }

  int32_t Read(HAL_SPIPort port, uint32_t* buffer, int32_t numToRead,
               double timeout, SPIAutoResult& result) noexcept {
    result = ValidatePort(port);
    if (result != SPIAutoResult::kOk) {
      return 0;
    }
    return m_engine.Read(buffer, numToRead, timeout, result);
  }

  int32_t GetDropped(HAL_SPIPort port, SPIAutoResult& result) noexcept {
    result = ValidatePort(port);
    if (result != SPIAutoResult::kOk) {
      return 0;
    }
    return m_engine.GetDropped(result);
  }

  SPIAutoResult ConfigureStall(HAL_SPIPort port) noexcept {
    const auto result = ValidatePort(port);
    return result == SPIAutoResult::kOk ? SPIAutoResult::kUnsupportedStall
                                        : result;
  }

  void Shutdown() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_initialized) {
      return;
    }
    const auto port = static_cast<HAL_SPIPort>(m_port.load());
    m_engine.Free();
    m_spi.Close(port);
    m_port.store(-1);
    m_initialized = false;
  }

 private:
  SPIAutoResult ValidatePort(HAL_SPIPort port) const noexcept {
    if (ValidateSPIPort(port) != SPIResult::kOk) {
      return SPIAutoResult::kPortOutOfRange;
    }
    return m_initialized ? SPIAutoResult::kOk
                         : SPIAutoResult::kNotInitialized;
  }

  template <typename Function>
  SPIAutoResult WithPort(HAL_SPIPort port, Function&& function) noexcept {
    const auto result = ValidatePort(port);
    if (result != SPIAutoResult::kOk) {
      return result;
    }
    try {
      return function();
    } catch (...) {
      return SPIAutoResult::kHardwareFailure;
    }
  }

  static SPIAutoResult ValidateTriggerSource(
      HAL_Handle source, HAL_AnalogTriggerType triggerType,
      int32_t& channel) noexcept;

  SPIManager& m_spi;
  mutable std::mutex m_mutex;
  std::atomic<int32_t> m_port;
  SPIAutoEngine m_engine;
  SPIAutoTriggerFactory m_triggerFactory;
  std::atomic_bool m_initialized = false;
};

SPIAutoManager& GetSPIAutoManager();
void ShutdownSPIAuto() noexcept;

}  // namespace hal::vmx
