// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <utility>

#include "VMXConstants.h"
#include "hal/Interrupts.h"
#include "hal/Types.h"
#include "hal/handles/LimitedClassedHandleResource.h"

namespace hal::vmx {

enum class InterruptResult {
  kOk,
  kInvalidHandle,
  kInvalidSource,
  kUnsupportedSource,
  kOutOfRange,
  kNoResources,
  kAlreadyAllocated,
  kUnconfigured,
  kHardwareFailure,
};

enum class VMXInterruptEdge { kRising, kFalling, kBoth };

constexpr uint64_t InterruptRisingMask(int32_t index) noexcept {
  return uint64_t{1} << index;
}

constexpr uint64_t InterruptFallingMask(int32_t index) noexcept {
  return uint64_t{1} << (index + 8);
}

class InterruptCallbackState final {
 public:
  InterruptCallbackState(uint64_t risingMask, uint64_t fallingMask)
      : m_risingMask{risingMask}, m_fallingMask{fallingMask} {}

  void SetMasks(uint64_t risingMask, uint64_t fallingMask) noexcept {
    std::scoped_lock lock{m_mutex};
    m_risingMask = risingMask;
    m_fallingMask = fallingMask;
  }

  void SetActive(bool active) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_closed) {
      m_active = active;
    }
    if (!active) {
      m_cv.notify_all();
    }
  }

  void Close() noexcept {
    std::scoped_lock lock{m_mutex};
    m_active = false;
    m_closed = true;
    ++m_releaseGeneration;
    m_cv.notify_all();
  }

  // This is the only work performed by the VMX SDK callback: update the
  // native event sequence/timestamp and wake native waiters.
  void OnHardwareEvent(bool rising, uint64_t timestampUs) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_closed || !m_active) {
      return;
    }
    if (rising) {
      ++m_risingSequence;
      m_lastRisingTimestampUs = timestampUs;
    } else {
      ++m_fallingSequence;
      m_lastFallingTimestampUs = timestampUs;
    }
    m_cv.notify_all();
  }

  void ReleaseWaiting() noexcept {
    std::scoped_lock lock{m_mutex};
    ++m_releaseGeneration;
    m_cv.notify_all();
  }

  struct WaitResult {
    InterruptResult result = InterruptResult::kHardwareFailure;
    int64_t mask = 0;
  };

  WaitResult Wait(uint64_t eligibleMask, double timeout,
                  bool ignorePrevious) noexcept {
    if (!std::isfinite(timeout) && timeout != std::numeric_limits<double>::infinity()) {
      return {InterruptResult::kOutOfRange, 0};
    }
    if (timeout < 0.0) {
      return {InterruptResult::kOutOfRange, 0};
    }

    std::unique_lock lock{m_mutex};
    if (m_closed) {
      return {InterruptResult::kInvalidHandle, 0};
    }
    if (!m_active) {
      return {InterruptResult::kUnconfigured, 0};
    }
    if (eligibleMask == 0) {
      return {InterruptResult::kOk, 0};
    }

    if (ignorePrevious) {
      ConsumeCurrentLocked(eligibleMask);
    }
    const uint64_t releaseGeneration = m_releaseGeneration;
    auto pendingMask = PendingMaskLocked(eligibleMask);
    if (pendingMask != 0) {
      ConsumeMaskLocked(pendingMask);
      return {InterruptResult::kOk, static_cast<int64_t>(pendingMask)};
    }

    if (timeout == 0.0) {
      return {InterruptResult::kOk, 0};
    }

    auto ready = [&] {
      return m_closed || !m_active ||
             m_releaseGeneration != releaseGeneration ||
             PendingMaskLocked(eligibleMask) != 0;
    };
    bool woke = false;
    if (timeout == std::numeric_limits<double>::infinity()) {
      m_cv.wait(lock, ready);
      woke = true;
    } else {
      woke = m_cv.wait_for(lock, std::chrono::duration<double>{timeout}, ready);
    }
    if (!woke) {
      return {InterruptResult::kOk, 0};
    }
    if (m_closed) {
      return {InterruptResult::kInvalidHandle, 0};
    }
    if (!m_active) {
      return {InterruptResult::kUnconfigured, 0};
    }
    if (m_releaseGeneration != releaseGeneration) {
      return {InterruptResult::kOk, 0};
    }
    pendingMask = PendingMaskLocked(eligibleMask);
    if (pendingMask == 0) {
      return {InterruptResult::kOk, 0};
    }
    ConsumeMaskLocked(pendingMask);
    return {InterruptResult::kOk, static_cast<int64_t>(pendingMask)};
  }

 private:
  uint64_t PendingMaskLocked(uint64_t eligibleMask) const noexcept {
    uint64_t mask = 0;
    if ((eligibleMask & m_risingMask) != 0 &&
        m_risingSequence != m_consumedRisingSequence) {
      mask |= m_risingMask;
    }
    if ((eligibleMask & m_fallingMask) != 0 &&
        m_fallingSequence != m_consumedFallingSequence) {
      mask |= m_fallingMask;
    }
    return mask;
  }

  void ConsumeCurrentLocked(uint64_t eligibleMask) noexcept {
    if ((eligibleMask & m_risingMask) != 0) {
      m_consumedRisingSequence = m_risingSequence;
    }
    if ((eligibleMask & m_fallingMask) != 0) {
      m_consumedFallingSequence = m_fallingSequence;
    }
  }

  void ConsumeMaskLocked(uint64_t mask) noexcept {
    if ((mask & m_risingMask) != 0) {
      m_consumedRisingSequence = m_risingSequence;
    }
    if ((mask & m_fallingMask) != 0) {
      m_consumedFallingSequence = m_fallingSequence;
    }
  }

  mutable std::mutex m_mutex;
  std::condition_variable m_cv;
  bool m_active = false;
  bool m_closed = false;
  uint64_t m_risingSequence = 0;
  uint64_t m_fallingSequence = 0;
  uint64_t m_consumedRisingSequence = 0;
  uint64_t m_consumedFallingSequence = 0;
  uint64_t m_lastRisingTimestampUs = 0;
  uint64_t m_lastFallingTimestampUs = 0;
  uint64_t m_releaseGeneration = 0;
  uint64_t m_risingMask;
  uint64_t m_fallingMask;
};

class InterruptBackend {
 public:
  virtual ~InterruptBackend() = default;
  virtual bool SetEnabled(bool enabled) noexcept = 0;
  virtual bool GetEnabled(bool& enabled) noexcept = 0;
  virtual bool ReadTimestamp(bool rising, uint64_t& timestampUs) noexcept = 0;

  // Internal VMX resources (for example an Encoder index source) may need to
  // refer to the underlying interrupt resource without exposing another HAL
  // interrupt handle.  Test backends do not need to provide a resource handle.
  virtual uint16_t GetResourceHandle() const noexcept { return 0; }
};

using InterruptBackendFactory =
    std::function<std::unique_ptr<InterruptBackend>(
        int32_t channel, VMXInterruptEdge edge, InterruptCallbackState* state)>;

// Returns the same backend factory used by the public Interrupt HAL.  VMX
// adapters can use it to own an internal interrupt resource without consuming
// a public interrupt handle.
InterruptBackendFactory GetInterruptBackendFactory();

class InterruptPort final {
 public:
  InterruptPort(int32_t index, InterruptBackendFactory factory)
      : m_index{index},
        m_portMask{InterruptRisingMask(index) | InterruptFallingMask(index)},
        m_factory{std::move(factory)},
        m_state{std::make_shared<InterruptCallbackState>(
            InterruptRisingMask(index), InterruptFallingMask(index))} {}

  void SetIndex(int32_t index) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_closed && m_backend == nullptr) {
      m_index = index;
      m_portMask = InterruptRisingMask(index) | InterruptFallingMask(index);
      m_state->SetMasks(InterruptRisingMask(index),
                        InterruptFallingMask(index));
    }
  }

  ~InterruptPort() { Close(); }

  InterruptResult RequestSource(HAL_Handle source, int32_t channel) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_closed) {
      return InterruptResult::kInvalidHandle;
    }
    if (m_faulted) {
      return InterruptResult::kHardwareFailure;
    }
    if (m_source == source && m_channel == channel) {
      return m_backend ? InterruptResult::kOk : TryActivateLocked();
    }
    const auto oldSource = m_source;
    const auto oldChannel = m_channel;
    const bool hadBackend = m_backend != nullptr;
    StopBackendLocked();
    m_source = source;
    m_channel = channel;
    auto result = TryActivateLocked();
    if (result == InterruptResult::kOk) {
      return result;
    }
    m_source = oldSource;
    m_channel = oldChannel;
    if (hadBackend && TryActivateLocked() != InterruptResult::kOk) {
      m_faulted = true;
    }
    return result;
  }

  InterruptResult SetEdges(bool rising, bool falling) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_closed) {
      return InterruptResult::kInvalidHandle;
    }
    if (m_faulted) {
      return InterruptResult::kHardwareFailure;
    }
    if (!rising && !falling) {
      return InterruptResult::kOutOfRange;
    }
    if (m_rising == rising && m_falling == falling) {
      return m_backend ? InterruptResult::kOk : TryActivateLocked();
    }
    const bool oldRising = m_rising;
    const bool oldFalling = m_falling;
    const bool hadBackend = m_backend != nullptr;
    StopBackendLocked();
    m_rising = rising;
    m_falling = falling;
    auto result = TryActivateLocked();
    if (result == InterruptResult::kOk) {
      return result;
    }
    m_rising = oldRising;
    m_falling = oldFalling;
    if (hadBackend && TryActivateLocked() != InterruptResult::kOk) {
      m_faulted = true;
    }
    return result;
  }

  InterruptCallbackState::WaitResult Wait(double timeout, bool ignorePrevious,
                                           uint64_t mask) noexcept {
    uint64_t eligibleMask = 0;
    {
      std::scoped_lock lock{m_mutex};
      if (m_closed) {
        return {InterruptResult::kInvalidHandle, 0};
      }
      if (m_faulted) {
        return {InterruptResult::kHardwareFailure, 0};
      }
      if (!m_backend) {
        return {InterruptResult::kUnconfigured, 0};
      }
      eligibleMask = mask & m_portMask;
    }
    return m_state->Wait(eligibleMask, timeout, ignorePrevious);
  }

  std::pair<InterruptResult, uint64_t> ReadTimestamp(bool rising) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_closed) {
      return {InterruptResult::kInvalidHandle, 0};
    }
    if (!m_backend) {
      return {InterruptResult::kUnconfigured, 0};
    }
    uint64_t timestamp = 0;
    return m_backend->ReadTimestamp(rising, timestamp)
               ? std::pair{InterruptResult::kOk, timestamp}
               : std::pair{InterruptResult::kHardwareFailure, uint64_t{0}};
  }

  InterruptResult ReleaseWaiting() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_closed) {
      return InterruptResult::kInvalidHandle;
    }
    m_state->ReleaseWaiting();
    return InterruptResult::kOk;
  }

  void Close() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_closed) {
      return;
    }
    m_closed = true;
    m_state->Close();
    StopBackendLocked();
    m_source = HAL_kInvalidHandle;
    m_channel = -1;
  }

  HAL_Handle GetSource() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_source;
  }

  int32_t GetChannel() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_channel;
  }

  InterruptCallbackState* GetCallbackStateForTesting() const noexcept {
    return m_state.get();
  }

 private:
  InterruptResult TryActivateLocked() noexcept {
    if (m_source == HAL_kInvalidHandle || (!m_rising && !m_falling)) {
      return InterruptResult::kOk;
    }
    if (m_faulted) {
      return InterruptResult::kHardwareFailure;
    }
    std::unique_ptr<InterruptBackend> backend;
    try {
      backend = m_factory ? m_factory(m_channel, ToVMXEdge(), m_state.get())
                          : nullptr;
    } catch (...) {
      backend.reset();
    }
    if (!backend) {
      return InterruptResult::kHardwareFailure;
    }
    m_backend = std::move(backend);
    m_state->SetActive(true);
    return InterruptResult::kOk;
  }

  VMXInterruptEdge ToVMXEdge() const noexcept {
    if (m_rising && m_falling) {
      return VMXInterruptEdge::kBoth;
    }
    return m_rising ? VMXInterruptEdge::kRising : VMXInterruptEdge::kFalling;
  }

  void StopBackendLocked() noexcept {
    m_state->SetActive(false);
    m_backend.reset();
  }

  mutable std::mutex m_mutex;
  int32_t m_index;
  uint64_t m_portMask;
  HAL_Handle m_source = HAL_kInvalidHandle;
  int32_t m_channel = -1;
  // WPILib requests a source before selecting edges. Use the VMX default
  // rising edge so HAL_RequestInterrupts activates the resource immediately;
  // HAL_SetInterruptUpSourceEdge can then switch to falling or both.
  bool m_rising = true;
  bool m_falling = false;
  bool m_faulted = false;
  bool m_closed = false;
  InterruptBackendFactory m_factory;
  std::shared_ptr<InterruptCallbackState> m_state;
  std::unique_ptr<InterruptBackend> m_backend;
};

struct InterruptAllocationResult {
  HAL_InterruptHandle handle = HAL_kInvalidHandle;
  InterruptResult result = InterruptResult::kHardwareFailure;
};

constexpr int32_t kNumVMXInterrupts = 8;

class InterruptHandleResource final
    : public hal::LimitedClassedHandleResource<HAL_InterruptHandle,
                                               InterruptPort,
                                               kNumVMXInterrupts,
                                               HAL_HandleEnum::Interrupt> {
 public:
  InterruptHandleResource() { m_version = 0; }
};

class InterruptManager final {
 public:
  explicit InterruptManager(InterruptBackendFactory factory)
      : m_factory{std::move(factory)} {}

  InterruptAllocationResult Allocate(int32_t* index) {
    std::scoped_lock lock{m_mutex};
    auto port = std::make_shared<InterruptPort>(0, m_factory);
    auto handle = m_handles.Allocate(port);
    if (handle == HAL_kInvalidHandle) {
      return {HAL_kInvalidHandle, InterruptResult::kNoResources};
    }
    auto assignedIndex = m_handles.GetIndex(handle);
    // The handle index is the WPILib interrupt bit index.
    port->SetIndex(assignedIndex);
    if (index) {
      *index = assignedIndex;
    }
    return {handle, InterruptResult::kOk};
  }

  void Free(HAL_InterruptHandle handle) noexcept {
    std::scoped_lock lock{m_mutex};
    auto port = GetPort(handle);
    if (!port) {
      return;
    }
    port->Close();
    m_handles.Free(handle);
  }

  bool IsValid(HAL_InterruptHandle handle) {
    return GetPort(handle) != nullptr;
  }

  InterruptResult RequestSource(HAL_InterruptHandle handle, HAL_Handle source,
                                int32_t channel) {
    auto port = GetPort(handle);
    return port ? port->RequestSource(source, channel)
                : InterruptResult::kInvalidHandle;
  }

  InterruptResult SetEdges(HAL_InterruptHandle handle, bool rising,
                           bool falling) {
    auto port = GetPort(handle);
    return port ? port->SetEdges(rising, falling)
                : InterruptResult::kInvalidHandle;
  }

  InterruptCallbackState::WaitResult Wait(HAL_InterruptHandle handle,
                                           double timeout, bool ignorePrevious,
                                           uint64_t mask) {
    auto port = GetPort(handle);
    return port ? port->Wait(timeout, ignorePrevious, mask)
                : InterruptCallbackState::WaitResult{
                      InterruptResult::kInvalidHandle, 0};
  }

  std::pair<InterruptResult, uint64_t> ReadTimestamp(
      HAL_InterruptHandle handle, bool rising) {
    auto port = GetPort(handle);
    return port ? port->ReadTimestamp(rising)
                : std::pair{InterruptResult::kInvalidHandle, uint64_t{0}};
  }

  InterruptResult ReleaseWaiting(HAL_InterruptHandle handle) {
    auto port = GetPort(handle);
    return port ? port->ReleaseWaiting() : InterruptResult::kInvalidHandle;
  }

 private:
  std::shared_ptr<InterruptPort> GetPort(HAL_InterruptHandle handle) {
    return m_handles.Get(handle);
  }

  InterruptBackendFactory m_factory;
  std::mutex m_mutex;
  InterruptHandleResource m_handles;
};

}  // namespace hal::vmx
