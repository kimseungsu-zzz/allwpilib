// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <utility>

#include "VMXConstants.h"
#include "hal/Counter.h"
#include "hal/Types.h"
#include "hal/handles/LimitedClassedHandleResource.h"

namespace hal::vmx {

enum class CounterResult {
  kOk,
  kInvalidHandle,
  kInvalidSource,
  kUnsupportedSource,
  kUnsupportedMode,
  kUnsupported,
  kOutOfRange,
  kAlreadyAllocated,
  kNoResources,
  kUnconfigured,
  kHardwareFailure,
};

class CounterBackend {
 public:
  virtual ~CounterBackend() = default;
  virtual bool GetChannelCounts(uint32_t& channel1,
                                uint32_t& channel2) noexcept = 0;
  virtual bool GetInputStatus(bool& forward, bool& active) noexcept = 0;
  virtual bool Reset() noexcept = 0;
};

using CounterBackendFactory =
    std::function<std::unique_ptr<CounterBackend>(
        int32_t channelUp, int32_t channelDown, bool upRising, bool upFalling,
        bool downRising, bool downFalling)>;
using CounterModeBackendFactory = std::function<std::unique_ptr<CounterBackend>(
    HAL_Counter_Mode mode, int32_t channelUp, int32_t channelDown,
    bool upRising, bool upFalling, bool downRising, bool downFalling)>;
using CounterClock =
    std::function<std::chrono::steady_clock::time_point(void)>;

struct CounterSourceClaim {
  CounterResult result = CounterResult::kInvalidSource;
  int32_t channelUp = -1;
  int32_t channelDown = -1;
};

using CounterSourceClaimer =
    std::function<CounterSourceClaim(HAL_Handle sourceUp,
                                     HAL_Handle sourceDown)>;
using CounterSourceReleaser = std::function<void(
    HAL_Handle sourceUp, HAL_Handle sourceDown, int32_t channelUp,
    int32_t channelDown)>;

class CounterPort final {
 public:
  CounterPort(HAL_Counter_Mode mode, CounterBackendFactory factory,
              CounterSourceClaimer claimer, CounterSourceReleaser releaser,
              CounterClock clock,
              CounterModeBackendFactory modeFactory = {})
      : m_mode{mode},
        m_factory{std::move(factory)},
        m_modeFactory{std::move(modeFactory)},
        m_claimer{std::move(claimer)},
        m_releaser{std::move(releaser)},
        m_clock{std::move(clock)} {
    m_lastChange = m_clock ? m_clock() : std::chrono::steady_clock::time_point{};
  }

  ~CounterPort() { Close(); }

  CounterResult SetUpSource(HAL_Handle source, int32_t channel) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted) {
      return CounterResult::kHardwareFailure;
    }
    auto oldSource = m_upSource;
    auto oldChannel = m_upChannel;
    bool wasActive = m_backend != nullptr;
    auto result = ReplaceSourceLocked(true, source, channel);
    if (result != CounterResult::kOk) {
      m_upSource = oldSource;
      m_upChannel = oldChannel;
      if (wasActive && TryActivateLocked() != CounterResult::kOk) {
        m_faulted = true;
      }
    }
    return result;
  }

  CounterResult SetDownSource(HAL_Handle source, int32_t channel) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted) {
      return CounterResult::kHardwareFailure;
    }
    if (m_mode == HAL_Counter_kSemiperiod ||
        m_mode == HAL_Counter_kPulseLength) {
      return CounterResult::kUnsupportedMode;
    }
    auto oldSource = m_downSource;
    auto oldChannel = m_downChannel;
    bool wasActive = m_backend != nullptr;
    auto result = ReplaceSourceLocked(false, source, channel);
    if (result != CounterResult::kOk) {
      m_downSource = oldSource;
      m_downChannel = oldChannel;
      if (wasActive && TryActivateLocked() != CounterResult::kOk) {
        m_faulted = true;
      }
    }
    return result;
  }

  CounterResult ClearUpSource() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted) {
      return CounterResult::kHardwareFailure;
    }
    StopResourceLocked();
    m_upSource = HAL_kInvalidHandle;
    m_upChannel = -1;
    return CounterResult::kOk;
  }

  CounterResult ClearDownSource() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted) {
      return CounterResult::kHardwareFailure;
    }
    if (m_mode == HAL_Counter_kSemiperiod ||
        m_mode == HAL_Counter_kPulseLength) {
      return CounterResult::kUnsupportedMode;
    }
    StopResourceLocked();
    m_downSource = HAL_kInvalidHandle;
    m_downChannel = -1;
    return CounterResult::kOk;
  }

  CounterResult SetUpSourceEdge(bool rising, bool falling) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted) {
      return CounterResult::kHardwareFailure;
    }
    m_upRising = rising;
    m_upFalling = falling;
    return RecreateResourceLocked();
  }

  CounterResult SetDownSourceEdge(bool rising, bool falling) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted) {
      return CounterResult::kHardwareFailure;
    }
    if (m_mode != HAL_Counter_kTwoPulse &&
        m_mode != HAL_Counter_kExternalDirection) {
      return CounterResult::kUnsupportedMode;
    }
    m_downRising = rising;
    m_downFalling = falling;
    return RecreateResourceLocked();
  }

  CounterResult Reset() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted) {
      return CounterResult::kHardwareFailure;
    }
    if (m_backend && !m_backend->Reset()) {
      return CounterResult::kHardwareFailure;
    }
    m_countOffset = 0;
    m_hardwareCount = 0;
    m_lastChannel1 = 0;
    m_lastChannel2 = 0;
    m_hasHardwareSnapshot = false;
    m_hasObservedEvent = false;
    m_lastDirection = false;
    m_lastChange = m_clock();
    return CounterResult::kOk;
  }

  std::pair<CounterResult, int32_t> Get() noexcept {
    std::scoped_lock lock{m_mutex};
    auto result = RefreshLocked();
    if (result != CounterResult::kOk) {
      return {result, 0};
    }
    return {CounterResult::kOk, ClampToInt32(ReportedCountLocked())};
  }

  std::pair<CounterResult, double> GetPeriod() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsConfiguredLocked()) {
      return {CounterResult::kUnconfigured, 0.0};
    }
    auto result = RefreshLocked();
    if (result != CounterResult::kOk) {
      return {result, 0.0};
    }
    if (!m_hasObservedEvent || !m_hasPreviousEventTime) {
      return {CounterResult::kOk,
              std::numeric_limits<double>::infinity()};
    }
    return {CounterResult::kOk, m_lastPeriod};
  }

  CounterResult SetMaxPeriod(double maxPeriod) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted) {
      return CounterResult::kHardwareFailure;
    }
    if (!std::isfinite(maxPeriod) || maxPeriod <= 0.0) {
      return CounterResult::kOutOfRange;
    }
    m_maxPeriod = maxPeriod;
    return CounterResult::kOk;
  }

  std::pair<CounterResult, bool> GetStopped() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsConfiguredLocked()) {
      return {CounterResult::kUnconfigured, false};
    }
    auto result = RefreshLocked();
    if (result != CounterResult::kOk) {
      return {result, false};
    }
    if (!m_hasObservedEvent) {
      return {CounterResult::kOk, true};
    }
    return {CounterResult::kOk,
            std::chrono::duration<double>(m_clock() - m_lastChange).count() >
                m_maxPeriod};
  }

  std::pair<CounterResult, bool> GetDirection() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsConfiguredLocked()) {
      return {CounterResult::kUnconfigured, false};
    }
    auto result = RefreshLocked();
    if (result != CounterResult::kOk) {
      return {result, false};
    }
    return {CounterResult::kOk, m_lastDirection != m_reverseDirection};
  }

  CounterResult SetReverseDirection(bool reverse) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted) {
      return CounterResult::kHardwareFailure;
    }
    m_reverseDirection = reverse;
    return CounterResult::kOk;
  }

  std::pair<CounterResult, int32_t> GetSamplesToAverage() const noexcept {
    std::scoped_lock lock{m_mutex};
    return {CounterResult::kUnsupported, 0};
  }

  CounterResult SetSamplesToAverage(int32_t samples) noexcept {
    std::scoped_lock lock{m_mutex};
    if (samples < 1 || samples > 127) {
      return CounterResult::kOutOfRange;
    }
    return CounterResult::kUnsupported;
  }

  CounterResult SetAverageSize(int32_t size) noexcept {
    std::scoped_lock lock{m_mutex};
    if (size < 1 || size > 127) {
      return CounterResult::kOutOfRange;
    }
    return CounterResult::kUnsupported;
  }

  CounterResult SetUpdateWhenEmpty(bool enabled) noexcept {
    static_cast<void>(enabled);
    std::scoped_lock lock{m_mutex};
    return CounterResult::kUnsupported;
  }

  CounterResult SetTwoPulseMode() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_mode == HAL_Counter_kTwoPulse) {
      return CounterResult::kOk;
    }
    if (m_faulted) {
      return CounterResult::kHardwareFailure;
    }
    m_mode = HAL_Counter_kTwoPulse;
    return RecreateResourceLocked();
  }

  CounterResult SetSemiPeriodMode(bool highSemiPeriod) noexcept {
    std::scoped_lock lock{m_mutex};
    static_cast<void>(highSemiPeriod);
    if (m_faulted) {
      return CounterResult::kHardwareFailure;
    }
    StopResourceLocked();
    m_mode = HAL_Counter_kSemiperiod;
    m_downSource = HAL_kInvalidHandle;
    m_downChannel = -1;
    return TryActivateLocked();
  }

  CounterResult SetExternalDirectionMode() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted) {
      return CounterResult::kHardwareFailure;
    }
    StopResourceLocked();
    m_mode = HAL_Counter_kExternalDirection;
    return TryActivateLocked();
  }

  CounterResult SetUnsupportedMode() noexcept {
    std::scoped_lock lock{m_mutex};
    return CounterResult::kUnsupportedMode;
  }

  void Close() noexcept {
    std::scoped_lock lock{m_mutex};
    StopResourceLocked();
    m_faulted = false;
  }

 private:
  bool IsConfiguredLocked() const noexcept {
    return m_backend != nullptr;
  }

  static int32_t ClampToInt32(int64_t value) noexcept {
    if (value > std::numeric_limits<int32_t>::max()) {
      return std::numeric_limits<int32_t>::max();
    }
    if (value < std::numeric_limits<int32_t>::min()) {
      return std::numeric_limits<int32_t>::min();
    }
    return static_cast<int32_t>(value);
  }

  static int64_t Delta(uint32_t current, uint32_t previous) noexcept {
    return static_cast<int32_t>(current - previous);
  }

  int64_t ReportedCountLocked() const noexcept {
    auto count = m_countOffset + m_hardwareCount;
    return m_reverseDirection ? -count : count;
  }

  CounterResult RefreshLocked() noexcept {
    if (!IsConfiguredLocked()) {
      return CounterResult::kUnconfigured;
    }
    uint32_t channel1 = 0;
    uint32_t channel2 = 0;
    if (!m_backend->GetChannelCounts(channel1, channel2)) {
      return CounterResult::kHardwareFailure;
    }
    auto now = m_clock();
    if (!m_hasHardwareSnapshot) {
      m_lastChannel1 = channel1;
      m_lastChannel2 = channel2;
      m_hasHardwareSnapshot = true;
      return CounterResult::kOk;
    }
    auto upDelta = m_upRising || m_upFalling
                       ? Delta(channel1, m_lastChannel1)
                       : int64_t{0};
    auto downDelta = m_downRising || m_downFalling
                         ? Delta(channel2, m_lastChannel2)
                         : int64_t{0};
    m_lastChannel1 = channel1;
    m_lastChannel2 = channel2;
    auto effectiveDelta = upDelta - downDelta;
    if (effectiveDelta != 0) {
      m_hardwareCount += effectiveDelta;
      m_lastDirection = effectiveDelta > 0;
      if (m_hasPreviousEventTime) {
        m_lastPeriod =
            std::chrono::duration<double>(now - m_previousEventTime).count() /
            static_cast<double>(std::abs(effectiveDelta));
      }
      m_previousEventTime = now;
      m_hasPreviousEventTime = true;
      m_lastChange = now;
      m_hasObservedEvent = true;
    }
    return CounterResult::kOk;
  }

  CounterResult ReplaceSourceLocked(bool up, HAL_Handle source,
                                    int32_t channel) noexcept {
    if (m_backend) {
      StopResourceLocked();
    }
    if (up) {
      m_upSource = source;
      m_upChannel = channel;
    } else {
      m_downSource = source;
      m_downChannel = channel;
    }
    auto result = TryActivateLocked();
    return result;
  }

  CounterResult TryActivateLocked() noexcept {
    if (m_upSource == HAL_kInvalidHandle) {
      return CounterResult::kOk;
    }
    if (m_mode == HAL_Counter_kPulseLength) {
      return CounterResult::kUnsupportedMode;
    }
    if (m_mode == HAL_Counter_kSemiperiod &&
        m_downSource != HAL_kInvalidHandle) {
      return CounterResult::kUnsupportedSource;
    }
    if ((m_mode == HAL_Counter_kTwoPulse ||
         m_mode == HAL_Counter_kExternalDirection) &&
        m_downSource == HAL_kInvalidHandle) {
      return CounterResult::kOk;
    }
    if (m_mode == HAL_Counter_kTwoPulse && m_upSource != m_downSource) {
      // VMX has one hardware up/down input capture resource; two independent
      // source handles are not a supported TwoPulse configuration.
      return CounterResult::kUnsupportedSource;
    }
    if (m_mode == HAL_Counter_kExternalDirection &&
        m_upSource == m_downSource) {
      return CounterResult::kUnsupportedSource;
    }
    CounterSourceClaim claim;
    const HAL_Handle claimDownSource =
        m_mode == HAL_Counter_kSemiperiod ? m_upSource : m_downSource;
    try {
      claim = m_claimer(m_upSource, claimDownSource);
    } catch (...) {
      return CounterResult::kHardwareFailure;
    }
    if (claim.result != CounterResult::kOk) {
      return claim.result;
    }
    auto result = ActivateBackendLocked(claim.channelUp, claim.channelDown);
    if (result != CounterResult::kOk) {
      try {
        m_releaser(m_upSource, claimDownSource, claim.channelUp,
                   claim.channelDown);
      } catch (...) {
      }
    }
    return result;
  }

  CounterResult ActivateBackendLocked(int32_t channelUp,
                                      int32_t channelDown) noexcept {
    std::unique_ptr<CounterBackend> backend;
    try {
      if (m_modeFactory) {
        backend = m_modeFactory(m_mode, channelUp, channelDown, m_upRising,
                                m_upFalling, m_downRising, m_downFalling);
      } else {
        backend = m_factory(channelUp, channelDown, m_upRising, m_upFalling,
                            m_downRising, m_downFalling);
      }
    } catch (...) {
      backend.reset();
    }
    if (!backend) {
      return CounterResult::kHardwareFailure;
    }
    m_backend = std::move(backend);
    m_claimedChannelUp = channelUp;
    m_claimedChannelDown = channelDown;
    m_hasHardwareSnapshot = false;
    m_hardwareCount = 0;
    m_lastChange = m_clock();
    return CounterResult::kOk;
  }

  CounterResult RecreateResourceLocked() noexcept {
    if (!m_backend) {
      return CounterResult::kOk;
    }
    auto refreshResult = RefreshLocked();
    if (refreshResult != CounterResult::kOk) {
      return refreshResult;
    }
    auto current = m_countOffset + m_hardwareCount;
    m_countOffset = current;
    m_hardwareCount = 0;
    m_backend.reset();
    auto result =
        ActivateBackendLocked(m_claimedChannelUp, m_claimedChannelDown);
    if (result != CounterResult::kOk) {
      m_faulted = true;
    }
    return result;
  }

  void StopResourceLocked() noexcept {
    if (m_backend) {
      RefreshLocked();
      // Keep the last known hardware delta even if the final read fails while
      // the resource is being torn down. A failed read must not make a later
      // resource activation appear to lose count continuity.
      m_countOffset += m_hardwareCount;
      m_hardwareCount = 0;
      m_backend.reset();
    }
    const HAL_Handle releaseDownSource =
        m_mode == HAL_Counter_kSemiperiod ? m_upSource : m_downSource;
    if (m_upSource != HAL_kInvalidHandle &&
        releaseDownSource != HAL_kInvalidHandle &&
        m_claimedChannelUp >= 0 && m_claimedChannelDown >= 0) {
      try {
        m_releaser(m_upSource, releaseDownSource, m_claimedChannelUp,
                   m_claimedChannelDown);
      } catch (...) {
      }
    }
    m_claimedChannelUp = -1;
    m_claimedChannelDown = -1;
    m_hasHardwareSnapshot = false;
  }

  mutable std::mutex m_mutex;
  HAL_Counter_Mode m_mode;
  HAL_Handle m_upSource = HAL_kInvalidHandle;
  HAL_Handle m_downSource = HAL_kInvalidHandle;
  int32_t m_upChannel = -1;
  int32_t m_downChannel = -1;
  int32_t m_claimedChannelUp = -1;
  int32_t m_claimedChannelDown = -1;
  bool m_upRising = true;
  bool m_upFalling = false;
  bool m_downRising = true;
  bool m_downFalling = false;
  bool m_reverseDirection = false;
  bool m_faulted = false;
  double m_maxPeriod = 0.5;
  int64_t m_countOffset = 0;
  int64_t m_hardwareCount = 0;
  uint32_t m_lastChannel1 = 0;
  uint32_t m_lastChannel2 = 0;
  bool m_hasHardwareSnapshot = false;
  bool m_hasObservedEvent = false;
  bool m_lastDirection = false;
  bool m_hasPreviousEventTime = false;
  std::chrono::steady_clock::time_point m_lastChange;
  std::chrono::steady_clock::time_point m_previousEventTime;
  double m_lastPeriod = std::numeric_limits<double>::infinity();
  CounterBackendFactory m_factory;
  CounterModeBackendFactory m_modeFactory;
  CounterSourceClaimer m_claimer;
  CounterSourceReleaser m_releaser;
  CounterClock m_clock;
  std::unique_ptr<CounterBackend> m_backend;
};

struct CounterAllocationResult {
  HAL_CounterHandle handle = HAL_kInvalidHandle;
  CounterResult result = CounterResult::kHardwareFailure;
};

constexpr int32_t kNumVMXCounters = 8;

class CounterHandleResource final
    : public hal::LimitedClassedHandleResource<HAL_CounterHandle, CounterPort,
                                               kNumVMXCounters,
                                               HAL_HandleEnum::Counter> {
 public:
  CounterHandleResource() { m_version = 0; }
};

class CounterManager final {
 public:
  CounterManager(CounterBackendFactory factory, CounterSourceClaimer claimer,
                 CounterSourceReleaser releaser,
                 CounterClock clock = std::chrono::steady_clock::now,
                 CounterModeBackendFactory modeFactory = {})
      : m_factory{std::move(factory)},
        m_claimer{std::move(claimer)},
        m_releaser{std::move(releaser)},
        m_clock{std::move(clock)},
        m_modeFactory{std::move(modeFactory)} {}

  CounterAllocationResult Allocate(HAL_Counter_Mode mode, int32_t* index) {
    std::scoped_lock lock{m_allocationMutex};
    if (mode == HAL_Counter_kPulseLength) {
      return {HAL_kInvalidHandle, CounterResult::kUnsupportedMode};
    }
    auto port = std::make_shared<CounterPort>(
        mode, m_factory, m_claimer, m_releaser, m_clock, m_modeFactory);
    auto handle = m_handles.Allocate(port);
    if (handle == HAL_kInvalidHandle) {
      return {HAL_kInvalidHandle, CounterResult::kNoResources};
    }
    if (index) {
      *index = m_handles.GetIndex(handle);
    }
    return {handle, CounterResult::kOk};
  }

  void Free(HAL_CounterHandle handle) noexcept {
    std::scoped_lock lock{m_allocationMutex};
    auto port = GetPort(handle);
    if (!port) {
      return;
    }
    port->Close();
    m_handles.Free(handle);
  }

#define VMX_COUNTER_PAIR(name, type)                                      \
  std::pair<CounterResult, type> name(HAL_CounterHandle handle) {         \
    auto port = GetPort(handle);                                          \
    return port ? port->name()                                           \
                : std::pair{CounterResult::kInvalidHandle, type{}};      \
  }

  VMX_COUNTER_PAIR(Get, int32_t)
  VMX_COUNTER_PAIR(GetPeriod, double)
  VMX_COUNTER_PAIR(GetStopped, bool)
  VMX_COUNTER_PAIR(GetDirection, bool)
  VMX_COUNTER_PAIR(GetSamplesToAverage, int32_t)

#undef VMX_COUNTER_PAIR

#define VMX_COUNTER_SET(name, type)                                     \
  CounterResult name(HAL_CounterHandle handle, type value) {             \
    auto port = GetPort(handle);                                         \
    return port ? port->name(value) : CounterResult::kInvalidHandle;     \
  }

  VMX_COUNTER_SET(SetMaxPeriod, double)
  VMX_COUNTER_SET(SetReverseDirection, bool)
  VMX_COUNTER_SET(SetSamplesToAverage, int32_t)
  VMX_COUNTER_SET(SetAverageSize, int32_t)
  VMX_COUNTER_SET(SetUpdateWhenEmpty, bool)

#undef VMX_COUNTER_SET

  CounterResult SetUpSource(HAL_CounterHandle handle, HAL_Handle source,
                            int32_t channel) {
    auto port = GetPort(handle);
    return port ? port->SetUpSource(source, channel)
                : CounterResult::kInvalidHandle;
  }
  CounterResult SetDownSource(HAL_CounterHandle handle, HAL_Handle source,
                              int32_t channel) {
    auto port = GetPort(handle);
    return port ? port->SetDownSource(source, channel)
                : CounterResult::kInvalidHandle;
  }
  CounterResult SetUpSourceEdge(HAL_CounterHandle handle, bool rising,
                                bool falling) {
    auto port = GetPort(handle);
    return port ? port->SetUpSourceEdge(rising, falling)
                : CounterResult::kInvalidHandle;
  }
  CounterResult SetDownSourceEdge(HAL_CounterHandle handle, bool rising,
                                  bool falling) {
    auto port = GetPort(handle);
    return port ? port->SetDownSourceEdge(rising, falling)
                : CounterResult::kInvalidHandle;
  }
  CounterResult ClearUpSource(HAL_CounterHandle handle) {
    auto port = GetPort(handle);
    return port ? port->ClearUpSource() : CounterResult::kInvalidHandle;
  }
  CounterResult ClearDownSource(HAL_CounterHandle handle) {
    auto port = GetPort(handle);
    return port ? port->ClearDownSource() : CounterResult::kInvalidHandle;
  }
  CounterResult Reset(HAL_CounterHandle handle) {
    auto port = GetPort(handle);
    return port ? port->Reset() : CounterResult::kInvalidHandle;
  }

  CounterResult SetUnsupportedMode(HAL_CounterHandle handle) {
    auto port = GetPort(handle);
    return port ? port->SetUnsupportedMode() : CounterResult::kInvalidHandle;
  }

  CounterResult SetSemiPeriodMode(HAL_CounterHandle handle,
                                  bool highSemiPeriod) {
    auto port = GetPort(handle);
    return port ? port->SetSemiPeriodMode(highSemiPeriod)
                : CounterResult::kInvalidHandle;
  }

  CounterResult SetExternalDirectionMode(HAL_CounterHandle handle) {
    auto port = GetPort(handle);
    return port ? port->SetExternalDirectionMode()
                : CounterResult::kInvalidHandle;
  }

  CounterResult SetTwoPulseMode(HAL_CounterHandle handle) {
    auto port = GetPort(handle);
    return port ? port->SetTwoPulseMode() : CounterResult::kInvalidHandle;
  }

 private:
  std::shared_ptr<CounterPort> GetPort(HAL_CounterHandle handle) {
    return m_handles.Get(handle);
  }

  CounterBackendFactory m_factory;
  CounterSourceClaimer m_claimer;
  CounterSourceReleaser m_releaser;
  CounterClock m_clock;
  CounterModeBackendFactory m_modeFactory;
  std::mutex m_allocationMutex;
  CounterHandleResource m_handles;
};

}  // namespace hal::vmx
