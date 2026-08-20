// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string_view>
#include <utility>

#include "VMXConstants.h"
#include "hal/Encoder.h"
#include "hal/Types.h"
#include "hal/handles/LimitedClassedHandleResource.h"

namespace hal::vmx {

enum class EncoderResult {
  kOk,
  kInvalidHandle,
  kInvalidSource,
  kUnsupportedSource,
  kInvalidEncoding,
  kOutOfRange,
  kAlreadyAllocated,
  kNoResources,
  kHardwareFailure,
  kUnsupported,
};

enum class VMXEncoderEdge { k1X, k2X, k4X };

class EncoderBackend {
 public:
  virtual ~EncoderBackend() = default;
  virtual bool GetCount(int32_t& count) noexcept = 0;
  virtual bool GetDirection(bool& forward) noexcept = 0;
  virtual bool Reset() noexcept = 0;
  virtual bool GetPeriodMicroseconds(uint16_t& period) noexcept = 0;
};

using EncoderBackendFactory =
    std::function<std::unique_ptr<EncoderBackend>(int32_t channelA,
                                                  int32_t channelB,
                                                  VMXEncoderEdge edge)>;
using EncoderClock =
    std::function<std::chrono::steady_clock::time_point(void)>;

struct EncoderSourceClaim {
  EncoderResult result = EncoderResult::kInvalidSource;
  int32_t channelA = -1;
  int32_t channelB = -1;
};

using EncoderSourceClaimer =
    std::function<EncoderSourceClaim(HAL_Handle sourceA, HAL_Handle sourceB)>;
using EncoderSourceReleaser = std::function<void(
    HAL_Handle sourceA, HAL_Handle sourceB, int32_t channelA, int32_t channelB)>;

constexpr int32_t EncodingScale(HAL_EncoderEncodingType type) noexcept {
  switch (type) {
    case HAL_Encoder_k1X:
      return 1;
    case HAL_Encoder_k2X:
      return 2;
    case HAL_Encoder_k4X:
      return 4;
    default:
      return 0;
  }
}

constexpr double DecodingScale(HAL_EncoderEncodingType type) noexcept {
  int32_t scale = EncodingScale(type);
  return scale == 0 ? 0.0 : 1.0 / scale;
}

constexpr VMXEncoderEdge ToVMXEncoderEdge(
    HAL_EncoderEncodingType type) noexcept {
  switch (type) {
    case HAL_Encoder_k1X:
      return VMXEncoderEdge::k1X;
    case HAL_Encoder_k2X:
      return VMXEncoderEdge::k2X;
    default:
      return VMXEncoderEdge::k4X;
  }
}

class EncoderPort final {
 public:
  bool Initialize(HAL_Handle sourceA, HAL_Handle sourceB, int32_t channelA,
                  int32_t channelB, bool reverseDirection,
                  HAL_EncoderEncodingType encodingType,
                  EncoderBackendFactory factory, EncoderClock clock) noexcept {
    std::scoped_lock lock{m_mutex};
    m_sourceA = sourceA;
    m_sourceB = sourceB;
    m_channelA = channelA;
    m_channelB = channelB;
    m_reverseDirection = reverseDirection;
    m_encodingType = encodingType;
    m_clock = std::move(clock);
    try {
      m_backend = factory ? factory(channelA, channelB,
                                    ToVMXEncoderEdge(encodingType))
                          : nullptr;
    } catch (...) {
      m_backend.reset();
    }
    if (!m_backend || !m_clock) {
      m_backend.reset();
      return false;
    }
    m_lastChange = m_clock();
    m_lastObservedHardwareCount = 0;
    m_hasObservedCount = false;
    return true;
  }

  std::pair<EncoderResult, int32_t> GetRaw() noexcept {
    std::scoped_lock lock{m_mutex};
    int32_t raw = 0;
    auto result = RefreshCountLocked(raw);
    return {result, result == EncoderResult::kOk ? raw : 0};
  }

  std::pair<EncoderResult, int32_t> Get() noexcept {
    std::scoped_lock lock{m_mutex};
    int32_t raw = 0;
    auto result = RefreshCountLocked(raw);
    return {result,
            result == EncoderResult::kOk ? raw / EncodingScale(m_encodingType)
                                         : 0};
  }

  std::pair<EncoderResult, int32_t> GetEncodingScale() const noexcept {
    std::scoped_lock lock{m_mutex};
    return IsUsable()
               ? std::pair{EncoderResult::kOk, EncodingScale(m_encodingType)}
               : std::pair{EncoderResult::kHardwareFailure, 0};
  }

  EncoderResult Reset() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsUsable() || !m_backend->Reset()) {
      return EncoderResult::kHardwareFailure;
    }
    m_lastObservedHardwareCount = 0;
    m_hasObservedCount = true;
    m_lastChange = m_clock();
    return EncoderResult::kOk;
  }

  std::pair<EncoderResult, double> GetPeriod() noexcept {
    std::scoped_lock lock{m_mutex};
    return GetPeriodLocked();
  }

  EncoderResult SetMaxPeriod(double maxPeriod) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsUsable()) {
      return EncoderResult::kHardwareFailure;
    }
    if (!std::isfinite(maxPeriod) || maxPeriod <= 0.0) {
      return EncoderResult::kOutOfRange;
    }
    m_maxPeriod = maxPeriod;
    return EncoderResult::kOk;
  }

  std::pair<EncoderResult, bool> GetStopped() noexcept {
    std::scoped_lock lock{m_mutex};
    int32_t ignored = 0;
    auto result = RefreshCountLocked(ignored);
    if (result != EncoderResult::kOk) {
      return {result, false};
    }
    return {EncoderResult::kOk,
            std::chrono::duration<double>(m_clock() - m_lastChange).count() >
                m_maxPeriod};
  }

  std::pair<EncoderResult, bool> GetDirection() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsUsable()) {
      return {EncoderResult::kHardwareFailure, false};
    }
    bool forward = false;
    if (!m_backend->GetDirection(forward)) {
      return {EncoderResult::kHardwareFailure, false};
    }
    return {EncoderResult::kOk, forward != m_reverseDirection};
  }

  EncoderResult SetDistancePerPulse(double distancePerPulse) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsUsable()) {
      return EncoderResult::kHardwareFailure;
    }
    if (!std::isfinite(distancePerPulse) || distancePerPulse == 0.0) {
      return EncoderResult::kOutOfRange;
    }
    m_distancePerPulse = distancePerPulse;
    return EncoderResult::kOk;
  }

  std::pair<EncoderResult, double> GetDistancePerPulse() const noexcept {
    std::scoped_lock lock{m_mutex};
    return IsUsable()
               ? std::pair{EncoderResult::kOk, m_distancePerPulse}
               : std::pair{EncoderResult::kHardwareFailure, 0.0};
  }

  std::pair<EncoderResult, double> GetDistance() noexcept {
    std::scoped_lock lock{m_mutex};
    int32_t raw = 0;
    auto result = RefreshCountLocked(raw);
    return {result, result == EncoderResult::kOk
                        ? static_cast<double>(raw) *
                              DecodingScale(m_encodingType) * m_distancePerPulse
                        : 0.0};
  }

  std::pair<EncoderResult, double> GetRate() noexcept {
    std::scoped_lock lock{m_mutex};
    auto [periodResult, period] = GetPeriodLocked();
    if (periodResult != EncoderResult::kOk) {
      return {periodResult, 0.0};
    }
    bool forward = false;
    if (!m_backend->GetDirection(forward)) {
      return {EncoderResult::kHardwareFailure, 0.0};
    }
    if (period <= 0.0) {
      return {EncoderResult::kOk, 0.0};
    }
    double sign = (forward != m_reverseDirection) ? 1.0 : -1.0;
    return {EncoderResult::kOk, sign * m_distancePerPulse / period};
  }

  EncoderResult SetMinRate(double minRate) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsUsable()) {
      return EncoderResult::kHardwareFailure;
    }
    if (!std::isfinite(minRate) || minRate <= 0.0) {
      return EncoderResult::kOutOfRange;
    }
    m_maxPeriod = std::abs(m_distancePerPulse / minRate);
    return EncoderResult::kOk;
  }

  EncoderResult SetReverseDirection(bool reverseDirection) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsUsable()) {
      return EncoderResult::kHardwareFailure;
    }
    m_reverseDirection = reverseDirection;
    return EncoderResult::kOk;
  }

  EncoderResult SetSamplesToAverage(int32_t samples) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsUsable()) {
      return EncoderResult::kHardwareFailure;
    }
    if (samples < 1 || samples > 127) {
      return EncoderResult::kOutOfRange;
    }
    return EncoderResult::kUnsupported;
  }

  std::pair<EncoderResult, int32_t> GetSamplesToAverage() const noexcept {
    std::scoped_lock lock{m_mutex};
    return IsUsable() ? std::pair{EncoderResult::kUnsupported, 0}
                      : std::pair{EncoderResult::kHardwareFailure, 0};
  }

  std::pair<EncoderResult, double> GetDecodingScale() const noexcept {
    std::scoped_lock lock{m_mutex};
    return IsUsable()
               ? std::pair{EncoderResult::kOk,
                           DecodingScale(m_encodingType)}
               : std::pair{EncoderResult::kHardwareFailure, 0.0};
  }

  std::pair<EncoderResult, HAL_EncoderEncodingType> GetEncodingType()
      const noexcept {
    std::scoped_lock lock{m_mutex};
    return IsUsable()
               ? std::pair{EncoderResult::kOk, m_encodingType}
               : std::pair{EncoderResult::kHardwareFailure,
                           HAL_Encoder_k4X};
  }

  HAL_Handle GetSourceA() const noexcept { return m_sourceA; }
  HAL_Handle GetSourceB() const noexcept { return m_sourceB; }
  int32_t GetChannelA() const noexcept { return m_channelA; }
  int32_t GetChannelB() const noexcept { return m_channelB; }

  void Close() noexcept {
    std::scoped_lock lock{m_mutex};
    m_backend.reset();
  }

 private:
  bool IsUsable() const noexcept { return m_backend != nullptr; }

  EncoderResult RefreshCountLocked(int32_t& adjustedCount) noexcept {
    adjustedCount = 0;
    if (!IsUsable()) {
      return EncoderResult::kHardwareFailure;
    }
    int32_t hardwareCount = 0;
    if (!m_backend->GetCount(hardwareCount)) {
      return EncoderResult::kHardwareFailure;
    }
    if (!m_hasObservedCount || hardwareCount != m_lastObservedHardwareCount) {
      m_lastObservedHardwareCount = hardwareCount;
      m_hasObservedCount = true;
      m_lastChange = m_clock();
    }
    adjustedCount = m_reverseDirection ? -hardwareCount : hardwareCount;
    return EncoderResult::kOk;
  }

  std::pair<EncoderResult, double> GetPeriodLocked() noexcept {
    if (!IsUsable()) {
      return {EncoderResult::kHardwareFailure, 0.0};
    }
    uint16_t microseconds = 0;
    if (!m_backend->GetPeriodMicroseconds(microseconds)) {
      return {EncoderResult::kHardwareFailure, 0.0};
    }
    return {EncoderResult::kOk,
            static_cast<double>(microseconds) / 1.0e6};
  }

  mutable std::mutex m_mutex;
  HAL_Handle m_sourceA = HAL_kInvalidHandle;
  HAL_Handle m_sourceB = HAL_kInvalidHandle;
  int32_t m_channelA = -1;
  int32_t m_channelB = -1;
  bool m_reverseDirection = false;
  HAL_EncoderEncodingType m_encodingType = HAL_Encoder_k4X;
  double m_distancePerPulse = 1.0;
  double m_maxPeriod = 0.5;
  int32_t m_lastObservedHardwareCount = 0;
  bool m_hasObservedCount = false;
  std::chrono::steady_clock::time_point m_lastChange;
  EncoderClock m_clock;
  std::unique_ptr<EncoderBackend> m_backend;
};

struct EncoderAllocationResult {
  HAL_EncoderHandle handle = HAL_kInvalidHandle;
  EncoderResult result = EncoderResult::kHardwareFailure;
};

class EncoderHandleResource final
    : public hal::LimitedClassedHandleResource<HAL_EncoderHandle, EncoderPort,
                                               kNumVMXEncoders,
                                               HAL_HandleEnum::Encoder> {
 public:
  EncoderHandleResource() { m_version = 0; }
};

class EncoderManager final {
 public:
  EncoderManager(EncoderBackendFactory factory, EncoderSourceClaimer claimer,
                 EncoderSourceReleaser releaser,
                 EncoderClock clock = std::chrono::steady_clock::now)
      : m_factory{std::move(factory)},
        m_claimer{std::move(claimer)},
        m_releaser{std::move(releaser)},
        m_clock{std::move(clock)} {}

  EncoderAllocationResult Allocate(
      HAL_Handle sourceA, HAL_Handle sourceB, bool reverseDirection,
      HAL_EncoderEncodingType encodingType) noexcept {
    std::scoped_lock allocationLock{m_allocationMutex};
    if (EncodingScale(encodingType) == 0) {
      return {HAL_kInvalidHandle, EncoderResult::kInvalidEncoding};
    }
    auto port = std::make_shared<EncoderPort>();
    auto handle = m_handles.Allocate(port);
    if (handle == HAL_kInvalidHandle) {
      return {HAL_kInvalidHandle, EncoderResult::kNoResources};
    }
    EncoderSourceClaim claim;
    try {
      claim = m_claimer ? m_claimer(sourceA, sourceB) : EncoderSourceClaim{};
    } catch (...) {
      claim = {};
    }
    if (claim.result != EncoderResult::kOk) {
      m_handles.Free(handle);
      return {HAL_kInvalidHandle, claim.result};
    }
    if (!port->Initialize(sourceA, sourceB, claim.channelA, claim.channelB,
                          reverseDirection, encodingType, m_factory, m_clock)) {
      try {
        m_releaser(sourceA, sourceB, claim.channelA, claim.channelB);
      } catch (...) {
      }
      m_handles.Free(handle);
      return {HAL_kInvalidHandle, EncoderResult::kHardwareFailure};
    }
    return {handle, EncoderResult::kOk};
  }

  void Free(HAL_EncoderHandle handle) noexcept {
    std::scoped_lock allocationLock{m_allocationMutex};
    auto port = GetPort(handle);
    if (!port) {
      return;
    }
    HAL_Handle sourceA = port->GetSourceA();
    HAL_Handle sourceB = port->GetSourceB();
    int32_t channelA = port->GetChannelA();
    int32_t channelB = port->GetChannelB();
    port->Close();
    m_handles.Free(handle);
    try {
      m_releaser(sourceA, sourceB, channelA, channelB);
    } catch (...) {
    }
  }

#define VMX_ENCODER_MANAGER_PAIR(name, type)                         \
  std::pair<EncoderResult, type> name(HAL_EncoderHandle handle) {    \
    auto port = GetPort(handle);                                     \
    return port ? port->name()                                       \
                : std::pair{EncoderResult::kInvalidHandle, type{}};  \
  }

  VMX_ENCODER_MANAGER_PAIR(Get, int32_t)
  VMX_ENCODER_MANAGER_PAIR(GetRaw, int32_t)
  VMX_ENCODER_MANAGER_PAIR(GetEncodingScale, int32_t)
  VMX_ENCODER_MANAGER_PAIR(GetPeriod, double)
  VMX_ENCODER_MANAGER_PAIR(GetStopped, bool)
  VMX_ENCODER_MANAGER_PAIR(GetDirection, bool)
  VMX_ENCODER_MANAGER_PAIR(GetDistance, double)
  VMX_ENCODER_MANAGER_PAIR(GetRate, double)
  VMX_ENCODER_MANAGER_PAIR(GetDistancePerPulse, double)
  VMX_ENCODER_MANAGER_PAIR(GetSamplesToAverage, int32_t)
  VMX_ENCODER_MANAGER_PAIR(GetDecodingScale, double)

#undef VMX_ENCODER_MANAGER_PAIR

  std::pair<EncoderResult, HAL_EncoderEncodingType> GetEncodingType(
      HAL_EncoderHandle handle) {
    auto port = GetPort(handle);
    return port ? port->GetEncodingType()
                : std::pair{EncoderResult::kInvalidHandle, HAL_Encoder_k4X};
  }

#define VMX_ENCODER_MANAGER_SET(name, type)                        \
  EncoderResult name(HAL_EncoderHandle handle, type value) {       \
    auto port = GetPort(handle);                                   \
    return port ? port->name(value) : EncoderResult::kInvalidHandle; \
  }

  VMX_ENCODER_MANAGER_SET(SetMaxPeriod, double)
  VMX_ENCODER_MANAGER_SET(SetDistancePerPulse, double)
  VMX_ENCODER_MANAGER_SET(SetMinRate, double)
  VMX_ENCODER_MANAGER_SET(SetReverseDirection, bool)
  VMX_ENCODER_MANAGER_SET(SetSamplesToAverage, int32_t)

#undef VMX_ENCODER_MANAGER_SET

  EncoderResult Reset(HAL_EncoderHandle handle) {
    auto port = GetPort(handle);
    return port ? port->Reset() : EncoderResult::kInvalidHandle;
  }

 private:
  std::shared_ptr<EncoderPort> GetPort(HAL_EncoderHandle handle) {
    return m_handles.Get(handle);
  }

  EncoderBackendFactory m_factory;
  EncoderSourceClaimer m_claimer;
  EncoderSourceReleaser m_releaser;
  EncoderClock m_clock;
  std::mutex m_allocationMutex;
  EncoderHandleResource m_handles;
};

}  // namespace hal::vmx
