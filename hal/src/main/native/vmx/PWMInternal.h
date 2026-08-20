// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>

#include "DigitalChannelRegistry.h"
#include "VMXChannelCapabilities.h"
#include "VMXConstants.h"
#include "hal/Errors.h"
#include "hal/Types.h"
#include "hal/handles/DigitalHandleResource.h"

namespace hal::vmx {

constexpr int32_t kPWMDisabledPulse = 0;
constexpr int32_t kPWMMaximumRawPulse = 4095;

struct PWMConfig {
  int32_t maxPwm = 2000;
  int32_t deadbandMaxPwm = 1501;
  int32_t centerPwm = 1500;
  int32_t deadbandMinPwm = 1499;
  int32_t minPwm = 1000;
};

enum class PWMResult {
  kOk,
  kOutOfRange,
  kAlreadyAllocated,
  kInvalidHandle,
  kHardwareFailure,
  kScaleError,
  kUnsupportedCapability,
};

class PWMBackend {
 public:
  virtual ~PWMBackend() = default;
  virtual bool SetPulseTimeMicroseconds(int32_t requested,
                                        int32_t& applied) noexcept = 0;
  virtual bool GetPulseTimeMicroseconds(int32_t& pulse) noexcept = 0;
  virtual bool Disable() noexcept = 0;
};

using PWMBackendFactory =
    std::function<std::unique_ptr<PWMBackend>(int32_t channel)>;

class PWMPort final {
 public:
  bool Initialize(int32_t channel, PWMBackendFactory factory,
                  int32_t physicalChannel = -1) noexcept {
    std::scoped_lock lock{m_mutex};
    m_channel = channel;
    m_physicalChannel = physicalChannel >= 0 ? physicalChannel : channel;
    try {
      m_backend = factory ? factory(m_physicalChannel) : nullptr;
    } catch (...) {
      m_backend.reset();
    }
    if (!m_backend || !m_backend->Disable()) {
      m_backend.reset();
      return false;
    }
    m_currentPulseMicroseconds = kPWMDisabledPulse;
    m_disabled = true;
    return true;
  }

  PWMResult SetConfig(PWMConfig config) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return PWMResult::kHardwareFailure;
    }
    m_config = config;
    return PWMResult::kOk;
  }

  std::pair<PWMResult, PWMConfig> GetConfig() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_backend ? std::pair{PWMResult::kOk, m_config}
                     : std::pair{PWMResult::kHardwareFailure, PWMConfig{}};
  }

  PWMResult SetEliminateDeadband(bool eliminate) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return PWMResult::kHardwareFailure;
    }
    m_eliminateDeadband = eliminate;
    return PWMResult::kOk;
  }

  std::pair<PWMResult, bool> GetEliminateDeadband() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_backend ? std::pair{PWMResult::kOk, m_eliminateDeadband}
                     : std::pair{PWMResult::kHardwareFailure, false};
  }

  PWMResult SetPulseTimeMicroseconds(int32_t value) noexcept {
    std::scoped_lock lock{m_mutex};
    return SetPulseTimeMicrosecondsLocked(value);
  }

  PWMResult SetSpeed(double speed) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return PWMResult::kHardwareFailure;
    }
    if (std::isfinite(speed)) {
      speed = std::clamp(speed, -1.0, 1.0);
    } else {
      speed = 0.0;
    }

    int32_t rawValue;
    if (speed == 0.0) {
      rawValue = m_config.centerPwm;
    } else if (speed > 0.0) {
      int32_t scale = PositiveScaleFactor();
      if (scale < 0) {
        return PWMResult::kScaleError;
      }
      rawValue = std::lround(speed * static_cast<double>(scale) +
                             static_cast<double>(MinPositivePwm()));
    } else {
      int32_t scale = NegativeScaleFactor();
      if (scale < 0) {
        return PWMResult::kScaleError;
      }
      rawValue = std::lround(speed * static_cast<double>(scale) +
                             static_cast<double>(MaxNegativePwm()));
    }

    if (rawValue < m_config.minPwm || rawValue > m_config.maxPwm ||
        rawValue == kPWMDisabledPulse) {
      return PWMResult::kScaleError;
    }
    return SetPulseTimeMicrosecondsLocked(rawValue);
  }

  PWMResult SetPosition(double position) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return PWMResult::kHardwareFailure;
    }
    if (!std::isfinite(position)) {
      position = 0.0;
    }
    position = std::clamp(position, 0.0, 1.0);
    int32_t scale = FullRangeScaleFactor();
    if (scale < 0) {
      return PWMResult::kScaleError;
    }
    int32_t rawValue = static_cast<int32_t>(
        position * static_cast<double>(scale) + m_config.minPwm);
    if (rawValue == kPWMDisabledPulse) {
      return PWMResult::kScaleError;
    }
    return SetPulseTimeMicrosecondsLocked(rawValue);
  }

  PWMResult Disable() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return PWMResult::kHardwareFailure;
    }
    if (!m_backend->Disable()) {
      return PWMResult::kHardwareFailure;
    }
    m_currentPulseMicroseconds = kPWMDisabledPulse;
    m_disabled = true;
    return PWMResult::kOk;
  }

  std::pair<PWMResult, int32_t> GetPulseTimeMicroseconds() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!ReadPulseTimeMicrosecondsLocked()) {
      return {PWMResult::kHardwareFailure, 0};
    }
    return {PWMResult::kOk, m_currentPulseMicroseconds};
  }

  std::pair<PWMResult, double> GetSpeed() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!ReadPulseTimeMicrosecondsLocked()) {
      return {PWMResult::kHardwareFailure, 0.0};
    }
    int32_t value = m_currentPulseMicroseconds;
    if (m_disabled) {
      return {PWMResult::kOk, 0.0};
    }
    if (value > m_config.maxPwm) {
      return {PWMResult::kOk, 1.0};
    }
    if (value < m_config.minPwm) {
      return {PWMResult::kOk, -1.0};
    }
    if (value > MinPositivePwm()) {
      int32_t scale = PositiveScaleFactor();
      return scale > 0
                 ? std::pair{PWMResult::kOk,
                             static_cast<double>(value - MinPositivePwm()) /
                                 static_cast<double>(scale)}
                 : std::pair{PWMResult::kScaleError, 0.0};
    }
    if (value < MaxNegativePwm()) {
      int32_t scale = NegativeScaleFactor();
      return scale > 0
                 ? std::pair{PWMResult::kOk,
                             static_cast<double>(value - MaxNegativePwm()) /
                                 static_cast<double>(scale)}
                 : std::pair{PWMResult::kScaleError, 0.0};
    }
    return {PWMResult::kOk, 0.0};
  }

  std::pair<PWMResult, double> GetPosition() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!ReadPulseTimeMicrosecondsLocked()) {
      return {PWMResult::kHardwareFailure, 0.0};
    }
    int32_t value = m_currentPulseMicroseconds;
    if (value < m_config.minPwm || m_disabled) {
      return {PWMResult::kOk, 0.0};
    }
    if (value > m_config.maxPwm) {
      return {PWMResult::kOk, 1.0};
    }
    int32_t scale = FullRangeScaleFactor();
    return scale > 0 ? std::pair{PWMResult::kOk,
                                 static_cast<double>(value - m_config.minPwm) /
                                     static_cast<double>(scale)}
                     : std::pair{PWMResult::kScaleError, 0.0};
  }

  std::pair<PWMResult, bool> IsDisabled() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_backend ? std::pair{PWMResult::kOk, m_disabled}
                     : std::pair{PWMResult::kHardwareFailure, false};
  }

  int32_t GetChannel() const noexcept { return m_channel; }
  int32_t GetPhysicalChannel() const noexcept { return m_physicalChannel; }

  void Close() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_backend) {
      m_backend->Disable();
      m_backend.reset();
    }
  }

 private:
  bool ReadPulseTimeMicrosecondsLocked() noexcept {
    if (!m_backend) {
      return false;
    }
    int32_t pulse = 0;
    if (!m_backend->GetPulseTimeMicroseconds(pulse)) {
      return false;
    }
    m_currentPulseMicroseconds = pulse;
    m_disabled = pulse == kPWMDisabledPulse;
    return true;
  }

  PWMResult SetPulseTimeMicrosecondsLocked(int32_t value) noexcept {
    if (!m_backend) {
      return PWMResult::kHardwareFailure;
    }
    if (value < kPWMDisabledPulse || value > kPWMMaximumRawPulse) {
      return PWMResult::kOutOfRange;
    }
    if (value == kPWMDisabledPulse) {
      if (!m_backend->Disable()) {
        return PWMResult::kHardwareFailure;
      }
      m_currentPulseMicroseconds = kPWMDisabledPulse;
      m_disabled = true;
      return PWMResult::kOk;
    }

    int32_t applied = 0;
    if (!m_backend->SetPulseTimeMicroseconds(value, applied)) {
      return PWMResult::kHardwareFailure;
    }
    m_currentPulseMicroseconds = applied;
    m_disabled = false;
    return PWMResult::kOk;
  }

  int32_t MinPositivePwm() const noexcept {
    return m_eliminateDeadband ? m_config.deadbandMaxPwm
                               : m_config.centerPwm + 1;
  }
  int32_t MaxNegativePwm() const noexcept {
    return m_eliminateDeadband ? m_config.deadbandMinPwm
                               : m_config.centerPwm - 1;
  }
  int32_t PositiveScaleFactor() const noexcept {
    return m_config.maxPwm - MinPositivePwm();
  }
  int32_t NegativeScaleFactor() const noexcept {
    return MaxNegativePwm() - m_config.minPwm;
  }
  int32_t FullRangeScaleFactor() const noexcept {
    return m_config.maxPwm - m_config.minPwm;
  }

  mutable std::mutex m_mutex;
  int32_t m_channel = -1;
  int32_t m_physicalChannel = -1;
  PWMConfig m_config;
  bool m_eliminateDeadband = false;
  int32_t m_currentPulseMicroseconds = kPWMDisabledPulse;
  bool m_disabled = true;
  std::unique_ptr<PWMBackend> m_backend;
};

struct PWMAllocationResult {
  HAL_DigitalHandle handle = HAL_kInvalidHandle;
  PWMResult result = PWMResult::kHardwareFailure;
  std::string previousAllocation;
};

class PWMHandleResource final
    : public hal::DigitalHandleResource<HAL_DigitalHandle, PWMPort,
                                        kNumPWMChannels> {
 public:
  PWMHandleResource() { m_version = 0; }
};

class PWMManager final {
 public:
  explicit PWMManager(
      PWMBackendFactory factory,
      DigitalChannelRegistry& registry = GetDigitalChannelRegistry(),
      const VMXCapabilityProvider* capabilities = nullptr)
      : m_factory{std::move(factory)},
        m_registry{registry},
        m_capabilities{capabilities} {}

  PWMAllocationResult Allocate(int32_t channel, std::string_view location) {
    std::scoped_lock allocationLock{m_allocationMutex};
    if (!IsPWMChannelValid(channel)) {
      return {HAL_kInvalidHandle, PWMResult::kOutOfRange, {}};
    }
    const auto physicalChannel = ToVMXDigitalChannel(channel);
    if (m_capabilities && !m_capabilities->SupportsPhysical(
                              physicalChannel,
                              VMXCapability::kPWMGenerator)) {
      return {HAL_kInvalidHandle, PWMResult::kUnsupportedCapability, {}};
    }
    auto reservation = m_registry.Reserve(physicalChannel,
                                          DigitalChannelOwner::kPWM, location);
    if (!reservation.reserved) {
      return {HAL_kInvalidHandle, PWMResult::kAlreadyAllocated,
              std::move(reservation.previousAllocation)};
    }
    auto timerReservation = m_registry.ReserveFlexTimerGroup(
        physicalChannel, DigitalChannelOwner::kPWM, location);
    if (!timerReservation.reserved && physicalChannel < 12) {
      m_registry.Release(physicalChannel, DigitalChannelOwner::kPWM);
      return {HAL_kInvalidHandle, PWMResult::kAlreadyAllocated,
              std::move(timerReservation.previousAllocation)};
    }

    HAL_DigitalHandle handle = HAL_kInvalidHandle;
    int32_t status = 0;
    auto port =
        m_handles.Allocate(channel, HAL_HandleEnum::PWM, &handle, &status);
    if (status != 0) {
      m_registry.Release(physicalChannel, DigitalChannelOwner::kPWM);
      m_registry.ReleaseFlexTimerGroup(physicalChannel,
                                       DigitalChannelOwner::kPWM);
      return {HAL_kInvalidHandle, PWMResult::kHardwareFailure, {}};
    }
    if (!port->Initialize(channel, m_factory, physicalChannel)) {
      m_handles.Free(handle, HAL_HandleEnum::PWM);
      m_registry.Release(physicalChannel, DigitalChannelOwner::kPWM);
      m_registry.ReleaseFlexTimerGroup(physicalChannel,
                                       DigitalChannelOwner::kPWM);
      return {HAL_kInvalidHandle, PWMResult::kHardwareFailure, {}};
    }
    return {handle, PWMResult::kOk, {}};
  }

  void Free(HAL_DigitalHandle handle) noexcept {
    std::scoped_lock allocationLock{m_allocationMutex};
    auto port = Get(handle);
    if (!port) {
      return;
    }
    port->Close();
    m_handles.Free(handle, HAL_HandleEnum::PWM);
    m_registry.Release(port->GetPhysicalChannel(), DigitalChannelOwner::kPWM);
    m_registry.ReleaseFlexTimerGroup(port->GetPhysicalChannel(),
                                     DigitalChannelOwner::kPWM);
  }

  PWMResult SetConfig(HAL_DigitalHandle handle, PWMConfig config) noexcept {
    auto port = Get(handle);
    return port ? port->SetConfig(config) : PWMResult::kInvalidHandle;
  }
  std::pair<PWMResult, PWMConfig> GetConfig(HAL_DigitalHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetConfig()
                : std::pair{PWMResult::kInvalidHandle, PWMConfig{}};
  }
  PWMResult SetEliminateDeadband(HAL_DigitalHandle handle,
                                 bool eliminate) noexcept {
    auto port = Get(handle);
    return port ? port->SetEliminateDeadband(eliminate)
                : PWMResult::kInvalidHandle;
  }
  std::pair<PWMResult, bool> GetEliminateDeadband(
      HAL_DigitalHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetEliminateDeadband()
                : std::pair{PWMResult::kInvalidHandle, false};
  }
  PWMResult SetPulseTimeMicroseconds(HAL_DigitalHandle handle,
                                     int32_t value) noexcept {
    auto port = Get(handle);
    return port ? port->SetPulseTimeMicroseconds(value)
                : PWMResult::kInvalidHandle;
  }
  PWMResult SetSpeed(HAL_DigitalHandle handle, double speed) noexcept {
    auto port = Get(handle);
    return port ? port->SetSpeed(speed) : PWMResult::kInvalidHandle;
  }
  PWMResult SetPosition(HAL_DigitalHandle handle, double position) noexcept {
    auto port = Get(handle);
    return port ? port->SetPosition(position) : PWMResult::kInvalidHandle;
  }
  PWMResult Disable(HAL_DigitalHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->Disable() : PWMResult::kInvalidHandle;
  }
  std::pair<PWMResult, int32_t> GetPulseTimeMicroseconds(
      HAL_DigitalHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetPulseTimeMicroseconds()
                : std::pair{PWMResult::kInvalidHandle, 0};
  }
  std::pair<PWMResult, double> GetSpeed(HAL_DigitalHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetSpeed() : std::pair{PWMResult::kInvalidHandle, 0.0};
  }
  std::pair<PWMResult, double> GetPosition(HAL_DigitalHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetPosition()
                : std::pair{PWMResult::kInvalidHandle, 0.0};
  }
  std::pair<PWMResult, bool> IsDisabled(HAL_DigitalHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->IsDisabled()
                : std::pair{PWMResult::kInvalidHandle, false};
  }

 private:
  std::shared_ptr<PWMPort> Get(HAL_DigitalHandle handle) noexcept {
    return m_handles.Get(handle, HAL_HandleEnum::PWM);
  }

  PWMBackendFactory m_factory;
  DigitalChannelRegistry& m_registry;
  const VMXCapabilityProvider* m_capabilities;
  std::mutex m_allocationMutex;
  PWMHandleResource m_handles;
};

}  // namespace hal::vmx
