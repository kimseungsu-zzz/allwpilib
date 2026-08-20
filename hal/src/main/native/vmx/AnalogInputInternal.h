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

#include "VMXConstants.h"
#include "hal/Errors.h"
#include "hal/Types.h"
#include "hal/handles/IndexedHandleResource.h"

namespace hal::vmx {

constexpr int32_t kDefaultAnalogAverageBits = 7;
constexpr int32_t kDefaultAnalogOversampleBits = 0;
constexpr int32_t kVMXADCResolutionBits = 12;
constexpr int32_t kVMXADCCounts = 1 << kVMXADCResolutionBits;
constexpr double kVMXAnalogSampleRate = 46500.0;

enum class AnalogInputResult {
  kOk,
  kOutOfRange,
  kAlreadyAllocated,
  kInvalidHandle,
  kHardwareFailure,
  kRollbackFailure,
  kInvalidAccumulatorChannel,
  kAccumulatorNotInitialized,
};

struct AnalogInputConfig {
  int32_t averageBits = kDefaultAnalogAverageBits;
  int32_t oversampleBits = kDefaultAnalogOversampleBits;
  bool accumulatorEnabled = false;
  int32_t center = 0;
  int32_t deadband = 0;
};

struct AnalogAccumulatorOutput {
  int64_t value = 0;
  int64_t count = 0;
};

class AnalogInputBackend {
 public:
  virtual ~AnalogInputBackend() = default;
  virtual bool GetValue(uint32_t& value) noexcept = 0;
  virtual bool GetAverageValue(uint32_t& value) noexcept = 0;
  virtual bool GetVoltage(double& voltage) noexcept = 0;
  virtual bool GetAverageVoltage(double& voltage) noexcept = 0;
  virtual bool ResetAccumulator() noexcept = 0;
  virtual bool GetAccumulatorOutput(int64_t& value,
                                    uint32_t& count) noexcept = 0;
  virtual double GetFullScaleVoltage() const noexcept = 0;
};

using AnalogInputBackendFactory =
    std::function<std::unique_ptr<AnalogInputBackend>(
        int32_t physicalChannel, const AnalogInputConfig& config)>;

class AnalogInputPort final {
 public:
  bool Initialize(int32_t logicalChannel, int32_t physicalChannel,
                  std::string_view allocationLocation,
                  AnalogInputBackendFactory factory) noexcept {
    std::scoped_lock lock{m_mutex};
    m_logicalChannel = logicalChannel;
    m_physicalChannel = physicalChannel;
    m_previousAllocation = allocationLocation;
    m_factory = std::move(factory);
    m_backend = CreateBackend(m_config);
    m_faulted = !m_backend;
    return !m_faulted;
  }

  std::pair<AnalogInputResult, double> GetVoltage() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted || !m_backend) {
      return {AnalogInputResult::kHardwareFailure, 0.0};
    }
    double voltage = 0.0;
    return m_backend->GetVoltage(voltage)
               ? std::pair{AnalogInputResult::kOk, voltage}
               : std::pair{AnalogInputResult::kHardwareFailure, 0.0};
  }

  std::pair<AnalogInputResult, double> GetAverageVoltage() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted || !m_backend) {
      return {AnalogInputResult::kHardwareFailure, 0.0};
    }
    double voltage = 0.0;
    return m_backend->GetAverageVoltage(voltage)
               ? std::pair{AnalogInputResult::kOk, voltage}
               : std::pair{AnalogInputResult::kHardwareFailure, 0.0};
  }

  std::pair<AnalogInputResult, int32_t> GetValue() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted || !m_backend) {
      return {AnalogInputResult::kHardwareFailure, 0};
    }
    uint32_t value = 0;
    return m_backend->GetValue(value)
               ? std::pair{AnalogInputResult::kOk, static_cast<int32_t>(value)}
               : std::pair{AnalogInputResult::kHardwareFailure, 0};
  }

  std::pair<AnalogInputResult, int32_t> GetAverageValue() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted || !m_backend) {
      return {AnalogInputResult::kHardwareFailure, 0};
    }
    uint32_t value = 0;
    return m_backend->GetAverageValue(value)
               ? std::pair{AnalogInputResult::kOk, static_cast<int32_t>(value)}
               : std::pair{AnalogInputResult::kHardwareFailure, 0};
  }

  AnalogInputResult SetAverageBits(int32_t bits) noexcept {
    std::scoped_lock lock{m_mutex};
    auto config = m_config;
    config.averageBits = bits;
    return Reconfigure(config);
  }

  AnalogInputResult SetOversampleBits(int32_t bits) noexcept {
    std::scoped_lock lock{m_mutex};
    auto config = m_config;
    config.oversampleBits = bits;
    return Reconfigure(config);
  }

  std::pair<AnalogInputResult, int32_t> GetAverageBits() const noexcept {
    std::scoped_lock lock{m_mutex};
    return IsUsableLocked()
               ? std::pair{AnalogInputResult::kOk, m_config.averageBits}
                      : std::pair{AnalogInputResult::kHardwareFailure, 0};
  }

  std::pair<AnalogInputResult, int32_t> GetOversampleBits() const noexcept {
    std::scoped_lock lock{m_mutex};
    return IsUsableLocked()
               ? std::pair{AnalogInputResult::kOk, m_config.oversampleBits}
                      : std::pair{AnalogInputResult::kHardwareFailure, 0};
  }

  std::pair<AnalogInputResult, int32_t> GetLSBWeight() const noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsUsableLocked()) {
      return {AnalogInputResult::kHardwareFailure, 0};
    }
    return {AnalogInputResult::kOk,
            static_cast<int32_t>(std::llround(m_backend->GetFullScaleVoltage() /
                                              kVMXADCCounts * 1.0e9))};
  }

  std::pair<AnalogInputResult, double> ValueToVolts(
      int32_t rawValue) const noexcept {
    std::scoped_lock lock{m_mutex};
    return IsUsableLocked()
               ? std::pair{AnalogInputResult::kOk,
                           static_cast<double>(rawValue) *
                               m_backend->GetFullScaleVoltage() / kVMXADCCounts}
               : std::pair{AnalogInputResult::kHardwareFailure, 0.0};
  }

  struct VoltsToValueResult {
    AnalogInputResult result = AnalogInputResult::kHardwareFailure;
    int32_t value = 0;
    bool clamped = false;
  };

  VoltsToValueResult VoltsToValue(double voltage) const noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsUsableLocked()) {
      return {};
    }
    double fullScale = m_backend->GetFullScaleVoltage();
    bool clamped =
        !std::isfinite(voltage) || voltage < 0.0 || voltage > fullScale;
    if (!std::isfinite(voltage)) {
      voltage = 0.0;
    }
    voltage = std::clamp(voltage, 0.0, fullScale);
    return {AnalogInputResult::kOk,
            static_cast<int32_t>(voltage * kVMXADCCounts / fullScale), clamped};
  }

  AnalogInputResult InitAccumulator() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsAnalogAccumulatorChannelValid(m_logicalChannel)) {
      return AnalogInputResult::kInvalidAccumulatorChannel;
    }
    if (!IsUsableLocked()) {
      return AnalogInputResult::kHardwareFailure;
    }
    if (m_config.accumulatorEnabled) {
      return AnalogInputResult::kOk;
    }
    auto config = m_config;
    config.accumulatorEnabled = true;
    config.center = 0;
    config.deadband = 0;
    auto result = Reconfigure(config);
    if (result == AnalogInputResult::kOk) {
      m_accumulatorValueOffset = 0;
      m_accumulatorCountOffset = 0;
      m_accumulatorCountWrapOffset = 0;
      m_lastHardwareCount = 0;
    }
    return result;
  }

  AnalogInputResult ResetAccumulator() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsUsableLocked()) {
      return AnalogInputResult::kHardwareFailure;
    }
    if (!m_config.accumulatorEnabled) {
      return AnalogInputResult::kAccumulatorNotInitialized;
    }
    if (!m_backend->ResetAccumulator()) {
      return AnalogInputResult::kHardwareFailure;
    }
    m_accumulatorValueOffset = 0;
    m_accumulatorCountOffset = 0;
    m_accumulatorCountWrapOffset = 0;
    m_lastHardwareCount = 0;
    return AnalogInputResult::kOk;
  }

  AnalogInputResult SetAccumulatorCenter(int32_t center) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_config.accumulatorEnabled) {
      return AnalogInputResult::kAccumulatorNotInitialized;
    }
    auto config = m_config;
    config.center = center;
    return Reconfigure(config);
  }

  AnalogInputResult SetAccumulatorDeadband(int32_t deadband) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_config.accumulatorEnabled) {
      return AnalogInputResult::kAccumulatorNotInitialized;
    }
    auto config = m_config;
    config.deadband = deadband;
    return Reconfigure(config);
  }

  std::pair<AnalogInputResult, AnalogAccumulatorOutput> GetAccumulatorOutput()
      noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsUsableLocked()) {
      return {AnalogInputResult::kHardwareFailure, {}};
    }
    if (!m_config.accumulatorEnabled) {
      return {AnalogInputResult::kAccumulatorNotInitialized, {}};
    }
    AnalogAccumulatorOutput output;
    auto result = ReadAccumulatorLocked(output);
    return {result, result == AnalogInputResult::kOk ? output
                                                     : AnalogAccumulatorOutput{}};
  }

  int32_t GetLogicalChannel() const noexcept { return m_logicalChannel; }
  int32_t GetPhysicalChannel() const noexcept { return m_physicalChannel; }

  std::string GetPreviousAllocation() const {
    std::scoped_lock lock{m_mutex};
    return m_previousAllocation;
  }

  // AnalogTrigger keeps this shared reference internally so the trigger can
  // never dereference a reclaimed AnalogInputPort. HAL_FreeAnalogInputPort()
  // still closes the port immediately; a trigger that outlives its input then
  // observes a hardware failure instead of a use-after-free.
  bool IsUsable() const noexcept {
    std::scoped_lock lock{m_mutex};
    return IsUsableLocked();
  }

  void Close() noexcept {
    std::scoped_lock lock{m_mutex};
    m_faulted = true;
    m_backend.reset();
  }

 private:
  bool IsUsableLocked() const noexcept {
    return !m_faulted && m_backend != nullptr;
  }

  std::unique_ptr<AnalogInputBackend> CreateBackend(
      const AnalogInputConfig& config) noexcept {
    try {
      return m_factory ? m_factory(m_physicalChannel, config) : nullptr;
    } catch (...) {
      return nullptr;
    }
  }

  AnalogInputResult ReadAccumulatorLocked(
      AnalogAccumulatorOutput& output) noexcept {
    int64_t hardwareValue = 0;
    uint32_t hardwareCount = 0;
    if (!m_backend->GetAccumulatorOutput(hardwareValue, hardwareCount)) {
      return AnalogInputResult::kHardwareFailure;
    }
    if (hardwareCount < m_lastHardwareCount) {
      m_accumulatorCountWrapOffset += uint64_t{1} << 32;
    }
    m_lastHardwareCount = hardwareCount;
    output.value = m_accumulatorValueOffset + hardwareValue;
    output.count = m_accumulatorCountOffset + m_accumulatorCountWrapOffset +
                   static_cast<int64_t>(hardwareCount);
    return AnalogInputResult::kOk;
  }

  AnalogInputResult Reconfigure(const AnalogInputConfig& config) noexcept {
    if (!IsUsableLocked()) {
      return AnalogInputResult::kHardwareFailure;
    }
    if (config.averageBits < 0 || config.averageBits > 255 ||
        config.oversampleBits < 0 || config.oversampleBits > 255 ||
        config.center < INT16_MIN || config.center > INT16_MAX ||
        config.deadband < 0 || config.deadband > INT16_MAX) {
      return AnalogInputResult::kOutOfRange;
    }
    if (config.averageBits == m_config.averageBits &&
        config.oversampleBits == m_config.oversampleBits &&
        config.accumulatorEnabled == m_config.accumulatorEnabled &&
        config.center == m_config.center &&
        config.deadband == m_config.deadband) {
      return AnalogInputResult::kOk;
    }

    AnalogAccumulatorOutput preserved;
    if (m_config.accumulatorEnabled) {
      auto snapshotResult = ReadAccumulatorLocked(preserved);
      if (snapshotResult != AnalogInputResult::kOk) {
        return snapshotResult;
      }
    }

    auto previousConfig = m_config;
    m_backend.reset();

    auto replacement = CreateBackend(config);
    if (replacement) {
      m_backend = std::move(replacement);
      m_config = config;
      if (previousConfig.accumulatorEnabled) {
        m_accumulatorValueOffset = preserved.value;
        m_accumulatorCountOffset = preserved.count;
      }
      m_accumulatorCountWrapOffset = 0;
      m_lastHardwareCount = 0;
      return AnalogInputResult::kOk;
    }

    auto rollback = CreateBackend(previousConfig);
    if (rollback) {
      m_backend = std::move(rollback);
      if (previousConfig.accumulatorEnabled) {
        m_accumulatorValueOffset = preserved.value;
        m_accumulatorCountOffset = preserved.count;
      }
      m_accumulatorCountWrapOffset = 0;
      m_lastHardwareCount = 0;
      return AnalogInputResult::kHardwareFailure;
    }

    m_faulted = true;
    return AnalogInputResult::kRollbackFailure;
  }

  mutable std::mutex m_mutex;
  int32_t m_logicalChannel = -1;
  int32_t m_physicalChannel = -1;
  AnalogInputConfig m_config;
  int64_t m_accumulatorValueOffset = 0;
  int64_t m_accumulatorCountOffset = 0;
  int64_t m_accumulatorCountWrapOffset = 0;
  uint32_t m_lastHardwareCount = 0;
  bool m_faulted = true;
  std::string m_previousAllocation;
  AnalogInputBackendFactory m_factory;
  std::unique_ptr<AnalogInputBackend> m_backend;
};

struct AnalogInputAllocationResult {
  HAL_AnalogInputHandle handle = HAL_kInvalidHandle;
  AnalogInputResult result = AnalogInputResult::kHardwareFailure;
  std::string previousAllocation;
};

class AnalogInputHandleResource final
    : public hal::IndexedHandleResource<HAL_AnalogInputHandle, AnalogInputPort,
                                        kNumAnalogInputs,
                                        HAL_HandleEnum::AnalogInput> {
 public:
  AnalogInputHandleResource() { m_version = 0; }
};

class AnalogInputManager final {
 public:
  explicit AnalogInputManager(AnalogInputBackendFactory factory)
      : m_factory{std::move(factory)} {}

  AnalogInputAllocationResult Allocate(int32_t logicalChannel,
                                       std::string_view location) {
    std::scoped_lock allocationLock{m_allocationMutex};
    if (!IsAnalogInputChannelValid(logicalChannel)) {
      return {HAL_kInvalidHandle, AnalogInputResult::kOutOfRange, {}};
    }

    HAL_AnalogInputHandle handle = HAL_kInvalidHandle;
    int32_t status = 0;
    auto port = m_handles.Allocate(logicalChannel, &handle, &status);
    if (status != 0) {
      return {HAL_kInvalidHandle,
              status == RESOURCE_IS_ALLOCATED
                  ? AnalogInputResult::kAlreadyAllocated
                  : AnalogInputResult::kOutOfRange,
              port ? port->GetPreviousAllocation() : std::string{}};
    }

    if (!port->Initialize(logicalChannel, ToVMXAnalogChannel(logicalChannel),
                          location, m_factory)) {
      m_handles.Free(handle);
      return {HAL_kInvalidHandle, AnalogInputResult::kHardwareFailure, {}};
    }
    return {handle, AnalogInputResult::kOk, {}};
  }

  void Free(HAL_AnalogInputHandle handle) noexcept {
    std::scoped_lock allocationLock{m_allocationMutex};
    auto port = Get(handle);
    if (!port) {
      return;
    }
    port->Close();
    m_handles.Free(handle);
  }

  std::pair<AnalogInputResult, double> GetVoltage(
      HAL_AnalogInputHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetVoltage()
                : std::pair{AnalogInputResult::kInvalidHandle, 0.0};
  }
  std::pair<AnalogInputResult, double> GetAverageVoltage(
      HAL_AnalogInputHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetAverageVoltage()
                : std::pair{AnalogInputResult::kInvalidHandle, 0.0};
  }
  std::pair<AnalogInputResult, int32_t> GetValue(
      HAL_AnalogInputHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetValue()
                : std::pair{AnalogInputResult::kInvalidHandle, 0};
  }
  std::pair<AnalogInputResult, int32_t> GetAverageValue(
      HAL_AnalogInputHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetAverageValue()
                : std::pair{AnalogInputResult::kInvalidHandle, 0};
  }
  AnalogInputResult SetAverageBits(HAL_AnalogInputHandle handle,
                                   int32_t bits) noexcept {
    auto port = Get(handle);
    return port ? port->SetAverageBits(bits)
                : AnalogInputResult::kInvalidHandle;
  }
  AnalogInputResult SetOversampleBits(HAL_AnalogInputHandle handle,
                                      int32_t bits) noexcept {
    auto port = Get(handle);
    return port ? port->SetOversampleBits(bits)
                : AnalogInputResult::kInvalidHandle;
  }
  std::pair<AnalogInputResult, int32_t> GetAverageBits(
      HAL_AnalogInputHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetAverageBits()
                : std::pair{AnalogInputResult::kInvalidHandle, 0};
  }
  std::pair<AnalogInputResult, int32_t> GetOversampleBits(
      HAL_AnalogInputHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetOversampleBits()
                : std::pair{AnalogInputResult::kInvalidHandle, 0};
  }
  std::pair<AnalogInputResult, int32_t> GetLSBWeight(
      HAL_AnalogInputHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetLSBWeight()
                : std::pair{AnalogInputResult::kInvalidHandle, 0};
  }
  std::pair<AnalogInputResult, double> ValueToVolts(
      HAL_AnalogInputHandle handle, int32_t rawValue) noexcept {
    auto port = Get(handle);
    return port ? port->ValueToVolts(rawValue)
                : std::pair{AnalogInputResult::kInvalidHandle, 0.0};
  }
  AnalogInputPort::VoltsToValueResult VoltsToValue(HAL_AnalogInputHandle handle,
                                                   double voltage) noexcept {
    auto port = Get(handle);
    return port ? port->VoltsToValue(voltage)
                : AnalogInputPort::VoltsToValueResult{
                      AnalogInputResult::kInvalidHandle, 0, false};
  }
  std::shared_ptr<AnalogInputPort> AcquirePort(
      HAL_AnalogInputHandle handle) noexcept {
    return Get(handle);
  }
  std::pair<AnalogInputResult, bool> IsAccumulatorChannel(
      HAL_AnalogInputHandle handle) noexcept {
    auto port = Get(handle);
    return port ? std::pair{AnalogInputResult::kOk,
                            IsAnalogAccumulatorChannelValid(
                                port->GetLogicalChannel())}
                : std::pair{AnalogInputResult::kInvalidHandle, false};
  }
  AnalogInputResult InitAccumulator(HAL_AnalogInputHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->InitAccumulator()
                : AnalogInputResult::kInvalidHandle;
  }
  AnalogInputResult ResetAccumulator(HAL_AnalogInputHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->ResetAccumulator()
                : AnalogInputResult::kInvalidHandle;
  }
  AnalogInputResult SetAccumulatorCenter(HAL_AnalogInputHandle handle,
                                         int32_t center) noexcept {
    auto port = Get(handle);
    return port ? port->SetAccumulatorCenter(center)
                : AnalogInputResult::kInvalidHandle;
  }
  AnalogInputResult SetAccumulatorDeadband(HAL_AnalogInputHandle handle,
                                           int32_t deadband) noexcept {
    auto port = Get(handle);
    return port ? port->SetAccumulatorDeadband(deadband)
                : AnalogInputResult::kInvalidHandle;
  }
  std::pair<AnalogInputResult, AnalogAccumulatorOutput> GetAccumulatorOutput(
      HAL_AnalogInputHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetAccumulatorOutput()
                : std::pair{AnalogInputResult::kInvalidHandle,
                            AnalogAccumulatorOutput{}};
  }

 private:
  std::shared_ptr<AnalogInputPort> Get(HAL_AnalogInputHandle handle) noexcept {
    return m_handles.Get(handle);
  }

  AnalogInputBackendFactory m_factory;
  std::mutex m_allocationMutex;
  AnalogInputHandleResource m_handles;
};

AnalogInputManager& GetAnalogInputManager();

}  // namespace hal::vmx
