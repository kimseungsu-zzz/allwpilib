// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include "AnalogInputInternal.h"
#include "VMXConstants.h"
#include "hal/AnalogTrigger.h"
#include "hal/Types.h"
#include "hal/handles/LimitedClassedHandleResource.h"

namespace hal::vmx {

enum class AnalogTriggerResult {
  kOk,
  kInvalidHandle,
  kNoResources,
  kLimitOrder,
  kOutOfRange,
  kUnsupported,
  kPulseOutput,
  kHardwareFailure,
};

enum class AnalogTriggerLimitMode { kRaw, kVoltage };

class AnalogTriggerPort final {
 public:
  explicit AnalogTriggerPort(std::shared_ptr<AnalogInputPort> input)
      : m_input{std::move(input)},
        m_inputChannel{m_input ? m_input->GetLogicalChannel() : -1} {}

  ~AnalogTriggerPort() { Close(); }

  AnalogTriggerResult SetLimitsRaw(int32_t lower, int32_t upper) noexcept {
    std::scoped_lock lock{m_mutex};
    auto result = CheckUsableLocked();
    if (result != AnalogTriggerResult::kOk) {
      return result;
    }
    if (lower > upper) {
      return AnalogTriggerResult::kLimitOrder;
    }
    if (lower < 0 || upper >= kVMXADCCounts) {
      return AnalogTriggerResult::kOutOfRange;
    }
    m_limitMode = AnalogTriggerLimitMode::kRaw;
    m_lowerRaw = lower;
    m_upperRaw = upper;
    return AnalogTriggerResult::kOk;
  }

  AnalogTriggerResult SetLimitsVoltage(double lower, double upper) noexcept {
    std::scoped_lock lock{m_mutex};
    auto result = CheckUsableLocked();
    if (result != AnalogTriggerResult::kOk) {
      return result;
    }
    if (lower > upper) {
      return AnalogTriggerResult::kLimitOrder;
    }
    if (!std::isfinite(lower) || !std::isfinite(upper)) {
      return AnalogTriggerResult::kOutOfRange;
    }

    // ValueToVolts(4096) exposes the same runtime-discovered full-scale value
    // used by AnalogInput's conversion helpers without allocating a resource.
    auto [conversionResult, fullScale] =
        m_input->ValueToVolts(kVMXADCCounts);
    if (conversionResult != AnalogInputResult::kOk) {
      return AnalogTriggerResult::kHardwareFailure;
    }
    if (lower < 0.0 || upper > fullScale) {
      return AnalogTriggerResult::kOutOfRange;
    }
    m_limitMode = AnalogTriggerLimitMode::kVoltage;
    m_lowerVoltage = lower;
    m_upperVoltage = upper;
    return AnalogTriggerResult::kOk;
  }

  AnalogTriggerResult SetAveraged(bool averaged) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_closed) {
      return AnalogTriggerResult::kInvalidHandle;
    }
    m_useAveraged = averaged;
    return AnalogTriggerResult::kOk;
  }

  AnalogTriggerResult SetFiltered(bool filtered) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_closed) {
      return AnalogTriggerResult::kInvalidHandle;
    }
    return filtered ? AnalogTriggerResult::kUnsupported
                    : AnalogTriggerResult::kOk;
  }

  std::pair<AnalogTriggerResult, bool> GetInWindow() noexcept {
    std::scoped_lock lock{m_mutex};
    auto value = ReadValueLocked();
    if (value.first != AnalogTriggerResult::kOk) {
      return {value.first, false};
    }
    return {AnalogTriggerResult::kOk, InWindowLocked(value.second)};
  }

  std::pair<AnalogTriggerResult, bool> GetTriggerState() noexcept {
    std::scoped_lock lock{m_mutex};
    auto value = ReadValueLocked();
    if (value.first != AnalogTriggerResult::kOk) {
      return {value.first, false};
    }
    if (value.second < LowerBoundLocked()) {
      m_triggerState = false;
    } else if (value.second > UpperBoundLocked()) {
      m_triggerState = true;
    }
    return {AnalogTriggerResult::kOk, m_triggerState};
  }

  std::pair<AnalogTriggerResult, bool> GetOutput(
      HAL_AnalogTriggerType type) noexcept {
    switch (type) {
      case HAL_Trigger_kInWindow:
        return GetInWindow();
      case HAL_Trigger_kState:
        return GetTriggerState();
      case HAL_Trigger_kRisingPulse:
      case HAL_Trigger_kFallingPulse:
        return {AnalogTriggerResult::kPulseOutput, false};
      default:
        return {AnalogTriggerResult::kOutOfRange, false};
    }
  }

  int32_t GetInputChannel() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_inputChannel;
  }

  void Close() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_closed) {
      return;
    }
    m_closed = true;
    m_input.reset();
  }

 private:
  AnalogTriggerResult CheckUsableLocked() const noexcept {
    if (m_closed) {
      return AnalogTriggerResult::kInvalidHandle;
    }
    return m_input && m_input->IsUsable()
               ? AnalogTriggerResult::kOk
               : AnalogTriggerResult::kHardwareFailure;
  }

  std::pair<AnalogTriggerResult, double> ReadValueLocked() noexcept {
    auto result = CheckUsableLocked();
    if (result != AnalogTriggerResult::kOk) {
      return {result, 0.0};
    }
    if (m_limitMode == AnalogTriggerLimitMode::kRaw) {
      if (m_useAveraged) {
        auto [readResult, value] = m_input->GetAverageValue();
        return {readResult == AnalogInputResult::kOk
                    ? AnalogTriggerResult::kOk
                    : AnalogTriggerResult::kHardwareFailure,
                static_cast<double>(value)};
      }
      auto [readResult, value] = m_input->GetValue();
      return {readResult == AnalogInputResult::kOk
                  ? AnalogTriggerResult::kOk
                  : AnalogTriggerResult::kHardwareFailure,
              static_cast<double>(value)};
    }

    if (m_useAveraged) {
      auto [readResult, value] = m_input->GetAverageVoltage();
      return {readResult == AnalogInputResult::kOk
                  ? AnalogTriggerResult::kOk
                  : AnalogTriggerResult::kHardwareFailure,
              value};
    }
    auto [readResult, value] = m_input->GetVoltage();
    return {readResult == AnalogInputResult::kOk
                ? AnalogTriggerResult::kOk
                : AnalogTriggerResult::kHardwareFailure,
            value};
  }

  double LowerBoundLocked() const noexcept {
    return m_limitMode == AnalogTriggerLimitMode::kRaw
               ? static_cast<double>(m_lowerRaw)
               : m_lowerVoltage;
  }

  double UpperBoundLocked() const noexcept {
    return m_limitMode == AnalogTriggerLimitMode::kRaw
               ? static_cast<double>(m_upperRaw)
               : m_upperVoltage;
  }

  bool InWindowLocked(double value) const noexcept {
    return value >= LowerBoundLocked() && value <= UpperBoundLocked();
  }

  mutable std::mutex m_mutex;
  std::shared_ptr<AnalogInputPort> m_input;
  int32_t m_inputChannel = -1;
  AnalogTriggerLimitMode m_limitMode = AnalogTriggerLimitMode::kRaw;
  int32_t m_lowerRaw = 0;
  int32_t m_upperRaw = 0;
  double m_lowerVoltage = 0.0;
  double m_upperVoltage = 0.0;
  bool m_useAveraged = false;
  bool m_triggerState = false;
  bool m_closed = false;
};

struct AnalogTriggerAllocationResult {
  HAL_AnalogTriggerHandle handle = HAL_kInvalidHandle;
  AnalogTriggerResult result = AnalogTriggerResult::kHardwareFailure;
};

using AnalogInputPortProvider =
    std::function<std::shared_ptr<AnalogInputPort>(HAL_AnalogInputHandle)>;

class AnalogTriggerHandleResource final
    : public hal::LimitedClassedHandleResource<HAL_AnalogTriggerHandle,
                                               AnalogTriggerPort,
                                               kNumVMXAnalogTriggers,
                                               HAL_HandleEnum::AnalogTrigger> {
 public:
  AnalogTriggerHandleResource() { m_version = 0; }
};

class AnalogTriggerManager final {
 public:
  explicit AnalogTriggerManager(AnalogInputPortProvider provider)
      : m_provider{std::move(provider)} {}

  AnalogTriggerAllocationResult Allocate(HAL_AnalogInputHandle inputHandle) {
    std::shared_ptr<AnalogInputPort> input;
    try {
      input = m_provider ? m_provider(inputHandle) : nullptr;
    } catch (...) {
      input.reset();
    }
    if (!input || !input->IsUsable()) {
      return {HAL_kInvalidHandle, AnalogTriggerResult::kInvalidHandle};
    }

    auto trigger = std::make_shared<AnalogTriggerPort>(std::move(input));
    auto handle = m_handles.Allocate(trigger);
    if (handle == HAL_kInvalidHandle) {
      return {HAL_kInvalidHandle, AnalogTriggerResult::kNoResources};
    }
    return {handle, AnalogTriggerResult::kOk};
  }

  void Free(HAL_AnalogTriggerHandle handle) noexcept {
    auto trigger = Get(handle);
    if (!trigger) {
      return;
    }
    trigger->Close();
    m_handles.Free(handle);
  }

  bool IsValid(HAL_AnalogTriggerHandle handle) noexcept {
    return Get(handle) != nullptr;
  }

  int32_t GetIndex(HAL_AnalogTriggerHandle handle) noexcept {
    return Get(handle) ? m_handles.GetIndex(handle) : -1;
  }

  AnalogTriggerResult SetLimitsRaw(HAL_AnalogTriggerHandle handle,
                                   int32_t lower, int32_t upper) noexcept {
    auto trigger = Get(handle);
    return trigger ? trigger->SetLimitsRaw(lower, upper)
                   : AnalogTriggerResult::kInvalidHandle;
  }

  AnalogTriggerResult SetLimitsVoltage(HAL_AnalogTriggerHandle handle,
                                       double lower, double upper) noexcept {
    auto trigger = Get(handle);
    return trigger ? trigger->SetLimitsVoltage(lower, upper)
                   : AnalogTriggerResult::kInvalidHandle;
  }

  AnalogTriggerResult SetAveraged(HAL_AnalogTriggerHandle handle,
                                  bool averaged) noexcept {
    auto trigger = Get(handle);
    return trigger ? trigger->SetAveraged(averaged)
                   : AnalogTriggerResult::kInvalidHandle;
  }

  AnalogTriggerResult SetFiltered(HAL_AnalogTriggerHandle handle,
                                  bool filtered) noexcept {
    auto trigger = Get(handle);
    return trigger ? trigger->SetFiltered(filtered)
                   : AnalogTriggerResult::kInvalidHandle;
  }

  std::pair<AnalogTriggerResult, bool> GetInWindow(
      HAL_AnalogTriggerHandle handle) noexcept {
    auto trigger = Get(handle);
    return trigger ? trigger->GetInWindow()
                   : std::pair{AnalogTriggerResult::kInvalidHandle, false};
  }

  std::pair<AnalogTriggerResult, bool> GetTriggerState(
      HAL_AnalogTriggerHandle handle) noexcept {
    auto trigger = Get(handle);
    return trigger ? trigger->GetTriggerState()
                   : std::pair{AnalogTriggerResult::kInvalidHandle, false};
  }

  std::pair<AnalogTriggerResult, bool> GetOutput(
      HAL_AnalogTriggerHandle handle, HAL_AnalogTriggerType type) noexcept {
    auto trigger = Get(handle);
    return trigger ? trigger->GetOutput(type)
                   : std::pair{AnalogTriggerResult::kInvalidHandle, false};
  }

 private:
  std::shared_ptr<AnalogTriggerPort> Get(
      HAL_AnalogTriggerHandle handle) noexcept {
    return m_handles.Get(handle);
  }

  AnalogInputPortProvider m_provider;
  AnalogTriggerHandleResource m_handles;
};

AnalogTriggerManager& GetAnalogTriggerManager();

}  // namespace hal::vmx
