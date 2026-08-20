// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <climits>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

#include "AnalogInputInternal.h"
#include "VMXConstants.h"
#include "hal/AnalogGyro.h"
#include "hal/handles/IndexedHandleResource.h"

namespace hal::vmx {

constexpr uint32_t kAnalogGyroAverageBits = 0;
constexpr uint32_t kAnalogGyroOversampleBits = 10;
constexpr double kAnalogGyroSampleRate = kVMXAnalogSampleRate;
constexpr double kAnalogGyroCalibrationSeconds = 5.0;
constexpr double kAnalogGyroSetupSettleSeconds = 0.1;
constexpr double kDefaultAnalogGyroVoltsPerDegreePerSecond = 0.007;

enum class AnalogGyroResult {
  kOk,
  kInvalidHandle,
  kInvalidChannel,
  kAlreadyAllocated,
  kInvalidParameter,
  kZeroCount,
  kHardwareFailure,
};

using AnalogGyroWait = std::function<void(double)>;

/**
 * Adapter-side AnalogGyro implementation.  The gyro never owns a second VMX
 * resource: it retains the existing AnalogInputPort and enables that port's
 * accumulator configuration.
 */
class AnalogGyroPort final {
 public:
  explicit AnalogGyroPort(AnalogGyroWait wait = {})
      : m_wait{wait ? std::move(wait)
                    : [](double seconds) {
                        if (seconds > 0.0) {
                          std::this_thread::sleep_for(
                              std::chrono::duration<double>(seconds));
                        }
                      }} {}

  AnalogGyroResult Attach(std::shared_ptr<AnalogInputPort> input,
                          std::string_view allocationLocation) noexcept {
    if (!input || !IsAnalogAccumulatorChannelValid(
                      input->GetLogicalChannel())) {
      return AnalogGyroResult::kInvalidChannel;
    }
    std::scoped_lock lock{m_mutex};
    m_input = std::move(input);
    m_previousAllocation = allocationLocation;
    return AnalogGyroResult::kOk;
  }

  void SetWait(AnalogGyroWait wait) noexcept {
    std::scoped_lock lock{m_mutex};
    if (wait) {
      m_wait = std::move(wait);
    }
  }

  AnalogGyroResult Setup() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_input) {
      return AnalogGyroResult::kInvalidHandle;
    }
    if (m_input->SetAverageBits(kAnalogGyroAverageBits) !=
            AnalogInputResult::kOk ||
        m_input->SetOversampleBits(kAnalogGyroOversampleBits) !=
            AnalogInputResult::kOk ||
        m_input->InitAccumulator() != AnalogInputResult::kOk) {
      return AnalogGyroResult::kHardwareFailure;
    }
    if (SetDeadbandLocked(0.0) != AnalogGyroResult::kOk) {
      return AnalogGyroResult::kHardwareFailure;
    }
    try {
      m_wait(kAnalogGyroSetupSettleSeconds);
    } catch (...) {
      return AnalogGyroResult::kHardwareFailure;
    }
    return AnalogGyroResult::kOk;
  }

  AnalogGyroResult SetParameters(double voltsPerDegreePerSecond,
                                 double offset, int32_t center) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_input) {
      return AnalogGyroResult::kInvalidHandle;
    }
    if (!std::isfinite(voltsPerDegreePerSecond) ||
        voltsPerDegreePerSecond <= 0.0 || !std::isfinite(offset) ||
        center < INT16_MIN || center > INT16_MAX) {
      return AnalogGyroResult::kInvalidParameter;
    }
    if (m_input->SetAccumulatorCenter(center) != AnalogInputResult::kOk) {
      return AnalogGyroResult::kHardwareFailure;
    }
    m_voltsPerDegreePerSecond = voltsPerDegreePerSecond;
    m_offset = offset;
    m_center = center;
    return AnalogGyroResult::kOk;
  }

  AnalogGyroResult SetSensitivity(double voltsPerDegreePerSecond) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_input) {
      return AnalogGyroResult::kInvalidHandle;
    }
    if (!std::isfinite(voltsPerDegreePerSecond) ||
        voltsPerDegreePerSecond <= 0.0) {
      return AnalogGyroResult::kInvalidParameter;
    }
    m_voltsPerDegreePerSecond = voltsPerDegreePerSecond;
    return AnalogGyroResult::kOk;
  }

  AnalogGyroResult Reset() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_input) {
      return AnalogGyroResult::kInvalidHandle;
    }
    if (m_input->ResetAccumulator() != AnalogInputResult::kOk) {
      return AnalogGyroResult::kHardwareFailure;
    }
    try {
      const auto average = m_input->GetAverageBits();
      const auto oversample = m_input->GetOversampleBits();
      if (average.first != AnalogInputResult::kOk ||
          oversample.first != AnalogInputResult::kOk) {
        return AnalogGyroResult::kHardwareFailure;
      }
      const double samplePeriod =
          (1.0 / kAnalogGyroSampleRate) *
          std::ldexp(1.0, oversample.second) *
          std::ldexp(1.0, average.second);
      m_wait(samplePeriod);
    } catch (...) {
      return AnalogGyroResult::kHardwareFailure;
    }
    return AnalogGyroResult::kOk;
  }

  AnalogGyroResult Calibrate() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_input) {
      return AnalogGyroResult::kInvalidHandle;
    }
    if (m_input->InitAccumulator() != AnalogInputResult::kOk) {
      return AnalogGyroResult::kHardwareFailure;
    }
    try {
      m_wait(kAnalogGyroCalibrationSeconds);
    } catch (...) {
      return AnalogGyroResult::kHardwareFailure;
    }
    auto [readResult, output] = m_input->GetAccumulatorOutput();
    if (readResult != AnalogInputResult::kOk) {
      return AnalogGyroResult::kHardwareFailure;
    }
    if (output.count <= 0) {
      return AnalogGyroResult::kZeroCount;
    }
    const double average = static_cast<double>(output.value) /
                           static_cast<double>(output.count);
    const auto newCenter = static_cast<int32_t>(std::llround(average));
    const auto oldCenter = m_center;
    const auto oldOffset = m_offset;
    if (m_input->SetAccumulatorCenter(newCenter) != AnalogInputResult::kOk) {
      return AnalogGyroResult::kHardwareFailure;
    }
    if (m_input->ResetAccumulator() != AnalogInputResult::kOk) {
      // Keep the adapter state and the hardware configuration coherent if a
      // reset fails after the center update.
      static_cast<void>(m_input->SetAccumulatorCenter(oldCenter));
      m_center = oldCenter;
      m_offset = oldOffset;
      return AnalogGyroResult::kHardwareFailure;
    }
    m_center = newCenter;
    m_offset = average - static_cast<double>(newCenter);
    return AnalogGyroResult::kOk;
  }

  AnalogGyroResult SetDeadband(double volts) noexcept {
    std::scoped_lock lock{m_mutex};
    return SetDeadbandLocked(volts);
  }

  std::pair<AnalogGyroResult, double> GetAngle() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_input) {
      return {AnalogGyroResult::kInvalidHandle, 0.0};
    }
    auto [result, output] = m_input->GetAccumulatorOutput();
    if (result != AnalogInputResult::kOk) {
      return {AnalogGyroResult::kHardwareFailure, 0.0};
    }
    auto [lsbResult, lsbWeight] = m_input->GetLSBWeight();
    auto [averageResult, averageBits] = m_input->GetAverageBits();
    if (lsbResult != AnalogInputResult::kOk ||
        averageResult != AnalogInputResult::kOk ||
        m_voltsPerDegreePerSecond <= 0.0) {
      return {AnalogGyroResult::kHardwareFailure, 0.0};
    }
    const double value = static_cast<double>(output.value) -
                         static_cast<double>(output.count) * m_offset;
    const double scaled =
        value * 1.0e-9 * static_cast<double>(lsbWeight) *
        std::ldexp(1.0, averageBits) /
        (kAnalogGyroSampleRate * m_voltsPerDegreePerSecond);
    return {AnalogGyroResult::kOk, scaled};
  }

  std::pair<AnalogGyroResult, double> GetRate() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_input) {
      return {AnalogGyroResult::kInvalidHandle, 0.0};
    }
    auto [valueResult, averageValue] = m_input->GetAverageValue();
    auto [lsbResult, lsbWeight] = m_input->GetLSBWeight();
    auto [oversampleResult, oversampleBits] = m_input->GetOversampleBits();
    if (valueResult != AnalogInputResult::kOk ||
        lsbResult != AnalogInputResult::kOk ||
        oversampleResult != AnalogInputResult::kOk ||
        m_voltsPerDegreePerSecond <= 0.0) {
      return {AnalogGyroResult::kHardwareFailure, 0.0};
    }
    const double centered = static_cast<double>(averageValue) -
                            (static_cast<double>(m_center) + m_offset);
    const double scale =
        1.0e-9 * static_cast<double>(lsbWeight) /
        (std::ldexp(1.0, oversampleBits) *
         m_voltsPerDegreePerSecond);
    return {AnalogGyroResult::kOk, centered * scale};
  }

  std::pair<AnalogGyroResult, double> GetOffset() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_input ? std::pair{AnalogGyroResult::kOk, m_offset}
                   : std::pair{AnalogGyroResult::kInvalidHandle, 0.0};
  }

  std::pair<AnalogGyroResult, int32_t> GetCenter() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_input ? std::pair{AnalogGyroResult::kOk, m_center}
                   : std::pair{AnalogGyroResult::kInvalidHandle, 0};
  }

  void Close() noexcept {
    std::scoped_lock lock{m_mutex};
    m_input.reset();
  }

  std::string_view PreviousAllocation() const noexcept {
    return m_previousAllocation;
  }

 private:
  AnalogGyroResult SetDeadbandLocked(double volts) noexcept {
    if (!m_input) {
      return AnalogGyroResult::kInvalidHandle;
    }
    if (!std::isfinite(volts) || volts < 0.0) {
      return AnalogGyroResult::kInvalidParameter;
    }
    auto [lsbResult, lsbWeight] = m_input->GetLSBWeight();
    auto [oversampleResult, oversampleBits] = m_input->GetOversampleBits();
    if (lsbResult != AnalogInputResult::kOk ||
        oversampleResult != AnalogInputResult::kOk || lsbWeight <= 0) {
      return AnalogGyroResult::kHardwareFailure;
    }
    const double rawDeadband =
        volts * 1.0e9 / static_cast<double>(lsbWeight) *
        std::ldexp(1.0, oversampleBits);
    if (!std::isfinite(rawDeadband) || rawDeadband > INT16_MAX) {
      return AnalogGyroResult::kInvalidParameter;
    }
    return m_input->SetAccumulatorDeadband(
               static_cast<int32_t>(rawDeadband)) == AnalogInputResult::kOk
               ? AnalogGyroResult::kOk
               : AnalogGyroResult::kHardwareFailure;
  }

  mutable std::mutex m_mutex;
  std::shared_ptr<AnalogInputPort> m_input;
  std::string m_previousAllocation;
  AnalogGyroWait m_wait;
  double m_voltsPerDegreePerSecond =
      kDefaultAnalogGyroVoltsPerDegreePerSecond;
  double m_offset = 0.0;
  int32_t m_center = 0;
};

class AnalogGyroHandleResource final
    : public hal::IndexedHandleResource<HAL_GyroHandle, AnalogGyroPort,
                                        kNumAnalogAccumulators,
                                        HAL_HandleEnum::AnalogGyro> {};

class AnalogGyroManager final {
 public:
  explicit AnalogGyroManager(AnalogGyroWait wait = {},
                             AnalogInputManager* inputManager = nullptr)
      : m_wait{std::move(wait)},
        m_inputManager{inputManager},
        m_handles{} {}

  AnalogGyroResult Initialize(HAL_AnalogInputHandle analogHandle,
                              std::string_view allocationLocation,
                              HAL_GyroHandle& handle) noexcept;
  AnalogGyroResult Setup(HAL_GyroHandle handle) noexcept;
  AnalogGyroResult SetParameters(HAL_GyroHandle handle,
                                 double voltsPerDegreePerSecond,
                                 double offset, int32_t center) noexcept;
  AnalogGyroResult SetSensitivity(HAL_GyroHandle handle,
                                  double voltsPerDegreePerSecond) noexcept;
  AnalogGyroResult Reset(HAL_GyroHandle handle) noexcept;
  AnalogGyroResult Calibrate(HAL_GyroHandle handle) noexcept;
  AnalogGyroResult SetDeadband(HAL_GyroHandle handle, double volts) noexcept;
  std::pair<AnalogGyroResult, double> GetAngle(HAL_GyroHandle handle) noexcept;
  std::pair<AnalogGyroResult, double> GetRate(HAL_GyroHandle handle) noexcept;
  std::pair<AnalogGyroResult, double> GetOffset(HAL_GyroHandle handle) noexcept;
  std::pair<AnalogGyroResult, int32_t> GetCenter(HAL_GyroHandle handle) noexcept;
  void Free(HAL_GyroHandle handle) noexcept;

 private:
  std::shared_ptr<AnalogGyroPort> Get(HAL_GyroHandle handle) noexcept {
    return m_handles.Get(handle);
  }

  AnalogGyroWait m_wait;
  AnalogInputManager* m_inputManager;
  AnalogGyroHandleResource m_handles;
  std::mutex m_allocationMutex;
};

AnalogGyroManager& GetAnalogGyroManager();

}  // namespace hal::vmx
