// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/AnalogInput.h"

#include <memory>
#include <string_view>

#include "VMXPi.h"

#include "AnalogInputInternal.h"
#include "HALInitializer.h"
#include "HALInternal.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"

namespace hal::vmx {
namespace {

class DriverAnalogInputBackend final : public AnalogInputBackend {
 public:
  DriverAnalogInputBackend(int32_t physicalChannel, int32_t averageBits,
                           int32_t oversampleBits, bool accumulatorEnabled,
                           int32_t center, int32_t deadband,
                           std::shared_ptr<VMXPi> context)
      : m_context{std::move(context)} {
    if (!m_context || !m_context->IsOpen()) {
      return;
    }
    VMXErrorCode error;
    float fullScale = 0.0f;
    if (!m_context->io.Accumulator_GetFullScaleVoltage(fullScale, &error) ||
        fullScale <= 0.0f) {
      return;
    }
    AccumulatorConfig config;
    config.SetNumAverageBits(static_cast<uint8_t>(averageBits));
    config.SetNumOversampleBits(static_cast<uint8_t>(oversampleBits));
    config.SetEnableAccumulationCounter(accumulatorEnabled);
    config.SetAccumulationCounterCenter(static_cast<int16_t>(center));
    config.SetAccumulationCounterDeadband(static_cast<int16_t>(deadband));
    if (!m_context->io.ActivateSinglechannelResource(
            ::VMXChannelInfo(physicalChannel,
                           VMXChannelCapability::AccumulatorInput),
            &config, m_resourceHandle, &error)) {
      return;
    }
    m_fullScaleVoltage = fullScale;
    m_accumulatorEnabled = accumulatorEnabled;
    m_initialized = true;
  }

  ~DriverAnalogInputBackend() override {
    if (m_initialized) {
      VMXErrorCode error;
      m_context->io.DeallocateResource(m_resourceHandle, &error);
    }
  }

  bool IsInitialized() const noexcept { return m_initialized; }

  bool GetValue(uint32_t& value) noexcept override {
    try {
      if (!m_initialized) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.Accumulator_GetInstantaneousValue(
          m_resourceHandle, value, &error);
    } catch (...) {
      value = 0;
      return false;
    }
  }

  bool GetAverageValue(uint32_t& value) noexcept override {
    try {
      if (!m_initialized) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.Accumulator_GetAverageValue(m_resourceHandle, value,
                                                       &error);
    } catch (...) {
      value = 0;
      return false;
    }
  }

  bool GetVoltage(double& voltage) noexcept override {
    try {
      uint32_t raw = 0;
      if (!GetValue(raw)) {
        voltage = 0.0;
        return false;
      }
      voltage = static_cast<double>(raw) * m_fullScaleVoltage / kVMXADCCounts;
      return true;
    } catch (...) {
      voltage = 0.0;
      return false;
    }
  }

  bool GetAverageVoltage(double& voltage) noexcept override {
    try {
      if (!m_initialized) {
        voltage = 0.0;
        return false;
      }
      float value = 0.0f;
      VMXErrorCode error;
      if (!m_context->io.Accumulator_GetAverageVoltage(m_resourceHandle, value,
                                                        &error)) {
        voltage = 0.0;
        return false;
      }
      voltage = value;
      return true;
    } catch (...) {
      voltage = 0.0;
      return false;
    }
  }

  bool ResetAccumulator() noexcept override {
    try {
      if (!m_initialized || !m_accumulatorEnabled) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.Accumulator_Counter_Reset(m_resourceHandle, &error);
    } catch (...) {
      return false;
    }
  }

  bool GetAccumulatorOutput(int64_t& value,
                            uint32_t& count) noexcept override {
    try {
      if (!m_initialized || !m_accumulatorEnabled) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.Accumulator_Counter_GetValueAndCount(
          m_resourceHandle, value, count, &error);
    } catch (...) {
      value = 0;
      count = 0;
      return false;
    }
  }

  double GetFullScaleVoltage() const noexcept override {
    return m_fullScaleVoltage;
  }

 private:
  std::shared_ptr<VMXPi> m_context;
  VMXResourceHandle m_resourceHandle = 0;
  double m_fullScaleVoltage = 0.0;
  bool m_accumulatorEnabled = false;
  bool m_initialized = false;
};

std::unique_ptr<AnalogInputBackend> CreateAnalogInputBackend(
    int32_t physicalChannel, const AnalogInputConfig& config) {
  auto context = GetRuntimeContext();
  if (!context) {
    return nullptr;
  }
  auto backend = std::make_unique<DriverAnalogInputBackend>(
      physicalChannel, config.averageBits, config.oversampleBits,
      config.accumulatorEnabled, config.center, config.deadband,
      std::move(context));
  if (!backend->IsInitialized()) {
    return nullptr;
  }
  return backend;
}

void SetAnalogHardwareError(int32_t* status, std::string_view message) {
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(status, message);
}

void SetAnalogUnsupported(int32_t* status, std::string_view feature) {
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(status, feature);
}

void SetAnalogResult(AnalogInputResult result, int32_t* status,
                     std::string_view hardwareMessage) {
  switch (result) {
    case AnalogInputResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case AnalogInputResult::kInvalidHandle:
      *status = HAL_HANDLE_ERROR;
      return;
    case AnalogInputResult::kOutOfRange:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status,
                        "VMX analog average/oversample bits must be in the "
                        "range [0, 255]");
      return;
    case AnalogInputResult::kRollbackFailure:
      SetAnalogHardwareError(
          status,
          "VMX analog reconfiguration and rollback both failed; "
          "resource is faulted");
      return;
    default:
      SetAnalogHardwareError(status, hardwareMessage);
      return;
  }
}

}  // namespace

AnalogInputManager& GetAnalogInputManager() {
  static AnalogInputManager manager{CreateAnalogInputBackend};
  return manager;
}
}  // namespace hal::vmx

extern "C" {

HAL_AnalogInputHandle HAL_InitializeAnalogInputPort(
    HAL_PortHandle portHandle, const char* allocationLocation,
    int32_t* status) {
  hal::init::CheckInit();
  if (!hal::vmx::IsRuntimeInitialized()) {
    hal::vmx::SetAnalogHardwareError(status,
                                     "VMX HAL runtime is not initialized");
    return HAL_kInvalidHandle;
  }

  int16_t logicalChannel = hal::getPortHandleChannel(portHandle);
  if (!HAL_CheckAnalogInputChannel(logicalChannel)) {
    *status = RESOURCE_OUT_OF_RANGE;
    hal::SetLastErrorIndexOutOfRange(
        status, "Invalid Index for VMX Analog Input", 0,
        hal::vmx::kNumAnalogInputs - 1, logicalChannel);
    return HAL_kInvalidHandle;
  }

  auto result = hal::vmx::GetAnalogInputManager().Allocate(
      logicalChannel, allocationLocation ? allocationLocation : "");
  if (result.result == hal::vmx::AnalogInputResult::kAlreadyAllocated) {
    *status = RESOURCE_IS_ALLOCATED;
    hal::SetLastErrorPreviouslyAllocated(status, "Analog Input", logicalChannel,
                                         result.previousAllocation);
    return HAL_kInvalidHandle;
  }
  if (result.result != hal::vmx::AnalogInputResult::kOk) {
    hal::vmx::SetAnalogHardwareError(
        status, "Failed to activate the VMX analog input resource");
    return HAL_kInvalidHandle;
  }
  *status = HAL_SUCCESS;
  return result.handle;
}

void HAL_FreeAnalogInputPort(HAL_AnalogInputHandle analogPortHandle) {
  hal::vmx::GetAnalogInputManager().Free(analogPortHandle);
}

HAL_Bool HAL_CheckAnalogModule(int32_t module) {
  return module == 1;
}

HAL_Bool HAL_CheckAnalogInputChannel(int32_t channel) {
  return hal::vmx::IsAnalogInputChannelValid(channel);
}

void HAL_SetAnalogInputSimDevice(HAL_AnalogInputHandle handle,
                                 HAL_SimDeviceHandle device) {
  static_cast<void>(handle);
  static_cast<void>(device);
}

void HAL_SetAnalogSampleRate(double samplesPerSecond, int32_t* status) {
  static_cast<void>(samplesPerSecond);
  hal::vmx::SetAnalogUnsupported(
      status, "VMX analog input sample rate is fixed by the hardware");
}

double HAL_GetAnalogSampleRate(int32_t* status) {
  *status = HAL_SUCCESS;
  return hal::vmx::kVMXAnalogSampleRate;
}

void HAL_SetAnalogAverageBits(HAL_AnalogInputHandle analogPortHandle,
                              int32_t bits, int32_t* status) {
  hal::vmx::SetAnalogResult(
      hal::vmx::GetAnalogInputManager().SetAverageBits(analogPortHandle, bits),
      status,
      "VMX analog average-bits change failed; previous configuration "
      "restored");
}

int32_t HAL_GetAnalogAverageBits(HAL_AnalogInputHandle analogPortHandle,
                                 int32_t* status) {
  auto [result, bits] =
      hal::vmx::GetAnalogInputManager().GetAverageBits(analogPortHandle);
  hal::vmx::SetAnalogResult(result, status,
                            "Failed to read VMX analog average bits");
  return result == hal::vmx::AnalogInputResult::kOk ? bits : 0;
}

void HAL_SetAnalogOversampleBits(HAL_AnalogInputHandle analogPortHandle,
                                 int32_t bits, int32_t* status) {
  hal::vmx::SetAnalogResult(
      hal::vmx::GetAnalogInputManager().SetOversampleBits(analogPortHandle,
                                                          bits),
      status,
      "VMX analog oversample-bits change failed; previous configuration "
      "restored");
}

int32_t HAL_GetAnalogOversampleBits(HAL_AnalogInputHandle analogPortHandle,
                                    int32_t* status) {
  auto [result, bits] =
      hal::vmx::GetAnalogInputManager().GetOversampleBits(analogPortHandle);
  hal::vmx::SetAnalogResult(result, status,
                            "Failed to read VMX analog oversample bits");
  return result == hal::vmx::AnalogInputResult::kOk ? bits : 0;
}

int32_t HAL_GetAnalogValue(HAL_AnalogInputHandle analogPortHandle,
                           int32_t* status) {
  auto [result, value] =
      hal::vmx::GetAnalogInputManager().GetValue(analogPortHandle);
  hal::vmx::SetAnalogResult(result, status,
                            "Failed to read instantaneous VMX analog value");
  return result == hal::vmx::AnalogInputResult::kOk ? value : 0;
}

int32_t HAL_GetAnalogAverageValue(HAL_AnalogInputHandle analogPortHandle,
                                  int32_t* status) {
  auto [result, value] =
      hal::vmx::GetAnalogInputManager().GetAverageValue(analogPortHandle);
  hal::vmx::SetAnalogResult(result, status,
                            "Failed to read averaged VMX analog value");
  return result == hal::vmx::AnalogInputResult::kOk ? value : 0;
}

int32_t HAL_GetAnalogVoltsToValue(HAL_AnalogInputHandle analogPortHandle,
                                  double voltage, int32_t* status) {
  auto converted =
      hal::vmx::GetAnalogInputManager().VoltsToValue(analogPortHandle, voltage);
  hal::vmx::SetAnalogResult(converted.result, status,
                            "Failed to convert VMX analog voltage");
  if (converted.result != hal::vmx::AnalogInputResult::kOk) {
    return 0;
  }
  if (converted.clamped) {
    *status = VOLTAGE_OUT_OF_RANGE;
  }
  return converted.value;
}

double HAL_GetAnalogVoltage(HAL_AnalogInputHandle analogPortHandle,
                            int32_t* status) {
  auto [result, voltage] =
      hal::vmx::GetAnalogInputManager().GetVoltage(analogPortHandle);
  hal::vmx::SetAnalogResult(result, status,
                            "Failed to read instantaneous VMX analog voltage");
  return result == hal::vmx::AnalogInputResult::kOk ? voltage : 0.0;
}

double HAL_GetAnalogValueToVolts(HAL_AnalogInputHandle analogPortHandle,
                                 int32_t rawValue, int32_t* status) {
  auto [result, voltage] = hal::vmx::GetAnalogInputManager().ValueToVolts(
      analogPortHandle, rawValue);
  hal::vmx::SetAnalogResult(result, status,
                            "Failed to convert VMX analog raw value");
  return result == hal::vmx::AnalogInputResult::kOk ? voltage : 0.0;
}

double HAL_GetAnalogAverageVoltage(HAL_AnalogInputHandle analogPortHandle,
                                   int32_t* status) {
  auto [result, voltage] =
      hal::vmx::GetAnalogInputManager().GetAverageVoltage(analogPortHandle);
  hal::vmx::SetAnalogResult(result, status,
                            "Failed to read averaged VMX analog voltage");
  return result == hal::vmx::AnalogInputResult::kOk ? voltage : 0.0;
}

int32_t HAL_GetAnalogLSBWeight(HAL_AnalogInputHandle analogPortHandle,
                               int32_t* status) {
  auto [result, weight] =
      hal::vmx::GetAnalogInputManager().GetLSBWeight(analogPortHandle);
  hal::vmx::SetAnalogResult(result, status,
                            "Failed to read VMX analog full-scale voltage");
  return result == hal::vmx::AnalogInputResult::kOk ? weight : 0;
}

int32_t HAL_GetAnalogOffset(HAL_AnalogInputHandle analogPortHandle,
                            int32_t* status) {
  auto [result, weight] =
      hal::vmx::GetAnalogInputManager().GetLSBWeight(analogPortHandle);
  static_cast<void>(weight);
  hal::vmx::SetAnalogResult(result, status,
                            "Failed to validate VMX analog input handle");
  return 0;
}

}  // extern "C"
