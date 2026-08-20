// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/DIO.h"

#include <cmath>
#include <limits>
#include <memory>
#include <string_view>

#include "VMXPi.h"

#include "DIOInternal.h"
#include "HALInitializer.h"
#include "HALInternal.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"

namespace hal::vmx {
namespace {

class DriverDIOBackend final : public DIOBackend {
 public:
  DriverDIOBackend(int32_t channel, bool input,
                   std::shared_ptr<VMXPi> context)
      : m_context{std::move(context)}, m_input{input} {
    if (!m_context || !m_context->IsOpen()) {
      return;
    }
    DIOConfig config;
    VMXChannelCapability capability;
    if (input) {
      config.SetInputMode(DIOConfig::InputMode::PULLUP);
      capability = VMXChannelCapability::DigitalInput;
    } else {
      config = DIOConfig{DIOConfig::PUSHPULL};
      capability = VMXChannelCapability::DigitalOutput;
    }
    VMXErrorCode error;
    m_initialized = m_context->io.ActivateSinglechannelResource(
        ::VMXChannelInfo(channel, capability), &config, m_resourceHandle,
        &error);
  }

  ~DriverDIOBackend() override {
    if (m_initialized) {
      VMXErrorCode error;
      m_context->io.DeallocateResource(m_resourceHandle, &error);
    }
  }

  bool IsInitialized() const noexcept { return m_initialized; }

  bool Set(bool value) noexcept override {
    try {
      if (!m_initialized || m_input) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.DIO_Set(m_resourceHandle, value, &error);
    } catch (...) {
      return false;
    }
  }

  bool Get(bool& value) noexcept override {
    try {
      if (!m_initialized) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.DIO_Get(m_resourceHandle, value, &error);
    } catch (...) {
      value = false;
      return false;
    }
  }

  bool Pulse(uint32_t microseconds) noexcept override {
    try {
      if (!m_initialized || m_input) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.DIO_Pulse(m_resourceHandle, true, microseconds,
                                     &error);
    } catch (...) {
      return false;
    }
  }

  bool IsPulsing(bool& isPulsing) noexcept override {
    try {
      if (!m_initialized || m_input) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.DIO_IsPulsing(m_resourceHandle, isPulsing, &error);
    } catch (...) {
      isPulsing = false;
      return false;
    }
  }

 private:
  std::shared_ptr<VMXPi> m_context;
  VMXResourceHandle m_resourceHandle = 0;
  bool m_input = true;
  bool m_initialized = false;
};

std::unique_ptr<DIOBackend> CreateDIOBackend(int32_t channel, bool input) {
  auto context = GetRuntimeContext();
  if (!context) {
    return nullptr;
  }

  auto backend =
      std::make_unique<DriverDIOBackend>(channel, input, std::move(context));
  if (!backend->IsInitialized()) {
    return nullptr;
  }
  return backend;
}

void SetHardwareError(int32_t* status, std::string_view message) {
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(status, message);
}

void SetUnsupported(int32_t* status, std::string_view feature) {
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(status,
                    std::string{"VMX HAL does not yet support "} +
                        std::string{feature});
}

bool CheckHandleForUnsupported(HAL_DigitalHandle handle, int32_t* status) {
  auto [result, input] = GetDIOManager().GetDirection(handle);
  static_cast<void>(input);
  if (result == DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }
  if (result != DIOResult::kOk) {
    SetHardwareError(status, "VMX DIO resource is faulted");
    return false;
  }
  return true;
}

}  // namespace

DIOManager& GetDIOManager() {
  static DIOManager manager{CreateDIOBackend, GetDigitalChannelRegistry(),
                            &GetVMXCapabilityProvider()};
  return manager;
}
}  // namespace hal::vmx

extern "C" {

HAL_DigitalHandle HAL_InitializeDIOPort(HAL_PortHandle portHandle,
                                        HAL_Bool input,
                                        const char* allocationLocation,
                                        int32_t* status) {
  hal::init::CheckInit();
  if (!hal::vmx::IsRuntimeInitialized()) {
    hal::vmx::SetHardwareError(status, "VMX HAL runtime is not initialized");
    return HAL_kInvalidHandle;
  }

  int16_t channel = hal::getPortHandleChannel(portHandle);
  if (!HAL_CheckDIOChannel(channel)) {
    *status = RESOURCE_OUT_OF_RANGE;
    hal::SetLastErrorIndexOutOfRange(status, "Invalid Index for VMX DIO", 0,
                                     hal::vmx::kNumDIOChannels - 1, channel);
    return HAL_kInvalidHandle;
  }

  auto result = hal::vmx::GetDIOManager().Allocate(
      channel, input != 0, allocationLocation ? allocationLocation : "");
  if (result.result == hal::vmx::DIOResult::kAlreadyAllocated) {
    *status = RESOURCE_IS_ALLOCATED;
    hal::SetLastErrorPreviouslyAllocated(status, "DIO", channel,
                                         result.previousAllocation);
    return HAL_kInvalidHandle;
  }
  if (result.result != hal::vmx::DIOResult::kOk) {
    if (result.result == hal::vmx::DIOResult::kUnsupportedCapability) {
      hal::vmx::SetHardwareError(
          status, "VMX DIO channel does not support the requested direction");
      return HAL_kInvalidHandle;
    }
    hal::vmx::SetHardwareError(
        status, "Failed to activate the VMX DIO hardware resource");
    return HAL_kInvalidHandle;
  }

  *status = HAL_SUCCESS;
  return result.handle;
}

HAL_Bool HAL_CheckDIOChannel(int32_t channel) {
  return hal::vmx::IsDIOChannelValid(channel);
}

void HAL_FreeDIOPort(HAL_DigitalHandle dioPortHandle) {
  hal::vmx::GetDIOManager().Free(dioPortHandle);
}

void HAL_SetDIOSimDevice(HAL_DigitalHandle handle,
                         HAL_SimDeviceHandle device) {
  static_cast<void>(handle);
  static_cast<void>(device);
}

void HAL_SetDIO(HAL_DigitalHandle dioPortHandle, HAL_Bool value,
                int32_t* status) {
  auto result = hal::vmx::GetDIOManager().Set(dioPortHandle, value != 0);
  switch (result) {
    case hal::vmx::DIOResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case hal::vmx::DIOResult::kInvalidHandle:
      *status = HAL_HANDLE_ERROR;
      return;
    case hal::vmx::DIOResult::kInputChannel:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status, "Cannot set output of an input channel");
      return;
    default:
      hal::vmx::SetHardwareError(status, "Failed to write VMX DIO resource");
      return;
  }
}

HAL_Bool HAL_GetDIO(HAL_DigitalHandle dioPortHandle, int32_t* status) {
  auto [result, value] = hal::vmx::GetDIOManager().GetValue(dioPortHandle);
  if (result == hal::vmx::DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }
  if (result != hal::vmx::DIOResult::kOk) {
    hal::vmx::SetHardwareError(status, "Failed to read VMX DIO resource");
    return false;
  }
  *status = HAL_SUCCESS;
  return value;
}

void HAL_SetDIODirection(HAL_DigitalHandle dioPortHandle, HAL_Bool input,
                         int32_t* status) {
  auto result =
      hal::vmx::GetDIOManager().SetDirection(dioPortHandle, input != 0);
  if (result == hal::vmx::DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  if (result == hal::vmx::DIOResult::kRollbackFailure) {
    hal::vmx::SetHardwareError(
        status, "VMX DIO direction change and rollback both failed; resource "
                "is faulted");
    return;
  }
  if (result != hal::vmx::DIOResult::kOk) {
    if (result == hal::vmx::DIOResult::kUnsupportedCapability) {
      hal::vmx::SetHardwareError(
          status, "VMX DIO channel does not support the requested direction");
      return;
    }
    hal::vmx::SetHardwareError(
        status, "VMX DIO direction change failed; previous direction restored");
    return;
  }
  *status = HAL_SUCCESS;
}

HAL_Bool HAL_GetDIODirection(HAL_DigitalHandle dioPortHandle,
                             int32_t* status) {
  auto [result, input] =
      hal::vmx::GetDIOManager().GetDirection(dioPortHandle);
  if (result == hal::vmx::DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }
  if (result != hal::vmx::DIOResult::kOk) {
    hal::vmx::SetHardwareError(status, "VMX DIO resource is faulted");
    return false;
  }
  *status = HAL_SUCCESS;
  return input;
}

HAL_DigitalPWMHandle HAL_AllocateDigitalPWM(int32_t* status) {
  hal::vmx::SetUnsupported(status, "Digital PWM over DIO");
  return HAL_kInvalidHandle;
}

void HAL_FreeDigitalPWM(HAL_DigitalPWMHandle pwmGenerator) {
  static_cast<void>(pwmGenerator);
}

void HAL_SetDigitalPWMRate(double rate, int32_t* status) {
  static_cast<void>(rate);
  hal::vmx::SetUnsupported(status, "Digital PWM over DIO");
}

void HAL_SetDigitalPWMDutyCycle(HAL_DigitalPWMHandle pwmGenerator,
                                double dutyCycle, int32_t* status) {
  static_cast<void>(pwmGenerator);
  static_cast<void>(dutyCycle);
  hal::vmx::SetUnsupported(status, "Digital PWM over DIO");
}

void HAL_SetDigitalPWMPPS(HAL_DigitalPWMHandle pwmGenerator, double dutyCycle,
                          int32_t* status) {
  static_cast<void>(pwmGenerator);
  static_cast<void>(dutyCycle);
  hal::vmx::SetUnsupported(status, "Digital PWM over DIO");
}

void HAL_SetDigitalPWMOutputChannel(HAL_DigitalPWMHandle pwmGenerator,
                                    int32_t channel, int32_t* status) {
  static_cast<void>(pwmGenerator);
  static_cast<void>(channel);
  hal::vmx::SetUnsupported(status, "Digital PWM over DIO");
}

void HAL_Pulse(HAL_DigitalHandle dioPortHandle, double pulseLengthSeconds,
               int32_t* status) {
  if (!std::isfinite(pulseLengthSeconds) || pulseLengthSeconds <= 0.0 ||
      pulseLengthSeconds >
          static_cast<double>(std::numeric_limits<uint32_t>::max()) / 1.0e6) {
    *status = PARAMETER_OUT_OF_RANGE;
    hal::SetLastError(status, "VMX DIO pulse length is out of range");
    return;
  }
  auto microseconds =
      static_cast<uint32_t>(std::llround(pulseLengthSeconds * 1.0e6));
  if (microseconds == 0) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  auto result = hal::vmx::GetDIOManager().Pulse(dioPortHandle, microseconds);
  if (result == hal::vmx::DIOResult::kOk) {
    *status = HAL_SUCCESS;
  } else if (result == hal::vmx::DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
  } else if (result == hal::vmx::DIOResult::kInputChannel) {
    *status = PARAMETER_OUT_OF_RANGE;
    hal::SetLastError(status, "Cannot pulse a VMX DIO input channel");
  } else {
    hal::vmx::SetHardwareError(status, "Failed to start VMX DIO pulse");
  }
}

void HAL_PulseMultiple(uint32_t channelMask, double pulseLengthSeconds,
                       int32_t* status) {
  if ((channelMask >> hal::vmx::kNumDIOChannels) != 0 ||
      !std::isfinite(pulseLengthSeconds) || pulseLengthSeconds <= 0.0 ||
      pulseLengthSeconds >
          static_cast<double>(std::numeric_limits<uint32_t>::max()) / 1.0e6) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  auto microseconds =
      static_cast<uint32_t>(std::llround(pulseLengthSeconds * 1.0e6));
  if (microseconds == 0) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  auto result =
      hal::vmx::GetDIOManager().PulseMultiple(channelMask, microseconds);
  if (result == hal::vmx::DIOResult::kOk) {
    *status = HAL_SUCCESS;
  } else if (result == hal::vmx::DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
  } else if (result == hal::vmx::DIOResult::kInputChannel) {
    *status = PARAMETER_OUT_OF_RANGE;
  } else {
    hal::vmx::SetHardwareError(status,
                               "Failed to start multiple VMX DIO pulses");
  }
}

HAL_Bool HAL_IsPulsing(HAL_DigitalHandle dioPortHandle, int32_t* status) {
  auto [result, pulsing] =
      hal::vmx::GetDIOManager().IsPulsing(dioPortHandle);
  if (result == hal::vmx::DIOResult::kOk) {
    *status = HAL_SUCCESS;
    return pulsing;
  }
  if (result == hal::vmx::DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
  } else if (result == hal::vmx::DIOResult::kInputChannel) {
    *status = PARAMETER_OUT_OF_RANGE;
  } else {
    hal::vmx::SetHardwareError(status, "Failed to read VMX DIO pulse state");
  }
  return false;
}

HAL_Bool HAL_IsAnyPulsing(int32_t* status) {
  auto [result, pulsing] = hal::vmx::GetDIOManager().IsAnyPulsing();
  if (result == hal::vmx::DIOResult::kOk) {
    *status = HAL_SUCCESS;
    return pulsing;
  }
  hal::vmx::SetHardwareError(status, "Failed to read VMX DIO pulse state");
  return false;
}

void HAL_SetFilterSelect(HAL_DigitalHandle dioPortHandle, int32_t filterIndex,
                         int32_t* status) {
  static_cast<void>(filterIndex);
  if (hal::vmx::CheckHandleForUnsupported(dioPortHandle, status)) {
    hal::vmx::SetUnsupported(status, "DIO glitch filters");
  }
}

int32_t HAL_GetFilterSelect(HAL_DigitalHandle dioPortHandle,
                            int32_t* status) {
  if (hal::vmx::CheckHandleForUnsupported(dioPortHandle, status)) {
    hal::vmx::SetUnsupported(status, "DIO glitch filters");
  }
  return 0;
}

void HAL_SetFilterPeriod(int32_t filterIndex, int64_t value,
                         int32_t* status) {
  static_cast<void>(filterIndex);
  static_cast<void>(value);
  hal::vmx::SetUnsupported(status, "DIO glitch filters");
}

int64_t HAL_GetFilterPeriod(int32_t filterIndex, int32_t* status) {
  static_cast<void>(filterIndex);
  hal::vmx::SetUnsupported(status, "DIO glitch filters");
  return 0;
}

}  // extern "C"
