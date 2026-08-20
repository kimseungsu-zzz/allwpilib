// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/DIO.h"

#include <memory>
#include <string_view>

#include "DIOInternal.h"
#include "HALInitializer.h"
#include "HALInternal.h"
#include "VMXRuntime.h"
#include "dio.hpp"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"

namespace hal::vmx {
namespace {

class DriverDIOBackend final : public DIOBackend {
 public:
  DriverDIOBackend(int32_t channel, bool input,
                   std::shared_ptr<VMXPi> context)
      : m_driver{std::make_unique<studica_driver::DIO>(
            channel,
            input ? studica_driver::PinMode::INPUT
                  : studica_driver::PinMode::OUTPUT,
            std::move(context))} {}

  bool IsInitialized() const noexcept {
    return m_driver && m_driver->IsInitialized();
  }

  bool Set(bool value) noexcept override {
    try {
      return m_driver && m_driver->Set(value);
    } catch (...) {
      return false;
    }
  }

  bool Get(bool& value) noexcept override {
    try {
      return m_driver && m_driver->Get(value);
    } catch (...) {
      value = false;
      return false;
    }
  }

 private:
  std::unique_ptr<studica_driver::DIO> m_driver;
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

DIOManager& GetDIOManager() {
  static DIOManager manager{CreateDIOBackend};
  return manager;
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
  static_cast<void>(pulseLengthSeconds);
  if (hal::vmx::CheckHandleForUnsupported(dioPortHandle, status)) {
    hal::vmx::SetUnsupported(status, "DIO pulse generation");
  }
}

void HAL_PulseMultiple(uint32_t channelMask, double pulseLengthSeconds,
                       int32_t* status) {
  static_cast<void>(channelMask);
  static_cast<void>(pulseLengthSeconds);
  hal::vmx::SetUnsupported(status, "DIO pulse generation");
}

HAL_Bool HAL_IsPulsing(HAL_DigitalHandle dioPortHandle, int32_t* status) {
  if (hal::vmx::CheckHandleForUnsupported(dioPortHandle, status)) {
    hal::vmx::SetUnsupported(status, "DIO pulse generation");
  }
  return false;
}

HAL_Bool HAL_IsAnyPulsing(int32_t* status) {
  hal::vmx::SetUnsupported(status, "DIO pulse generation");
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
