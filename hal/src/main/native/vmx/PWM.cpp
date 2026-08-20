// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/PWM.h"

#include <memory>
#include <string>
#include <string_view>

#include "HALInitializer.h"
#include "HALInternal.h"
#include "PWMInternal.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"
#include "pwm.hpp"

namespace hal::vmx {
namespace {

class DriverPWMBackend final : public PWMBackend {
 public:
  DriverPWMBackend(int32_t channel, std::shared_ptr<VMXPi> context)
      : m_driver{std::make_unique<studica_driver::PWM>(
            channel, studica_driver::PWMType::Standard, -150, 150,
            std::move(context))} {}

  bool IsInitialized() const noexcept {
    return m_driver && m_driver->IsInitialized();
  }

  bool SetPulseTimeMicroseconds(int32_t requested,
                                int32_t& applied) noexcept override {
    try {
      return m_driver && m_driver->SetPulseTimeMicroseconds(requested) &&
             m_driver->GetLastPulseTimeMicroseconds(applied);
    } catch (...) {
      applied = 0;
      return false;
    }
  }

  bool GetPulseTimeMicroseconds(int32_t& pulse) noexcept override {
    try {
      return m_driver && m_driver->GetPulseTimeMicroseconds(pulse);
    } catch (...) {
      pulse = 0;
      return false;
    }
  }

  bool Disable() noexcept override {
    try {
      return m_driver && m_driver->Disable();
    } catch (...) {
      return false;
    }
  }

 private:
  std::unique_ptr<studica_driver::PWM> m_driver;
};

std::unique_ptr<PWMBackend> CreatePWMBackend(int32_t channel) {
  auto context = GetRuntimeContext();
  if (!context) {
    return nullptr;
  }
  auto backend =
      std::make_unique<DriverPWMBackend>(channel, std::move(context));
  return backend->IsInitialized() ? std::move(backend) : nullptr;
}

PWMManager& GetPWMManager() {
  static PWMManager manager{CreatePWMBackend};
  return manager;
}

void SetPWMHardwareError(int32_t* status, std::string_view message) {
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(status, message);
}

void SetPWMUnsupported(int32_t* status, std::string_view feature) {
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(
      status, std::string{"VMX HAL does not support "} + std::string{feature});
}

void SetPWMResult(PWMResult result, int32_t* status,
                  std::string_view hardwareMessage) {
  switch (result) {
    case PWMResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case PWMResult::kInvalidHandle:
      *status = HAL_HANDLE_ERROR;
      return;
    case PWMResult::kOutOfRange:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status,
                        "PWM pulse time must be in the range [0, 4095] us");
      return;
    case PWMResult::kScaleError:
      *status = HAL_PWM_SCALE_ERROR;
      return;
    default:
      SetPWMHardwareError(status, hardwareMessage);
      return;
  }
}

bool CheckPWMHandleForUnsupported(HAL_DigitalHandle handle, int32_t* status) {
  auto [result, pulse] = GetPWMManager().GetPulseTimeMicroseconds(handle);
  static_cast<void>(pulse);
  if (result == PWMResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }
  if (result != PWMResult::kOk) {
    SetPWMHardwareError(status, "VMX PWM resource is unavailable");
    return false;
  }
  return true;
}

}  // namespace
}  // namespace hal::vmx

extern "C" {

HAL_DigitalHandle HAL_InitializePWMPort(HAL_PortHandle portHandle,
                                        const char* allocationLocation,
                                        int32_t* status) {
  hal::init::CheckInit();
  if (!hal::vmx::IsRuntimeInitialized()) {
    hal::vmx::SetPWMHardwareError(status, "VMX HAL runtime is not initialized");
    return HAL_kInvalidHandle;
  }

  int16_t channel = hal::getPortHandleChannel(portHandle);
  if (!HAL_CheckPWMChannel(channel)) {
    *status = RESOURCE_OUT_OF_RANGE;
    hal::SetLastErrorIndexOutOfRange(status, "Invalid Index for VMX PWM", 0,
                                     hal::vmx::kNumPWMChannels - 1, channel);
    return HAL_kInvalidHandle;
  }

  auto result = hal::vmx::GetPWMManager().Allocate(
      channel, allocationLocation ? allocationLocation : "");
  if (result.result == hal::vmx::PWMResult::kAlreadyAllocated) {
    *status = RESOURCE_IS_ALLOCATED;
    hal::SetLastErrorPreviouslyAllocated(status, "PWM or DIO", channel,
                                         result.previousAllocation);
    return HAL_kInvalidHandle;
  }
  if (result.result != hal::vmx::PWMResult::kOk) {
    hal::vmx::SetPWMHardwareError(
        status, "Failed to activate the VMX PWM hardware resource");
    return HAL_kInvalidHandle;
  }
  *status = HAL_SUCCESS;
  return result.handle;
}

void HAL_FreePWMPort(HAL_DigitalHandle pwmPortHandle) {
  hal::vmx::GetPWMManager().Free(pwmPortHandle);
}

HAL_Bool HAL_CheckPWMChannel(int32_t channel) {
  return hal::vmx::IsPWMChannelValid(channel);
}

void HAL_SetPWMConfigMicroseconds(HAL_DigitalHandle pwmPortHandle,
                                  int32_t maxPwm, int32_t deadbandMaxPwm,
                                  int32_t centerPwm, int32_t deadbandMinPwm,
                                  int32_t minPwm, int32_t* status) {
  hal::vmx::SetPWMResult(hal::vmx::GetPWMManager().SetConfig(
                             pwmPortHandle, {maxPwm, deadbandMaxPwm, centerPwm,
                                             deadbandMinPwm, minPwm}),
                         status, "Failed to configure VMX PWM resource");
}

void HAL_GetPWMConfigMicroseconds(HAL_DigitalHandle pwmPortHandle,
                                  int32_t* maxPwm, int32_t* deadbandMaxPwm,
                                  int32_t* centerPwm, int32_t* deadbandMinPwm,
                                  int32_t* minPwm, int32_t* status) {
  auto [result, config] = hal::vmx::GetPWMManager().GetConfig(pwmPortHandle);
  if (result != hal::vmx::PWMResult::kOk) {
    hal::vmx::SetPWMResult(result, status,
                           "Failed to read VMX PWM configuration");
    return;
  }
  *maxPwm = config.maxPwm;
  *deadbandMaxPwm = config.deadbandMaxPwm;
  *centerPwm = config.centerPwm;
  *deadbandMinPwm = config.deadbandMinPwm;
  *minPwm = config.minPwm;
  *status = HAL_SUCCESS;
}

void HAL_SetPWMEliminateDeadband(HAL_DigitalHandle pwmPortHandle,
                                 HAL_Bool eliminateDeadband, int32_t* status) {
  hal::vmx::SetPWMResult(hal::vmx::GetPWMManager().SetEliminateDeadband(
                             pwmPortHandle, eliminateDeadband != 0),
                         status, "Failed to configure VMX PWM deadband");
}

HAL_Bool HAL_GetPWMEliminateDeadband(HAL_DigitalHandle pwmPortHandle,
                                     int32_t* status) {
  auto [result, eliminate] =
      hal::vmx::GetPWMManager().GetEliminateDeadband(pwmPortHandle);
  hal::vmx::SetPWMResult(result, status,
                         "Failed to read VMX PWM deadband configuration");
  return result == hal::vmx::PWMResult::kOk && eliminate;
}

void HAL_SetPWMPulseTimeMicroseconds(HAL_DigitalHandle pwmPortHandle,
                                     int32_t microsecondPulseTime,
                                     int32_t* status) {
  hal::vmx::SetPWMResult(hal::vmx::GetPWMManager().SetPulseTimeMicroseconds(
                             pwmPortHandle, microsecondPulseTime),
                         status, "Failed to write VMX PWM resource");
}

void HAL_SetPWMSpeed(HAL_DigitalHandle pwmPortHandle, double speed,
                     int32_t* status) {
  hal::vmx::SetPWMResult(
      hal::vmx::GetPWMManager().SetSpeed(pwmPortHandle, speed), status,
      "Failed to write VMX PWM speed");
}

void HAL_SetPWMPosition(HAL_DigitalHandle pwmPortHandle, double position,
                        int32_t* status) {
  hal::vmx::SetPWMResult(
      hal::vmx::GetPWMManager().SetPosition(pwmPortHandle, position), status,
      "Failed to write VMX PWM position");
}

void HAL_SetPWMDisabled(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
  hal::vmx::SetPWMResult(hal::vmx::GetPWMManager().Disable(pwmPortHandle),
                         status, "Failed to disable VMX PWM output");
}

int32_t HAL_GetPWMPulseTimeMicroseconds(HAL_DigitalHandle pwmPortHandle,
                                        int32_t* status) {
  auto [result, pulse] =
      hal::vmx::GetPWMManager().GetPulseTimeMicroseconds(pwmPortHandle);
  hal::vmx::SetPWMResult(result, status, "Failed to read VMX PWM state");
  return result == hal::vmx::PWMResult::kOk ? pulse : 0;
}

double HAL_GetPWMSpeed(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
  auto [result, speed] = hal::vmx::GetPWMManager().GetSpeed(pwmPortHandle);
  hal::vmx::SetPWMResult(result, status, "Failed to read VMX PWM speed");
  return result == hal::vmx::PWMResult::kOk ? speed : 0.0;
}

double HAL_GetPWMPosition(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
  auto [result, position] =
      hal::vmx::GetPWMManager().GetPosition(pwmPortHandle);
  hal::vmx::SetPWMResult(result, status, "Failed to read VMX PWM position");
  return result == hal::vmx::PWMResult::kOk ? position : 0.0;
}

void HAL_LatchPWMZero(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
  if (hal::vmx::CheckPWMHandleForUnsupported(pwmPortHandle, status)) {
    hal::vmx::SetPWMUnsupported(status, "PWM zero latch");
  }
}

void HAL_SetPWMPeriodScale(HAL_DigitalHandle pwmPortHandle, int32_t squelchMask,
                           int32_t* status) {
  static_cast<void>(squelchMask);
  if (hal::vmx::CheckPWMHandleForUnsupported(pwmPortHandle, status)) {
    hal::vmx::SetPWMUnsupported(status, "PWM period scaling");
  }
}

void HAL_SetPWMAlwaysHighMode(HAL_DigitalHandle pwmPortHandle,
                              int32_t* status) {
  if (hal::vmx::CheckPWMHandleForUnsupported(pwmPortHandle, status)) {
    hal::vmx::SetPWMUnsupported(status, "PWM always-high mode");
  }
}

int32_t HAL_GetPWMLoopTiming(int32_t* status) {
  hal::vmx::SetPWMUnsupported(status, "PWM loop timing");
  return 0;
}

uint64_t HAL_GetPWMCycleStartTime(int32_t* status) {
  hal::vmx::SetPWMUnsupported(status, "PWM cycle start time");
  return 0;
}

}  // extern "C"
