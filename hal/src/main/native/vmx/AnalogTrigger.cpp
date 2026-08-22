// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/AnalogTrigger.h"

#include <string_view>

#include "AnalogInputInternal.h"
#include "AnalogTriggerInternal.h"
#include "HALInitializer.h"
#include "HALInternal.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"

namespace hal::vmx {
namespace {

void SetAnalogTriggerResult(AnalogTriggerResult result, int32_t* status,
                            std::string_view hardwareMessage) {
  switch (result) {
    case AnalogTriggerResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case AnalogTriggerResult::kInvalidHandle:
      *status = HAL_HANDLE_ERROR;
      return;
    case AnalogTriggerResult::kNoResources:
      *status = NO_AVAILABLE_RESOURCES;
      return;
    case AnalogTriggerResult::kLimitOrder:
      *status = ANALOG_TRIGGER_LIMIT_ORDER_ERROR;
      return;
    case AnalogTriggerResult::kOutOfRange:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status, "VMX AnalogTrigger threshold is out of range");
      return;
    case AnalogTriggerResult::kUnsupported:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, hardwareMessage);
      return;
    case AnalogTriggerResult::kPulseOutput:
      *status = ANALOG_TRIGGER_PULSE_OUTPUT_ERROR;
      return;
    case AnalogTriggerResult::kHardwareFailure:
    default:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, hardwareMessage);
      return;
  }
}

}  // namespace

AnalogTriggerManager& GetAnalogTriggerManager() {
  static AnalogTriggerManager manager{
      [](HAL_AnalogInputHandle handle) {
        return GetAnalogInputManager().AcquirePort(handle);
      }};
  return manager;
}

}  // namespace hal::vmx

extern "C" {

HAL_AnalogTriggerHandle HAL_InitializeAnalogTrigger(
    HAL_AnalogInputHandle portHandle, int32_t* status) {
  hal::init::CheckInit();
  if (!hal::vmx::IsRuntimeInitialized()) {
    *status = INCOMPATIBLE_STATE;
    hal::SetLastError(status, "VMX HAL runtime is not initialized");
    return HAL_kInvalidHandle;
  }

  auto allocation = hal::vmx::GetAnalogTriggerManager().Allocate(portHandle);
  hal::vmx::SetAnalogTriggerResult(
      allocation.result, status,
      "Failed to retain the VMX AnalogInput for AnalogTrigger");
  return allocation.result == hal::vmx::AnalogTriggerResult::kOk
             ? allocation.handle
             : HAL_kInvalidHandle;
}

HAL_AnalogTriggerHandle HAL_InitializeAnalogTriggerDutyCycle(
    HAL_DutyCycleHandle dutyCycleHandle, int32_t* status) {
  hal::init::CheckInit();
  static_cast<void>(dutyCycleHandle);
  if (!hal::isHandleType(dutyCycleHandle, hal::HAL_HandleEnum::DutyCycle)) {
    *status = HAL_HANDLE_ERROR;
    return HAL_kInvalidHandle;
  }
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(status,
                    "VMX AnalogTrigger DutyCycle input is not supported");
  return HAL_kInvalidHandle;
}

void HAL_CleanAnalogTrigger(HAL_AnalogTriggerHandle analogTriggerHandle) {
  hal::vmx::GetAnalogTriggerManager().Free(analogTriggerHandle);
}

void HAL_SetAnalogTriggerLimitsRaw(HAL_AnalogTriggerHandle analogTriggerHandle,
                                   int32_t lower, int32_t upper,
                                   int32_t* status) {
  hal::vmx::SetAnalogTriggerResult(
      hal::vmx::GetAnalogTriggerManager().SetLimitsRaw(analogTriggerHandle,
                                                       lower, upper),
      status, "Failed to set VMX AnalogTrigger raw limits");
}

void HAL_SetAnalogTriggerLimitsVoltage(
    HAL_AnalogTriggerHandle analogTriggerHandle, double lower, double upper,
    int32_t* status) {
  hal::vmx::SetAnalogTriggerResult(
      hal::vmx::GetAnalogTriggerManager().SetLimitsVoltage(analogTriggerHandle,
                                                           lower, upper),
      status, "Failed to set VMX AnalogTrigger voltage limits");
}

void HAL_SetAnalogTriggerLimitsDutyCycle(
    HAL_AnalogTriggerHandle analogTriggerHandle, double lower, double upper,
    int32_t* status) {
  static_cast<void>(lower);
  static_cast<void>(upper);
  if (!hal::vmx::GetAnalogTriggerManager().IsValid(analogTriggerHandle)) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  hal::vmx::SetAnalogTriggerResult(
      hal::vmx::AnalogTriggerResult::kUnsupported, status,
      "VMX AnalogTrigger DutyCycle limits are not supported");
}

void HAL_SetAnalogTriggerAveraged(HAL_AnalogTriggerHandle analogTriggerHandle,
                                  HAL_Bool useAveragedValue,
                                  int32_t* status) {
  hal::vmx::SetAnalogTriggerResult(
      hal::vmx::GetAnalogTriggerManager().SetAveraged(
          analogTriggerHandle, useAveragedValue != 0),
      status, "Failed to select VMX AnalogTrigger averaged input");
}

void HAL_SetAnalogTriggerFiltered(HAL_AnalogTriggerHandle analogTriggerHandle,
                                  HAL_Bool useFilteredValue,
                                  int32_t* status) {
  hal::vmx::SetAnalogTriggerResult(
      hal::vmx::GetAnalogTriggerManager().SetFiltered(
          analogTriggerHandle, useFilteredValue != 0),
      status, "VMX AnalogTrigger filtered mode is not supported");
}

HAL_Bool HAL_GetAnalogTriggerInWindow(
    HAL_AnalogTriggerHandle analogTriggerHandle, int32_t* status) {
  auto result =
      hal::vmx::GetAnalogTriggerManager().GetInWindow(analogTriggerHandle);
  hal::vmx::SetAnalogTriggerResult(
      result.first, status, "Failed to read VMX AnalogTrigger InWindow state");
  return result.first == hal::vmx::AnalogTriggerResult::kOk && result.second;
}

HAL_Bool HAL_GetAnalogTriggerTriggerState(
    HAL_AnalogTriggerHandle analogTriggerHandle, int32_t* status) {
  auto result =
      hal::vmx::GetAnalogTriggerManager().GetTriggerState(analogTriggerHandle);
  hal::vmx::SetAnalogTriggerResult(
      result.first, status, "Failed to read VMX AnalogTrigger state");
  return result.first == hal::vmx::AnalogTriggerResult::kOk && result.second;
}

HAL_Bool HAL_GetAnalogTriggerOutput(HAL_AnalogTriggerHandle analogTriggerHandle,
                                    HAL_AnalogTriggerType type,
                                    int32_t* status) {
  auto result =
      hal::vmx::GetAnalogTriggerManager().GetOutput(analogTriggerHandle, type);
  hal::vmx::SetAnalogTriggerResult(
      result.first, status, "Failed to read VMX AnalogTrigger output");
  return result.first == hal::vmx::AnalogTriggerResult::kOk && result.second;
}

int32_t HAL_GetAnalogTriggerFPGAIndex(
    HAL_AnalogTriggerHandle analogTriggerHandle, int32_t* status) {
  auto& manager = hal::vmx::GetAnalogTriggerManager();
  if (!manager.IsValid(analogTriggerHandle)) {
    *status = HAL_HANDLE_ERROR;
    return -1;
  }
  *status = HAL_SUCCESS;
  return manager.GetIndex(analogTriggerHandle);
}

}  // extern "C"
