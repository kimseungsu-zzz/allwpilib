// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/AnalogAccumulator.h"

#include <string_view>

#include "AnalogInputInternal.h"
#include "HALInternal.h"
#include "hal/Errors.h"

namespace hal::vmx {
namespace {

void SetAccumulatorResult(AnalogInputResult result, int32_t* status,
                          std::string_view hardwareMessage) {
  switch (result) {
    case AnalogInputResult::kOk:
      *status = HAL_SUCCESS;
      break;
    case AnalogInputResult::kInvalidHandle:
      *status = HAL_HANDLE_ERROR;
      break;
    case AnalogInputResult::kInvalidAccumulatorChannel:
      *status = HAL_INVALID_ACCUMULATOR_CHANNEL;
      break;
    case AnalogInputResult::kAccumulatorNotInitialized:
      *status = NULL_PARAMETER;
      hal::SetLastError(status, "VMX analog accumulator is not initialized");
      break;
    case AnalogInputResult::kOutOfRange:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(
          status,
          "VMX accumulator center must fit int16 and deadband must be in "
          "the range [0, 32767]");
      break;
    case AnalogInputResult::kRollbackFailure:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(
          status,
          "VMX accumulator reconfiguration and rollback both failed; "
          "resource is faulted");
      break;
    default:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, hardwareMessage);
      break;
  }
}

}  // namespace
}  // namespace hal::vmx

extern "C" {

HAL_Bool HAL_IsAccumulatorChannel(HAL_AnalogInputHandle analogPortHandle,
                                  int32_t* status) {
  auto [result, supported] =
      hal::vmx::GetAnalogInputManager().IsAccumulatorChannel(analogPortHandle);
  hal::vmx::SetAccumulatorResult(
      result, status, "Failed to inspect VMX analog accumulator channel");
  return result == hal::vmx::AnalogInputResult::kOk && supported;
}

void HAL_InitAccumulator(HAL_AnalogInputHandle analogPortHandle,
                         int32_t* status) {
  hal::vmx::SetAccumulatorResult(
      hal::vmx::GetAnalogInputManager().InitAccumulator(analogPortHandle),
      status, "Failed to enable the VMX analog accumulation counter");
}

void HAL_ResetAccumulator(HAL_AnalogInputHandle analogPortHandle,
                          int32_t* status) {
  hal::vmx::SetAccumulatorResult(
      hal::vmx::GetAnalogInputManager().ResetAccumulator(analogPortHandle),
      status, "Failed to reset the VMX analog accumulation counter");
}

void HAL_SetAccumulatorCenter(HAL_AnalogInputHandle analogPortHandle,
                              int32_t center, int32_t* status) {
  hal::vmx::SetAccumulatorResult(
      hal::vmx::GetAnalogInputManager().SetAccumulatorCenter(analogPortHandle,
                                                             center),
      status, "Failed to set the VMX analog accumulator center");
}

void HAL_SetAccumulatorDeadband(HAL_AnalogInputHandle analogPortHandle,
                                int32_t deadband, int32_t* status) {
  hal::vmx::SetAccumulatorResult(
      hal::vmx::GetAnalogInputManager().SetAccumulatorDeadband(analogPortHandle,
                                                               deadband),
      status, "Failed to set the VMX analog accumulator deadband");
}

int64_t HAL_GetAccumulatorValue(HAL_AnalogInputHandle analogPortHandle,
                                int32_t* status) {
  auto [result, output] =
      hal::vmx::GetAnalogInputManager().GetAccumulatorOutput(analogPortHandle);
  hal::vmx::SetAccumulatorResult(
      result, status, "Failed to read the VMX analog accumulator output");
  return result == hal::vmx::AnalogInputResult::kOk ? output.value : 0;
}

int64_t HAL_GetAccumulatorCount(HAL_AnalogInputHandle analogPortHandle,
                                int32_t* status) {
  auto [result, output] =
      hal::vmx::GetAnalogInputManager().GetAccumulatorOutput(analogPortHandle);
  hal::vmx::SetAccumulatorResult(
      result, status, "Failed to read the VMX analog accumulator output");
  return result == hal::vmx::AnalogInputResult::kOk ? output.count : 0;
}

void HAL_GetAccumulatorOutput(HAL_AnalogInputHandle analogPortHandle,
                              int64_t* value, int64_t* count, int32_t* status) {
  if (value == nullptr || count == nullptr) {
    *status = NULL_PARAMETER;
    return;
  }
  auto [result, output] =
      hal::vmx::GetAnalogInputManager().GetAccumulatorOutput(analogPortHandle);
  hal::vmx::SetAccumulatorResult(
      result, status, "Failed to read the VMX analog accumulator output");
  if (result != hal::vmx::AnalogInputResult::kOk) {
    *value = 0;
    *count = 0;
    return;
  }
  *value = output.value;
  *count = output.count;
}

}  // extern "C"
