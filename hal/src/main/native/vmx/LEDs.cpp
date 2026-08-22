// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/LEDs.h"

#include <string>
#include <string_view>

#include "HALInternal.h"
#include "hal/Errors.h"
#include "hal/Types.h"

// The roboRIO radio/RSL LED surface has no VMX equivalent. These entry points
// exist because the HAL C ABI is what wpilibc and the JNI layer link against,
// so every declared symbol has to resolve even when the hardware behind it
// does not. They report the absence rather than succeeding quietly: a robot
// that believes it configured something it did not is worse than one that
// fails at the call.

namespace {

void SetUnsupported(int32_t* status, std::string_view feature) {
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(
      status, std::string{"VMX HAL does not support "} + std::string{feature});
}

}  // namespace

extern "C" {

void HAL_SetRadioLEDState(HAL_RadioLEDState, int32_t* status) {
  SetUnsupported(status, "the roboRIO radio LED");
}

HAL_RadioLEDState HAL_GetRadioLEDState(int32_t* status) {
  SetUnsupported(status, "the roboRIO radio LED");
  return HAL_RadioLED_kOff;
}

}  // extern "C"
