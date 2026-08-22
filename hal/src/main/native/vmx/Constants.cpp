// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of the WPILib
// BSD license file in the root directory of this project.

#include "hal/Constants.h"

#include "VMXConstants.h"

extern "C" {

int32_t HAL_GetSystemClockTicksPerMicrosecond(void) {
  // VMXTime is already expressed in monotonic microseconds.
  return hal::vmx::kSystemClockTicksPerMicrosecond;
}

}  // extern "C"
