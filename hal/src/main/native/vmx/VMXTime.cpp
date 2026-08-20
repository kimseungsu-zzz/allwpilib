// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VMXTimeInternal.h"

#include <memory>

#include "HALInitializer.h"
#include "HALInternal.h"
#include "VMXPi.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"
#include "hal/HALBase.h"

namespace hal::vmx {
namespace {

bool ReadHardwareTime(uint64_t& timestampMicroseconds) noexcept {
  auto context = GetRuntimeContext();
  if (!context) {
    timestampMicroseconds = 0;
    return false;
  }
  try {
    timestampMicroseconds =
        context->getTime().GetCurrentTotalMicroseconds();
    return true;
  } catch (...) {
    timestampMicroseconds = 0;
    return false;
  }
}

}  // namespace

uint64_t GetTimeMicroseconds(int32_t* status) {
  uint64_t timestamp = 0;
  if (!ReadHardwareTime(timestamp)) {
    if (status) {
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, "VMX hardware time is unavailable");
    }
    return 0;
  }
  if (status) {
    *status = HAL_SUCCESS;
  }
  return timestamp;
}

}  // namespace hal::vmx

extern "C" {

uint64_t HAL_GetFPGATime(int32_t* status) {
  hal::init::CheckInit();
  return hal::vmx::GetTimeMicroseconds(status);
}

uint64_t HAL_ExpandFPGATime(uint32_t unexpandedLower, int32_t* status) {
  auto current = HAL_GetFPGATime(status);
  if (status && *status != HAL_SUCCESS) {
    return 0;
  }

  uint64_t upper = current & 0xffffffff00000000ULL;
  uint32_t currentLower = static_cast<uint32_t>(current);
  if (unexpandedLower > currentLower) {
    if (upper >= (uint64_t{1} << 32)) {
      upper -= uint64_t{1} << 32;
    } else {
      upper = 0;
    }
  }
  if (status) {
    *status = HAL_SUCCESS;
  }
  return upper | unexpandedLower;
}

}  // extern "C"
