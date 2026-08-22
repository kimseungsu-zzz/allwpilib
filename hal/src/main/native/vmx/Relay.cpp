// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/Relay.h"

#include <string>
#include <string_view>

#include "HALInternal.h"
#include "hal/Errors.h"
#include "hal/Types.h"

// VMX has no relay hardware. These entry points exist because the HAL C ABI is
// what wpilibc and the JNI layer link against, so every declared symbol has to
// resolve even when the hardware behind it does not. They report the absence
// rather than succeeding quietly: a robot that believes it configured
// something it did not is worse than one that fails at the call.

namespace {

void SetUnsupported(int32_t* status, std::string_view feature) {
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(
      status, std::string{"VMX HAL does not support "} + std::string{feature});
}

}  // namespace

extern "C" {

HAL_RelayHandle HAL_InitializeRelayPort(HAL_PortHandle, HAL_Bool, const char*,
                                        int32_t* status) {
  SetUnsupported(status, "relay outputs");
  return HAL_kInvalidHandle;
}

void HAL_FreeRelayPort(HAL_RelayHandle) {
}

HAL_Bool HAL_CheckRelayChannel(int32_t) {
  return 0;
}

void HAL_SetRelay(HAL_RelayHandle, HAL_Bool, int32_t* status) {
  SetUnsupported(status, "relay outputs");
}

HAL_Bool HAL_GetRelay(HAL_RelayHandle, int32_t* status) {
  SetUnsupported(status, "relay outputs");
  return 0;
}

}  // extern "C"
