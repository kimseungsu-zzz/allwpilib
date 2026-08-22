// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of the WPILib
// BSD license file in the root directory of this project.

#include "hal/Ports.h"

#include "VMXConstants.h"

extern "C" {

int32_t HAL_GetNumAccumulators(void) { return hal::vmx::kNumAccumulators; }
int32_t HAL_GetNumAnalogTriggers(void) {
  return hal::vmx::kNumAnalogTriggers;
}
int32_t HAL_GetNumAnalogInputs(void) { return hal::vmx::kNumAnalogInputs; }
int32_t HAL_GetNumAnalogOutputs(void) { return hal::vmx::kNumAnalogOutputs; }
int32_t HAL_GetNumCounters(void) { return hal::vmx::kNumCounters; }
int32_t HAL_GetNumDigitalHeaders(void) {
  return hal::vmx::kNumDigitalHeaders;
}
int32_t HAL_GetNumPWMHeaders(void) { return hal::vmx::kNumPWMHeaders; }
int32_t HAL_GetNumDigitalChannels(void) {
  return hal::vmx::kNumDigitalChannels;
}
int32_t HAL_GetNumPWMChannels(void) { return hal::vmx::kNumPWMChannels; }
int32_t HAL_GetNumDigitalPWMOutputs(void) {
  return hal::vmx::kNumDigitalPWMOutputs;
}
int32_t HAL_GetNumEncoders(void) { return hal::vmx::kNumEncoders; }
int32_t HAL_GetNumInterrupts(void) { return hal::vmx::kNumInterrupts; }
int32_t HAL_GetNumRelayChannels(void) {
  return hal::vmx::kNumRelayChannels;
}
int32_t HAL_GetNumRelayHeaders(void) { return hal::vmx::kNumRelayHeaders; }
int32_t HAL_GetNumCTREPCMModules(void) {
  return hal::vmx::kNumCTREPCMModules;
}
int32_t HAL_GetNumCTRESolenoidChannels(void) {
  return hal::vmx::kNumCTRESolenoidChannels;
}
int32_t HAL_GetNumCTREPDPModules(void) {
  return hal::vmx::kNumCTREPDPModules;
}
int32_t HAL_GetNumCTREPDPChannels(void) {
  return hal::vmx::kNumCTREPDPChannels;
}
int32_t HAL_GetNumREVPDHModules(void) {
  return hal::vmx::kNumREVPDHModules;
}
int32_t HAL_GetNumREVPDHChannels(void) {
  return hal::vmx::kNumREVPDHChannels;
}
int32_t HAL_GetNumREVPHModules(void) { return hal::vmx::kNumREVPHModules; }
int32_t HAL_GetNumREVPHChannels(void) { return hal::vmx::kNumREVPHChannels; }
int32_t HAL_GetNumDutyCycles(void) { return hal::vmx::kNumDutyCycles; }
int32_t HAL_GetNumAddressableLEDs(void) {
  return hal::vmx::kNumAddressableLEDs;
}

}  // extern "C"
