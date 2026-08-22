// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/DMA.h"

#include <string>
#include <string_view>

#include "HALInternal.h"
#include "hal/Errors.h"
#include "hal/Types.h"

// FPGA DMA has no VMX SDK equivalent. These entry points exist because the HAL
// C ABI is what wpilibc and the JNI layer link against, so every declared
// symbol has to resolve even when the hardware behind it does not. They report
// the absence rather than succeeding quietly: a robot that believes it
// configured something it did not is worse than one that fails at the call.

namespace {

void SetUnsupported(int32_t* status, std::string_view feature) {
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(
      status, std::string{"VMX HAL does not support "} + std::string{feature});
}

}  // namespace

extern "C" {

HAL_DMAHandle HAL_InitializeDMA(int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return HAL_kInvalidHandle;
}

void HAL_FreeDMA(HAL_DMAHandle) {
}

void HAL_SetDMAPause(HAL_DMAHandle, HAL_Bool, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_SetDMATimedTrigger(HAL_DMAHandle, double, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_SetDMATimedTriggerCycles(HAL_DMAHandle, uint32_t, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_AddDMAEncoder(HAL_DMAHandle, HAL_EncoderHandle, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_AddDMAEncoderPeriod(HAL_DMAHandle, HAL_EncoderHandle,
                             int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_AddDMACounter(HAL_DMAHandle, HAL_CounterHandle, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_AddDMACounterPeriod(HAL_DMAHandle, HAL_CounterHandle,
                             int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_AddDMADigitalSource(HAL_DMAHandle, HAL_Handle, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_AddDMAAnalogInput(HAL_DMAHandle, HAL_AnalogInputHandle,
                           int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_AddDMAAveragedAnalogInput(HAL_DMAHandle, HAL_AnalogInputHandle,
                                   int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_AddDMAAnalogAccumulator(HAL_DMAHandle, HAL_AnalogInputHandle,
                                 int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_AddDMADutyCycle(HAL_DMAHandle, HAL_DutyCycleHandle, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

int32_t HAL_SetDMAExternalTrigger(HAL_DMAHandle, HAL_Handle,
                                  HAL_AnalogTriggerType, HAL_Bool, HAL_Bool,
                                  int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return 0;
}

void HAL_ClearDMASensors(HAL_DMAHandle, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_ClearDMAExternalTriggers(HAL_DMAHandle, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_StartDMA(HAL_DMAHandle, int32_t, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void HAL_StopDMA(HAL_DMAHandle, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

void* HAL_GetDMADirectPointer(HAL_DMAHandle) {
  return nullptr;
}

enum HAL_DMAReadStatus HAL_ReadDMADirect(void*, HAL_DMASample*, double,
                                         int32_t*, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return HAL_DMA_ERROR;
}

enum HAL_DMAReadStatus HAL_ReadDMA(HAL_DMAHandle, HAL_DMASample*, double,
                                   int32_t*, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return HAL_DMA_ERROR;
}

uint64_t HAL_GetDMASampleTime(const HAL_DMASample*, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return 0;
}

int32_t HAL_GetDMASampleEncoderRaw(const HAL_DMASample*, HAL_EncoderHandle,
                                   int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return 0;
}

int32_t HAL_GetDMASampleCounter(const HAL_DMASample*, HAL_CounterHandle,
                                int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return 0;
}

int32_t HAL_GetDMASampleEncoderPeriodRaw(const HAL_DMASample*,
                                         HAL_EncoderHandle, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return 0;
}

int32_t HAL_GetDMASampleCounterPeriod(const HAL_DMASample*, HAL_CounterHandle,
                                      int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return 0;
}

HAL_Bool HAL_GetDMASampleDigitalSource(const HAL_DMASample*, HAL_Handle,
                                       int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return 0;
}

int32_t HAL_GetDMASampleAnalogInputRaw(const HAL_DMASample*,
                                       HAL_AnalogInputHandle, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return 0;
}

int32_t HAL_GetDMASampleAveragedAnalogInputRaw(const HAL_DMASample*,
                                               HAL_AnalogInputHandle,
                                               int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return 0;
}

void HAL_GetDMASampleAnalogAccumulator(const HAL_DMASample*,
                                       HAL_AnalogInputHandle, int64_t*,
                                       int64_t*, int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
}

int32_t HAL_GetDMASampleDutyCycleOutputRaw(const HAL_DMASample*,
                                           HAL_DutyCycleHandle,
                                           int32_t* status) {
  SetUnsupported(status, "FPGA DMA");
  return 0;
}

}  // extern "C"
