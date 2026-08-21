// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VMXAccelerometerInternal.h"

#include <memory>

#include "VMXPi.h"
#include "VMXRuntime.h"

namespace hal::vmx {
namespace {

double ReadVMXAxis(int axis) {
  auto context = GetRuntimeContext();
  if (!context || !context->IsOpen()) {
    return 0.0;
  }
  auto& ahrs = context->getAHRS();
  switch (axis) {
    case 0:
      return static_cast<double>(ahrs.GetRawAccelX());
    case 1:
      return static_cast<double>(ahrs.GetRawAccelY());
    case 2:
      return static_cast<double>(ahrs.GetRawAccelZ());
    default:
      return 0.0;
  }
}

}  // namespace

BuiltInAccelerometerState& GetBuiltInAccelerometerState() {
  static BuiltInAccelerometerState state{ReadVMXAxis};
  return state;
}

}  // namespace hal::vmx

extern "C" {

void HAL_SetAccelerometerActive(HAL_Bool active) {
  hal::vmx::GetBuiltInAccelerometerState().SetActive(active);
}

void HAL_SetAccelerometerRange(HAL_AccelerometerRange range) {
  hal::vmx::GetBuiltInAccelerometerState().SetRange(range);
}

double HAL_GetAccelerometerX(void) {
  return hal::vmx::GetBuiltInAccelerometerState().GetAxis(0);
}

double HAL_GetAccelerometerY(void) {
  return hal::vmx::GetBuiltInAccelerometerState().GetAxis(1);
}

double HAL_GetAccelerometerZ(void) {
  return hal::vmx::GetBuiltInAccelerometerState().GetAxis(2);
}

}  // extern "C"
