// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "hal/AnalogTrigger.h"
#include "hal/Types.h"
#include "hal/handles/HandlesInternal.h"

namespace hal::vmx {

enum class VMXDigitalSourceResult {
  kOk,
  kInvalid,
  kUnsupportedAnalogTrigger,
};

/**
 * Validates the source handle shape shared by Encoder, Counter, and future
 * interrupt/trigger adapters. Ownership and channel lookup remain in the DIO
 * manager so callers cannot accidentally treat a handle as a raw channel.
 */
inline VMXDigitalSourceResult DecodeVMXDigitalSource(
    HAL_Handle sourceHandle, HAL_AnalogTriggerType analogTriggerType) noexcept {
  static_cast<void>(analogTriggerType);
  if (hal::isHandleType(sourceHandle, HAL_HandleEnum::AnalogTrigger)) {
    return VMXDigitalSourceResult::kUnsupportedAnalogTrigger;
  }
  return hal::isHandleType(sourceHandle, HAL_HandleEnum::DIO)
             ? VMXDigitalSourceResult::kOk
             : VMXDigitalSourceResult::kInvalid;
}

}  // namespace hal::vmx
