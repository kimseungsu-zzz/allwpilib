// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>

namespace hal::init {
extern std::atomic_bool HAL_IsInitialized;
void RunInitialize();

inline void CheckInit() {
  if (!HAL_IsInitialized.load(std::memory_order_acquire)) {
    RunInitialize();
  }
}
}  // namespace hal::init
