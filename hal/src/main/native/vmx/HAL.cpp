// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/HAL.h"

#include <cstdint>
#include <cstdio>
#include <mutex>

#include "HALInitializer.h"
#include "AddressableLEDInternal.h"
#include "DriverStationInternal.h"
#include "NotifierInternal.h"
#include "SPIAutoInternal.h"
#include "VMXRuntime.h"
#include "VMXRTCInternal.h"
#include "VMXWatchdogInternal.h"
#include "VMXCANInternal.h"
#include "hal/handles/HandlesInternal.h"

namespace {
std::mutex gHalLifecycleMutex;
}

extern "C" {

HAL_PortHandle HAL_GetPort(int32_t channel) {
  if (channel < 0 || channel >= 255) {
    return HAL_kInvalidHandle;
  }
  return hal::createPortHandle(channel, 1);
}

HAL_PortHandle HAL_GetPortWithModule(int32_t module, int32_t channel) {
  if (channel < 0 || channel >= 255 || module < 0 || module >= 255) {
    return HAL_kInvalidHandle;
  }
  return hal::createPortHandle(channel, module);
}

HAL_RuntimeType HAL_GetRuntimeType(void) {
  // There is no public VMX runtime enum. Report a hardware runtime rather than
  // simulation so upper layers do not silently select simulation behavior.
  return HAL_Runtime_RoboRIO;
}

HAL_Bool HAL_Initialize(int32_t timeout, int32_t mode) {
  static_cast<void>(timeout);
  static_cast<void>(mode);

  std::scoped_lock lock{gHalLifecycleMutex};
  try {
    if (!hal::vmx::InitializeRuntime()) {
      return false;
    }
    // RTC is a wall-clock bootstrap only.  FPGA time, notifier deadlines, and
    // interrupt timestamps continue to use VMXTime's monotonic source.
    static_cast<void>(hal::vmx::BootstrapLinuxSystemClockFromVMXRTC());
    hal::init::HAL_IsInitialized.store(true, std::memory_order_release);
    if (!hal::vmx::InitializeCAN()) {
      hal::init::HAL_IsInitialized.store(false, std::memory_order_release);
      hal::vmx::ShutdownRuntime();
      return false;
    }
    hal::vmx::InitializeDriverStation();
    if (!hal::vmx::InitializeHardwareWatchdog()) {
      hal::vmx::ShutdownDriverStation();
      hal::vmx::ShutdownCAN();
      hal::init::HAL_IsInitialized.store(false, std::memory_order_release);
      hal::vmx::ShutdownRuntime();
      return false;
    }
    return true;
  } catch (...) {
    // No C++ exception may cross the public C HAL ABI used by JNI/RobotPy.
    std::fputs("VMX HAL initialization failed: unexpected exception\n", stderr);
    hal::init::HAL_IsInitialized.store(false, std::memory_order_release);
    hal::vmx::ShutdownHardwareWatchdog();
    hal::vmx::ShutdownDriverStation();
    hal::vmx::ShutdownCAN();
    hal::vmx::ShutdownRuntime();
    return false;
  }
}

void HAL_Shutdown(void) {
  std::scoped_lock lock{gHalLifecycleMutex};
  hal::init::HAL_IsInitialized.store(false, std::memory_order_release);
  hal::vmx::ShutdownHardwareWatchdog();
  hal::vmx::GetAddressableLEDManager().Shutdown();
  hal::vmx::ShutdownDriverStation();
  hal::vmx::ShutdownCAN();
  hal::vmx::ShutdownSPIAuto();
  hal::vmx::ShutdownNotifiers();
  hal::vmx::ShutdownRuntime();
}

void HAL_SimPeriodicBefore(void) {}

void HAL_SimPeriodicAfter(void) {}

int64_t HAL_Report(int32_t resource, int32_t instanceNumber, int32_t context,
                   const char* feature) {
  static_cast<void>(resource);
  static_cast<void>(instanceNumber);
  static_cast<void>(context);
  static_cast<void>(feature);
  return 0;
}

}  // extern "C"
