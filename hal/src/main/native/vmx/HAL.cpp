// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/HAL.h"

#include <cstdint>
#include <cstdio>
#include <mutex>

#include "hal/Errors.h"
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
#include "studica/Titan.h"
#include "studica/Cobra.h"
#include "studica/LightTower.h"
#include "studica/Parsec.h"
#include "studica/Colore.h"

namespace {
std::mutex gHalLifecycleMutex;
}

namespace hal {
// The shared CAN device implementations retain the roboRIO automatic-module
// initialization hook.  VMX has no Driver Station startup epoch, so automatic
// discovery starts immediately after the VMX runtime is available.
uint64_t GetDSInitializeTime() { return 0; }
}  // namespace hal

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
    hal::init::InitializeCTREPCM();
    hal::init::InitializeREVPH();
    hal::init::InitializeCTREPDP();
    hal::init::InitializeREVPDH();
    hal::vmx::InitializeDriverStation();
    if (!hal::vmx::InitializeHardwareWatchdog()) {
      hal::vmx::ShutdownDriverStation();
      hal::vmx::ShutdownCAN();
      StudicaTitan_ShutdownAll();
      StudicaCobra_ShutdownAll();
      StudicaLightTower_ShutdownAll();
      StudicaParsec_ShutdownAll();
      StudicaColore_ShutdownAll();
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
    StudicaTitan_ShutdownAll();
    StudicaCobra_ShutdownAll();
    StudicaLightTower_ShutdownAll();
    StudicaParsec_ShutdownAll();
    StudicaColore_ShutdownAll();
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
  // Vendor workers must stop and command zero before the shared VMXPi context
  // is released by ShutdownRuntime().
  StudicaTitan_ShutdownAll();
  StudicaCobra_ShutdownAll();
  StudicaLightTower_ShutdownAll();
  StudicaParsec_ShutdownAll();
  StudicaColore_ShutdownAll();
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

// HALBase entry points that the athena and sim backends provide and this one
// did not. They are not optional: ErrorHandling.cpp calls HAL_GetErrorMessage
// from inside the HAL itself, and wpilibc and the JNI layer call the rest, so
// their absence is a link failure rather than a missing feature. The coverage
// manifest marks several UNSUPPORTED_HARDWARE, which describes the hardware
// honestly but does not excuse the symbol: the C ABI still has to resolve.

// Maps every code/message pair that the public hal/Errors.h defines, rather
// than a hand-picked subset, so the mapping cannot drift as codes are added.
// The athena and sim backends additionally translate CTRE and NI codes from
// their own ErrorsInternal.h; VMX produces neither, and reaching into another
// backend's private header to borrow them would be worse than saying so.
const char* HAL_GetErrorMessage(int32_t code) {
  switch (code) {
    case 0:
      return "";
    case HAL_CAN_BUFFER_OVERRUN:
      return HAL_CAN_BUFFER_OVERRUN_MESSAGE;
    case HAL_CONSOLE_OUT_ENABLED_ERROR:
      return HAL_CONSOLE_OUT_ENABLED_ERROR_MESSAGE;
    case HAL_USE_LAST_ERROR:
      return HAL_USE_LAST_ERROR_MESSAGE;
    case HAL_SIM_NOT_SUPPORTED:
      return HAL_SIM_NOT_SUPPORTED_MESSAGE;
    case HAL_CAN_TIMEOUT:
      return HAL_CAN_TIMEOUT_MESSAGE;
    case HAL_THREAD_PRIORITY_RANGE_ERROR:
      return HAL_THREAD_PRIORITY_RANGE_ERROR_MESSAGE;
    case HAL_THREAD_PRIORITY_ERROR:
      return HAL_THREAD_PRIORITY_ERROR_MESSAGE;
    case HAL_SERIAL_PORT_ERROR:
      return HAL_SERIAL_PORT_ERROR_MESSAGE;
    case HAL_SERIAL_PORT_OPEN_ERROR:
      return HAL_SERIAL_PORT_OPEN_ERROR_MESSAGE;
    case HAL_SERIAL_PORT_NOT_FOUND:
      return HAL_SERIAL_PORT_NOT_FOUND_MESSAGE;
    case HAL_INVALID_DMA_STATE:
      return HAL_INVALID_DMA_STATE_MESSAGE;
    case HAL_INVALID_DMA_ADDITION:
      return HAL_INVALID_DMA_ADDITION_MESSAGE;
    case HAL_LED_CHANNEL_ERROR:
      return HAL_LED_CHANNEL_ERROR_MESSAGE;
    case HAL_HANDLE_ERROR:
      return HAL_HANDLE_ERROR_MESSAGE;
    case HAL_PWM_SCALE_ERROR:
      return HAL_PWM_SCALE_ERROR_MESSAGE;
    case HAL_COUNTER_NOT_SUPPORTED:
      return HAL_COUNTER_NOT_SUPPORTED_MESSAGE;
    case HAL_INVALID_ACCUMULATOR_CHANNEL:
      return HAL_INVALID_ACCUMULATOR_CHANNEL_MESSAGE;
    case RESOURCE_OUT_OF_RANGE:
      return RESOURCE_OUT_OF_RANGE_MESSAGE;
    case RESOURCE_IS_ALLOCATED:
      return RESOURCE_IS_ALLOCATED_MESSAGE;
    case PARAMETER_OUT_OF_RANGE:
      return PARAMETER_OUT_OF_RANGE_MESSAGE;
    case ANALOG_TRIGGER_PULSE_OUTPUT_ERROR:
      return ANALOG_TRIGGER_PULSE_OUTPUT_ERROR_MESSAGE;
    case ANALOG_TRIGGER_LIMIT_ORDER_ERROR:
      return ANALOG_TRIGGER_LIMIT_ORDER_ERROR_MESSAGE;
    case NULL_PARAMETER:
      return NULL_PARAMETER_MESSAGE;
    case NO_AVAILABLE_RESOURCES:
      return NO_AVAILABLE_RESOURCES_MESSAGE;
    case SAMPLE_RATE_TOO_HIGH:
      return SAMPLE_RATE_TOO_HIGH_MESSAGE;
    case VOLTAGE_OUT_OF_RANGE:
      return VOLTAGE_OUT_OF_RANGE_MESSAGE;
    case LOOP_TIMING_ERROR:
      return LOOP_TIMING_ERROR_MESSAGE;
    case SPI_WRITE_NO_MOSI:
      return SPI_WRITE_NO_MOSI_MESSAGE;
    case SPI_READ_NO_MISO:
      return SPI_READ_NO_MISO_MESSAGE;
    case SPI_READ_NO_DATA:
      return SPI_READ_NO_DATA_MESSAGE;
    case INCOMPATIBLE_STATE:
      return INCOMPATIBLE_STATE_MESSAGE;
    default:
      return "Unknown error status";
  }
}

// VMX has no FPGA. Report zero rather than inventing a version, and leave
// status successful so that startup paths which log this do not treat a
// truthful answer as a fault.
int32_t HAL_GetFPGAVersion(int32_t* status) {
  static_cast<void>(status);
  return 0;
}

int64_t HAL_GetFPGARevision(int32_t* status) {
  static_cast<void>(status);
  return 0;
}

// roboRIO identity metadata has no VMX equivalent, so these report empty
// rather than a placeholder that callers might display as real.
void HAL_GetSerialNumber(struct WPI_String* serialNumber) {
  if (serialNumber) {
    WPI_AllocateString(serialNumber, 0);
  }
}

void HAL_GetComments(struct WPI_String* comments) {
  if (comments) {
    WPI_AllocateString(comments, 0);
  }
}

// The team number lives in roboRIO identity storage, which VMX does not have.
int32_t HAL_GetTeamNumber(void) {
  return 0;
}

}  // extern "C"
