// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PowerInternal.h"

#include <string_view>

#include "VMXPi.h"

#include "HALInitializer.h"
#include "HALInternal.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"
#include "hal/HALBase.h"
#include "hal/Power.h"

namespace hal::vmx {
namespace {

void SetStatus(int32_t* status, int32_t value, std::string_view message = {}) {
  if (!status) {
    return;
  }
  *status = value;
  if (!message.empty() && value != HAL_SUCCESS) {
    hal::SetLastError(status, message);
  }
}

void SetUnsupported(int32_t* status, std::string_view feature) {
  SetStatus(status, INCOMPATIBLE_STATE, feature);
}

void SetUnavailable(int32_t* status, std::string_view feature) {
  SetStatus(status, INCOMPATIBLE_STATE, feature);
}

template <typename Function>
bool ReadVMXVoltage(Function&& function, double& voltage) noexcept {
  try {
    auto context = GetRuntimeContext();
    if (!context || !context->IsOpen()) {
      return false;
    }
    float value = 0.0f;
    VMXErrorCode error = 0;
    if (!function(context->getPower(), value, &error)) {
      return false;
    }
    voltage = static_cast<double>(value);
    return true;
  } catch (...) {
    voltage = 0.0;
    return false;
  }
}

}  // namespace
}  // namespace hal::vmx

extern "C" {

double HAL_GetVinVoltage(int32_t* status) {
  hal::init::CheckInit();
  double voltage = 0.0;
  const bool ok = hal::vmx::ReadVMXVoltage(
      [](auto& power, float& value, VMXErrorCode* error) {
        return power.GetSystemVoltage(value, error);
      },
      voltage);
  if (ok) {
    hal::vmx::SetStatus(status, HAL_SUCCESS);
  } else {
    hal::vmx::SetUnavailable(status,
                             "VMX system input voltage is unavailable");
  }
  return voltage;
}

double HAL_GetVinCurrent(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose input-current telemetry");
  return 0.0;
}

double HAL_GetUserVoltage6V(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose a 6V user rail");
  return 0.0;
}

double HAL_GetUserCurrent6V(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose 6V rail current");
  return 0.0;
}

HAL_Bool HAL_GetUserActive6V(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose 6V rail state");
  return false;
}

int32_t HAL_GetUserCurrentFaults6V(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose 6V rail faults");
  return 0;
}

void HAL_SetUserRailEnabled6V(HAL_Bool enabled, int32_t* status) {
  static_cast<void>(enabled);
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose 6V rail control");
}

double HAL_GetUserVoltage5V(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose a 5V user rail");
  return 0.0;
}

double HAL_GetUserCurrent5V(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose 5V rail current");
  return 0.0;
}

HAL_Bool HAL_GetUserActive5V(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose 5V rail state");
  return false;
}

int32_t HAL_GetUserCurrentFaults5V(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose 5V rail faults");
  return 0;
}

void HAL_SetUserRailEnabled5V(HAL_Bool enabled, int32_t* status) {
  static_cast<void>(enabled);
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose 5V rail control");
}

double HAL_GetUserVoltage3V3(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose a 3V3 user rail");
  return 0.0;
}

double HAL_GetUserCurrent3V3(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose 3V3 rail current");
  return 0.0;
}

HAL_Bool HAL_GetUserActive3V3(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose 3V3 rail state");
  return false;
}

int32_t HAL_GetUserCurrentFaults3V3(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose 3V3 rail faults");
  return 0;
}

void HAL_SetUserRailEnabled3V3(HAL_Bool enabled, int32_t* status) {
  static_cast<void>(enabled);
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose 3V3 rail control");
}

void HAL_ResetUserCurrentFaults(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose user-rail fault reset");
}

void HAL_SetBrownoutVoltage(double voltage, int32_t* status) {
  static_cast<void>(voltage);
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose brownout threshold control");
}

double HAL_GetBrownoutVoltage(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status,
                           "VMX SDK does not expose brownout threshold");
  return 0.0;
}

double HAL_GetCPUTemp(int32_t* status) {
  hal::init::CheckInit();
  double temperature = 0.0;
  if (hal::vmx::ReadCpuTemperatureFromSysfs("/sys/class/thermal",
                                            temperature)) {
    hal::vmx::SetStatus(status, HAL_SUCCESS);
    return temperature;
  }
  hal::vmx::SetUnavailable(
      status, "No readable CPU thermal zone was found in Linux sysfs");
  return 0.0;
}

HAL_Bool HAL_GetSystemActive(int32_t* status) {
  hal::init::CheckInit();
  const bool active = hal::vmx::IsRuntimeInitialized();
  hal::vmx::SetStatus(
      status, active ? HAL_SUCCESS : INCOMPATIBLE_STATE,
      "VMX runtime is not ready");
  return active;
}

HAL_Bool HAL_GetBrownedOut(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(
      status,
      "VMX overcurrent telemetry is not a brownout indication");
  return false;
}

int32_t HAL_GetCommsDisableCount(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(
      status, "VMX runtime has no Driver Station communication counter");
  return 0;
}

HAL_Bool HAL_GetFPGAButton(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status, "VMX hardware has no FPGA user button");
  return false;
}

HAL_Bool HAL_GetRSLState(int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetUnsupported(status, "VMX runtime has no RSL output");
  return false;
}

HAL_Bool HAL_GetSystemTimeValid(int32_t* status) {
  hal::init::CheckInit();
  int64_t unixSeconds = 0;
  if (!hal::vmx::ReadSystemUnixSeconds(unixSeconds)) {
    hal::vmx::SetUnavailable(status, "Linux wall clock could not be read");
    return false;
  }
  hal::vmx::SetStatus(status, HAL_SUCCESS);
  return hal::vmx::IsSystemTimeValidUnixSeconds(unixSeconds);
}

}  // extern "C"
