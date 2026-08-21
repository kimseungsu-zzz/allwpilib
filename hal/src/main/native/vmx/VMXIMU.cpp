// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify and/or share it under the WPILib
// BSD license file in the root directory of this project.

#include "studica/VMXIMU.h"

#include <cstring>
#include <memory>
#include <limits>

#include "VMXPi.h"
#include "VMXRuntime.h"

namespace {

constexpr StudicaVMXIMUHandle kHandle = 1;

bool IsHandleValid(StudicaVMXIMUHandle handle) noexcept {
  return handle == kHandle;
}

void InitializeSnapshot(StudicaVMXIMUSnapshot& snapshot) noexcept {
  std::memset(&snapshot, 0, sizeof(snapshot));
  snapshot.structSize = sizeof(snapshot);
  snapshot.abiVersion = STUDICA_VMX_IMU_ABI_VERSION;
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  snapshot.yaw = nan;
  snapshot.pitch = nan;
  snapshot.roll = nan;
  snapshot.accumulatedAngle = nan;
  snapshot.yawRate = nan;
  snapshot.quaternionW = nan;
  snapshot.quaternionX = nan;
  snapshot.quaternionY = nan;
  snapshot.quaternionZ = nan;
  snapshot.rawGyroX = nan;
  snapshot.rawGyroY = nan;
  snapshot.rawGyroZ = nan;
  snapshot.rawAccelX = nan;
  snapshot.rawAccelY = nan;
  snapshot.rawAccelZ = nan;
  snapshot.rawMagX = nan;
  snapshot.rawMagY = nan;
  snapshot.rawMagZ = nan;
  snapshot.worldLinearAccelX = nan;
  snapshot.worldLinearAccelY = nan;
  snapshot.worldLinearAccelZ = nan;
  snapshot.compassHeading = nan;
  snapshot.fusedHeading = nan;
  snapshot.temperatureC = nan;
  snapshot.pressure = nan;
  snapshot.altitude = nan;
}

std::shared_ptr<VMXPi> GetContext(StudicaVMXIMUHandle handle) noexcept {
  if (!IsHandleValid(handle) || !hal::vmx::IsRuntimeInitialized()) {
    return {};
  }
  return hal::vmx::GetRuntimeContext();
}

}  // namespace

extern "C" {

int32_t StudicaVMXIMU_Create(StudicaVMXIMUHandle* handleOut) {
  if (!handleOut) {
    return STUDICA_VMX_IMU_INVALID_ARGUMENT;
  }
  *handleOut = 0;
  // HAL initialization and all other VMX adapters own the VMXPi lifetime.
  // This call is idempotent and never creates a second VMXPi instance.
  if (!hal::vmx::InitializeRuntime()) {
    return STUDICA_VMX_IMU_UNAVAILABLE;
  }
  *handleOut = kHandle;
  return STUDICA_VMX_IMU_OK;
}

void StudicaVMXIMU_Destroy(StudicaVMXIMUHandle) {
  // The AHRS is owned by the shared VMXRuntime.  Destroying a vendor wrapper
  // must not stop it or release the VMXPi context used by the HAL.
}

int32_t StudicaVMXIMU_ReadSnapshot(StudicaVMXIMUHandle handle,
                                   StudicaVMXIMUSnapshot* snapshotOut) {
  if (!snapshotOut) {
    return STUDICA_VMX_IMU_INVALID_ARGUMENT;
  }
  InitializeSnapshot(*snapshotOut);
  if (!IsHandleValid(handle)) {
    return STUDICA_VMX_IMU_INVALID_ARGUMENT;
  }

  auto context = GetContext(handle);
  if (!context) {
    return STUDICA_VMX_IMU_UNAVAILABLE;
  }

  try {
    auto& ahrs = context->getAHRS();
    snapshotOut->yaw = ahrs.GetYaw();
    snapshotOut->pitch = ahrs.GetPitch();
    snapshotOut->roll = ahrs.GetRoll();
    snapshotOut->accumulatedAngle = ahrs.GetAngle();
    snapshotOut->yawRate = ahrs.GetRate();

    snapshotOut->quaternionW = ahrs.GetQuaternionW();
    snapshotOut->quaternionX = ahrs.GetQuaternionX();
    snapshotOut->quaternionY = ahrs.GetQuaternionY();
    snapshotOut->quaternionZ = ahrs.GetQuaternionZ();

    snapshotOut->rawGyroX = ahrs.GetRawGyroX();
    snapshotOut->rawGyroY = ahrs.GetRawGyroY();
    snapshotOut->rawGyroZ = ahrs.GetRawGyroZ();
    snapshotOut->rawAccelX = ahrs.GetRawAccelX();
    snapshotOut->rawAccelY = ahrs.GetRawAccelY();
    snapshotOut->rawAccelZ = ahrs.GetRawAccelZ();
    snapshotOut->rawMagX = ahrs.GetRawMagX();
    snapshotOut->rawMagY = ahrs.GetRawMagY();
    snapshotOut->rawMagZ = ahrs.GetRawMagZ();
    snapshotOut->worldLinearAccelX = ahrs.GetWorldLinearAccelX();
    snapshotOut->worldLinearAccelY = ahrs.GetWorldLinearAccelY();
    snapshotOut->worldLinearAccelZ = ahrs.GetWorldLinearAccelZ();

    snapshotOut->compassHeading = ahrs.GetCompassHeading();
    snapshotOut->fusedHeading = ahrs.GetFusedHeading();
    snapshotOut->temperatureC = ahrs.GetTempC();
    snapshotOut->pressure = ahrs.GetPressure();
    snapshotOut->altitude = ahrs.GetAltitude();

    snapshotOut->sensorTimestamp =
        static_cast<int64_t>(ahrs.GetLastSensorTimestamp());
    snapshotOut->moving = static_cast<uint8_t>(ahrs.IsMoving());
    snapshotOut->rotating = static_cast<uint8_t>(ahrs.IsRotating());
    snapshotOut->calibrating = static_cast<uint8_t>(ahrs.IsCalibrating());
    snapshotOut->connected = static_cast<uint8_t>(ahrs.IsConnected());
    snapshotOut->altitudeValid =
        static_cast<uint8_t>(ahrs.IsAltitudeValid());

    const auto firmware = ahrs.GetFirmwareVersion();
    std::strncpy(snapshotOut->firmwareVersion, firmware.c_str(),
                 sizeof(snapshotOut->firmwareVersion) - 1);
    snapshotOut->firmwareVersion[sizeof(snapshotOut->firmwareVersion) - 1] =
        '\0';
    return STUDICA_VMX_IMU_OK;
  } catch (...) {
    InitializeSnapshot(*snapshotOut);
    return STUDICA_VMX_IMU_INTERNAL_ERROR;
  }
}

int32_t StudicaVMXIMU_ZeroYaw(StudicaVMXIMUHandle handle) {
  auto context = GetContext(handle);
  if (!context) {
    return IsHandleValid(handle) ? STUDICA_VMX_IMU_UNAVAILABLE
                                 : STUDICA_VMX_IMU_INVALID_ARGUMENT;
  }
  try {
    context->getAHRS().ZeroYaw();
    return STUDICA_VMX_IMU_OK;
  } catch (...) {
    return STUDICA_VMX_IMU_INTERNAL_ERROR;
  }
}

int32_t StudicaVMXIMU_Reset(StudicaVMXIMUHandle handle) {
  auto context = GetContext(handle);
  if (!context) {
    return IsHandleValid(handle) ? STUDICA_VMX_IMU_UNAVAILABLE
                                 : STUDICA_VMX_IMU_INVALID_ARGUMENT;
  }
  try {
    context->getAHRS().Reset();
    return STUDICA_VMX_IMU_OK;
  } catch (...) {
    return STUDICA_VMX_IMU_INTERNAL_ERROR;
  }
}

}  // extern "C"
