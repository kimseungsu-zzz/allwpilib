// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify and/or share it under the WPILib
// BSD license file in the root directory of this project.

#include "studica/VMXIMU.h"

#include <cstring>
#include <limits>

namespace {
constexpr StudicaVMXIMUHandle kUnsupportedHandle = 1;

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
}  // namespace

extern "C" {

int32_t StudicaVMXIMU_Create(StudicaVMXIMUHandle* handleOut) {
  if (!handleOut) {
    return STUDICA_VMX_IMU_INVALID_ARGUMENT;
  }
  *handleOut = kUnsupportedHandle;
  return STUDICA_VMX_IMU_UNAVAILABLE;
}

void StudicaVMXIMU_Destroy(StudicaVMXIMUHandle) {}

int32_t StudicaVMXIMU_ReadSnapshot(StudicaVMXIMUHandle handle,
                                   StudicaVMXIMUSnapshot* snapshotOut) {
  if (!snapshotOut) {
    return STUDICA_VMX_IMU_INVALID_ARGUMENT;
  }
  InitializeSnapshot(*snapshotOut);
  return handle == kUnsupportedHandle ? STUDICA_VMX_IMU_UNAVAILABLE
                                      : STUDICA_VMX_IMU_INVALID_ARGUMENT;
}

int32_t StudicaVMXIMU_ZeroYaw(StudicaVMXIMUHandle handle) {
  return handle == kUnsupportedHandle ? STUDICA_VMX_IMU_UNAVAILABLE
                                      : STUDICA_VMX_IMU_INVALID_ARGUMENT;
}

int32_t StudicaVMXIMU_Reset(StudicaVMXIMUHandle handle) {
  return handle == kUnsupportedHandle ? STUDICA_VMX_IMU_UNAVAILABLE
                                      : STUDICA_VMX_IMU_INVALID_ARGUMENT;
}

}  // extern "C"
