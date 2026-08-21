// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify and/or share it under the WPILib
// BSD license file in the root directory of this project.

#pragma once

#ifdef __cplusplus
#include <cstdint>
#include <string>
#else
#include <stdint.h>
#endif

// This is a vendor API.  It deliberately does not add orientation fields to
// the WPILib HAL accelerometer or gyro contracts.  The C layout below is the
// stable boundary for Java JNI and future Python/ctypes bindings.
#define STUDICA_VMX_IMU_ABI_VERSION 1u
#define STUDICA_VMX_IMU_FIRMWARE_VERSION_CAPACITY 96u

typedef uint32_t StudicaVMXIMUHandle;

#ifdef __cplusplus
enum StudicaVMXIMUStatus : int32_t {
#else
typedef enum StudicaVMXIMUStatus {
#endif
  STUDICA_VMX_IMU_OK = 0,
  STUDICA_VMX_IMU_INVALID_ARGUMENT = -22,
  STUDICA_VMX_IMU_UNAVAILABLE = -19,
  STUDICA_VMX_IMU_BUFFER_TOO_SMALL = -75,
  STUDICA_VMX_IMU_INTERNAL_ERROR = -5,
#ifdef __cplusplus
};
#else
} StudicaVMXIMUStatus;
#endif

// Fixed-width, versioned snapshot.  sensorTimestamp is an opaque VMX sensor
// timestamp: the SDK documents that it belongs to the last sensor sample but
// does not define a unit or epoch.  It is not HAL_GetFPGATime().
typedef struct StudicaVMXIMUSnapshot {
  uint32_t structSize;
  uint32_t abiVersion;

  double yaw;
  double pitch;
  double roll;
  double accumulatedAngle;
  double yawRate;

  double quaternionW;
  double quaternionX;
  double quaternionY;
  double quaternionZ;

  double rawGyroX;
  double rawGyroY;
  double rawGyroZ;
  double rawAccelX;
  double rawAccelY;
  double rawAccelZ;
  double rawMagX;
  double rawMagY;
  double rawMagZ;
  double worldLinearAccelX;
  double worldLinearAccelY;
  double worldLinearAccelZ;

  double compassHeading;
  double fusedHeading;
  double temperatureC;
  double pressure;
  double altitude;

  int64_t sensorTimestamp;
  uint8_t moving;
  uint8_t rotating;
  uint8_t calibrating;
  uint8_t connected;
  uint8_t altitudeValid;
  uint8_t reserved[3];

  char firmwareVersion[STUDICA_VMX_IMU_FIRMWARE_VERSION_CAPACITY];
} StudicaVMXIMUSnapshot;

#ifdef __cplusplus

namespace studica {

/** Shared VMX onboard IMU vendor wrapper.
 *
 * The default object only retains a logical handle.  It reuses the VMXPi and
 * AHRS owned by VMXRuntime; it never constructs or stops a second AHRS.
 */
class VMXIMU final {
 public:
  VMXIMU() noexcept;
  ~VMXIMU();

  VMXIMU(const VMXIMU&) = delete;
  VMXIMU& operator=(const VMXIMU&) = delete;
  VMXIMU(VMXIMU&& other) noexcept;
  VMXIMU& operator=(VMXIMU&& other) noexcept;

  bool IsAvailable() const noexcept;
  bool ReadSnapshot(StudicaVMXIMUSnapshot& snapshot) const noexcept;

  double GetYaw() const noexcept;
  double GetPitch() const noexcept;
  double GetRoll() const noexcept;
  double GetAngle() const noexcept;
  double GetRate() const noexcept;
  bool ZeroYaw() noexcept;
  bool Reset() noexcept;

  double GetQuaternionW() const noexcept;
  double GetQuaternionX() const noexcept;
  double GetQuaternionY() const noexcept;
  double GetQuaternionZ() const noexcept;
  double GetRawGyroX() const noexcept;
  double GetRawGyroY() const noexcept;
  double GetRawGyroZ() const noexcept;
  double GetRawAccelX() const noexcept;
  double GetRawAccelY() const noexcept;
  double GetRawAccelZ() const noexcept;
  double GetRawMagX() const noexcept;
  double GetRawMagY() const noexcept;
  double GetRawMagZ() const noexcept;
  double GetWorldLinearAccelX() const noexcept;
  double GetWorldLinearAccelY() const noexcept;
  double GetWorldLinearAccelZ() const noexcept;
  double GetCompassHeading() const noexcept;
  double GetFusedHeading() const noexcept;
  bool IsMoving() const noexcept;
  bool IsRotating() const noexcept;
  bool IsCalibrating() const noexcept;
  bool IsConnected() const noexcept;
  int64_t GetLastSensorTimestamp() const noexcept;
  double GetTemperatureC() const noexcept;
  double GetPressure() const noexcept;
  double GetAltitude() const noexcept;
  bool IsAltitudeValid() const noexcept;
  std::string GetFirmwareVersion() const;

 private:
  StudicaVMXIMUHandle m_handle = 0;
};

}  // namespace studica

extern "C" {
#endif

int32_t StudicaVMXIMU_Create(StudicaVMXIMUHandle* handleOut);
void StudicaVMXIMU_Destroy(StudicaVMXIMUHandle handle);
int32_t StudicaVMXIMU_ReadSnapshot(StudicaVMXIMUHandle handle,
                                   StudicaVMXIMUSnapshot* snapshotOut);
int32_t StudicaVMXIMU_ZeroYaw(StudicaVMXIMUHandle handle);
int32_t StudicaVMXIMU_Reset(StudicaVMXIMUHandle handle);

#ifdef __cplusplus
}
#endif
