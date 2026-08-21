// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify and/or share it under the WPILib
// BSD license file in the root directory of this project.

#include "studica/VMXIMU.h"

#include <limits>

namespace {

template <typename Getter>
double ReadDouble(StudicaVMXIMUHandle handle, Getter getter) noexcept {
  StudicaVMXIMUSnapshot snapshot;
  if (StudicaVMXIMU_ReadSnapshot(handle, &snapshot) != STUDICA_VMX_IMU_OK) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return getter(snapshot);
}

}  // namespace

namespace studica {

VMXIMU::VMXIMU() noexcept { StudicaVMXIMU_Create(&m_handle); }

VMXIMU::~VMXIMU() { StudicaVMXIMU_Destroy(m_handle); }

VMXIMU::VMXIMU(VMXIMU&& other) noexcept : m_handle{other.m_handle} {
  other.m_handle = 0;
}

VMXIMU& VMXIMU::operator=(VMXIMU&& other) noexcept {
  if (this != &other) {
    StudicaVMXIMU_Destroy(m_handle);
    m_handle = other.m_handle;
    other.m_handle = 0;
  }
  return *this;
}

bool VMXIMU::IsAvailable() const noexcept {
  StudicaVMXIMUSnapshot snapshot;
  return ReadSnapshot(snapshot) && snapshot.connected;
}

bool VMXIMU::ReadSnapshot(StudicaVMXIMUSnapshot& snapshot) const noexcept {
  return StudicaVMXIMU_ReadSnapshot(m_handle, &snapshot) ==
         STUDICA_VMX_IMU_OK;
}

double VMXIMU::GetYaw() const noexcept {
  return ReadDouble(m_handle, [](const auto& s) { return s.yaw; });
}
double VMXIMU::GetPitch() const noexcept {
  return ReadDouble(m_handle, [](const auto& s) { return s.pitch; });
}
double VMXIMU::GetRoll() const noexcept {
  return ReadDouble(m_handle, [](const auto& s) { return s.roll; });
}
double VMXIMU::GetAngle() const noexcept {
  return ReadDouble(m_handle, [](const auto& s) { return s.accumulatedAngle; });
}
double VMXIMU::GetRate() const noexcept {
  return ReadDouble(m_handle, [](const auto& s) { return s.yawRate; });
}
bool VMXIMU::ZeroYaw() noexcept {
  return StudicaVMXIMU_ZeroYaw(m_handle) == STUDICA_VMX_IMU_OK;
}
bool VMXIMU::Reset() noexcept {
  return StudicaVMXIMU_Reset(m_handle) == STUDICA_VMX_IMU_OK;
}

#define STUDICA_VMXIMU_DOUBLE_GETTER(name, field)                       \
  double VMXIMU::name() const noexcept {                                \
    return ReadDouble(m_handle, [](const auto& s) { return s.field; });  \
  }

STUDICA_VMXIMU_DOUBLE_GETTER(GetQuaternionW, quaternionW)
STUDICA_VMXIMU_DOUBLE_GETTER(GetQuaternionX, quaternionX)
STUDICA_VMXIMU_DOUBLE_GETTER(GetQuaternionY, quaternionY)
STUDICA_VMXIMU_DOUBLE_GETTER(GetQuaternionZ, quaternionZ)
STUDICA_VMXIMU_DOUBLE_GETTER(GetRawGyroX, rawGyroX)
STUDICA_VMXIMU_DOUBLE_GETTER(GetRawGyroY, rawGyroY)
STUDICA_VMXIMU_DOUBLE_GETTER(GetRawGyroZ, rawGyroZ)
STUDICA_VMXIMU_DOUBLE_GETTER(GetRawAccelX, rawAccelX)
STUDICA_VMXIMU_DOUBLE_GETTER(GetRawAccelY, rawAccelY)
STUDICA_VMXIMU_DOUBLE_GETTER(GetRawAccelZ, rawAccelZ)
STUDICA_VMXIMU_DOUBLE_GETTER(GetRawMagX, rawMagX)
STUDICA_VMXIMU_DOUBLE_GETTER(GetRawMagY, rawMagY)
STUDICA_VMXIMU_DOUBLE_GETTER(GetRawMagZ, rawMagZ)
STUDICA_VMXIMU_DOUBLE_GETTER(GetWorldLinearAccelX, worldLinearAccelX)
STUDICA_VMXIMU_DOUBLE_GETTER(GetWorldLinearAccelY, worldLinearAccelY)
STUDICA_VMXIMU_DOUBLE_GETTER(GetWorldLinearAccelZ, worldLinearAccelZ)
STUDICA_VMXIMU_DOUBLE_GETTER(GetCompassHeading, compassHeading)
STUDICA_VMXIMU_DOUBLE_GETTER(GetFusedHeading, fusedHeading)
STUDICA_VMXIMU_DOUBLE_GETTER(GetTemperatureC, temperatureC)
STUDICA_VMXIMU_DOUBLE_GETTER(GetPressure, pressure)
STUDICA_VMXIMU_DOUBLE_GETTER(GetAltitude, altitude)

#undef STUDICA_VMXIMU_DOUBLE_GETTER

#define STUDICA_VMXIMU_BOOL_GETTER(name, field)                         \
  bool VMXIMU::name() const noexcept {                                  \
    StudicaVMXIMUSnapshot snapshot;                                     \
    return ReadSnapshot(snapshot) && snapshot.field != 0;                \
  }

STUDICA_VMXIMU_BOOL_GETTER(IsMoving, moving)
STUDICA_VMXIMU_BOOL_GETTER(IsRotating, rotating)
STUDICA_VMXIMU_BOOL_GETTER(IsCalibrating, calibrating)
STUDICA_VMXIMU_BOOL_GETTER(IsConnected, connected)
STUDICA_VMXIMU_BOOL_GETTER(IsAltitudeValid, altitudeValid)

#undef STUDICA_VMXIMU_BOOL_GETTER

int64_t VMXIMU::GetLastSensorTimestamp() const noexcept {
  StudicaVMXIMUSnapshot snapshot;
  return ReadSnapshot(snapshot) ? snapshot.sensorTimestamp : 0;
}

std::string VMXIMU::GetFirmwareVersion() const {
  StudicaVMXIMUSnapshot snapshot;
  if (!ReadSnapshot(snapshot)) {
    return {};
  }
  return snapshot.firmwareVersion;
}

}  // namespace studica
