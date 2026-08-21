// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "studica/Titan.h"

#include <cmath>
#include <cstring>
#include <mutex>
#include <unordered_map>

namespace {
struct SimTitan {
  uint8_t canId;
  uint8_t motorPort;
  uint16_t frequency;
  double distancePerTick;
  bool inverted = false;
  bool reversed = false;
  double speed = 0.0;
};

std::mutex g_mutex;
std::unordered_map<StudicaTitanHandle, SimTitan> g_handles;
StudicaTitanHandle g_nextHandle = 1;

SimTitan* Find(StudicaTitanHandle handle) {
  auto it = g_handles.find(handle);
  return it == g_handles.end() ? nullptr : &it->second;
}

int32_t Check(StudicaTitanHandle handle, SimTitan** value) {
  if (!value) {
    return STUDICA_TITAN_INVALID_ARGUMENT;
  }
  *value = Find(handle);
  return *value ? STUDICA_TITAN_UNAVAILABLE : STUDICA_TITAN_INVALID_ARGUMENT;
}

template <typename T>
int32_t Unavailable(StudicaTitanHandle handle, T* out) {
  if (!out) {
    return STUDICA_TITAN_INVALID_ARGUMENT;
  }
  SimTitan* value;
  const auto status = Check(handle, &value);
  if (status != STUDICA_TITAN_UNAVAILABLE) {
    return status;
  }
  return STUDICA_TITAN_UNAVAILABLE;
}

int32_t UnavailableNoOutput(StudicaTitanHandle handle) {
  SimTitan* value;
  const auto status = Check(handle, &value);
  return status == STUDICA_TITAN_UNAVAILABLE ? STUDICA_TITAN_UNAVAILABLE
                                             : status;
}

void InitializeSnapshot(StudicaTitanSnapshot& snapshot) {
  std::memset(&snapshot, 0, sizeof(snapshot));
  snapshot.structSize = sizeof(snapshot);
  snapshot.abiVersion = STUDICA_TITAN_ABI_VERSION;
  snapshot.commandedSpeed = std::nan("");
  snapshot.encoderDistance = std::nan("");
  snapshot.rpm = std::nan("");
  snapshot.absoluteAngleDegrees = std::nan("");
  snapshot.controllerTemperatureC = std::nanf("");
}
}  // namespace

extern "C" {

int32_t StudicaTitan_Create(uint8_t canId, uint8_t motorPort,
                            uint16_t motorFrequencyHz, double distancePerTick,
                            StudicaTitanHandle* handleOut) {
  if (!handleOut || canId < STUDICA_TITAN_MIN_CAN_ID ||
      canId > STUDICA_TITAN_MAX_CAN_ID ||
      motorPort >= STUDICA_TITAN_MOTOR_COUNT || motorFrequencyHz == 0 ||
      motorFrequencyHz > 20000 || !std::isfinite(distancePerTick)) {
    return STUDICA_TITAN_INVALID_ARGUMENT;
  }
  std::scoped_lock lock{g_mutex};
  const auto handle = g_nextHandle++;
  g_handles.emplace(handle,
                    SimTitan{canId, motorPort, motorFrequencyHz,
                             distancePerTick});
  *handleOut = handle;
  return STUDICA_TITAN_OK;
}

void StudicaTitan_Destroy(StudicaTitanHandle handle) {
  std::scoped_lock lock{g_mutex};
  g_handles.erase(handle);
}

int32_t StudicaTitan_Set(StudicaTitanHandle handle, double speed) {
  std::scoped_lock lock{g_mutex};
  SimTitan* value;
  if (Check(handle, &value) != STUDICA_TITAN_UNAVAILABLE) {
    return handle ? STUDICA_TITAN_INVALID_ARGUMENT : STUDICA_TITAN_NOT_INITIALIZED;
  }
  static_cast<void>(speed);
  return STUDICA_TITAN_UNAVAILABLE;
}

int32_t StudicaTitan_Get(StudicaTitanHandle handle, double* speedOut) {
  std::scoped_lock lock{g_mutex};
  if (!speedOut) return STUDICA_TITAN_INVALID_ARGUMENT;
  SimTitan* value;
  if (Check(handle, &value) != STUDICA_TITAN_UNAVAILABLE) return STUDICA_TITAN_INVALID_ARGUMENT;
  *speedOut = std::nan("");
  return STUDICA_TITAN_UNAVAILABLE;
}

int32_t StudicaTitan_SetInverted(StudicaTitanHandle handle, uint8_t inverted) {
  std::scoped_lock lock{g_mutex};
  SimTitan* value;
  if (Check(handle, &value) != STUDICA_TITAN_UNAVAILABLE) return STUDICA_TITAN_INVALID_ARGUMENT;
  value->inverted = inverted != 0;
  return STUDICA_TITAN_UNAVAILABLE;
}

int32_t StudicaTitan_GetInverted(StudicaTitanHandle handle,
                                 uint8_t* invertedOut) {
  std::scoped_lock lock{g_mutex};
  if (!invertedOut) return STUDICA_TITAN_INVALID_ARGUMENT;
  SimTitan* value;
  if (Check(handle, &value) != STUDICA_TITAN_UNAVAILABLE) return STUDICA_TITAN_INVALID_ARGUMENT;
  *invertedOut = static_cast<uint8_t>(value->inverted);
  return STUDICA_TITAN_OK;
}

int32_t StudicaTitan_Enable(StudicaTitanHandle handle) { return UnavailableNoOutput(handle); }
int32_t StudicaTitan_Disable(StudicaTitanHandle handle) { return UnavailableNoOutput(handle); }
int32_t StudicaTitan_StopMotor(StudicaTitanHandle handle) { return UnavailableNoOutput(handle); }

int32_t StudicaTitan_GetSnapshot(StudicaTitanHandle handle,
                                 StudicaTitanSnapshot* snapshotOut) {
  if (!snapshotOut) return STUDICA_TITAN_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  InitializeSnapshot(*snapshotOut);
  SimTitan* value;
  if (Check(handle, &value) != STUDICA_TITAN_UNAVAILABLE) return STUDICA_TITAN_INVALID_ARGUMENT;
  snapshotOut->canId = value->canId;
  snapshotOut->motorPort = value->motorPort;
  snapshotOut->inverted = static_cast<uint8_t>(value->inverted);
  snapshotOut->distancePerTick = value->distancePerTick;
  snapshotOut->motorFrequencyHz = value->frequency;
  return STUDICA_TITAN_UNAVAILABLE;
}

#define STUDICA_TITAN_UNAVAILABLE_READER(name, type) \
  int32_t name(StudicaTitanHandle handle, type* out) { \
    std::scoped_lock lock{g_mutex}; \
    return Unavailable(handle, out); \
  }

STUDICA_TITAN_UNAVAILABLE_READER(StudicaTitan_GetEncoderCount, int32_t)
STUDICA_TITAN_UNAVAILABLE_READER(StudicaTitan_GetEncoderDistance, double)
STUDICA_TITAN_UNAVAILABLE_READER(StudicaTitan_GetRPM, double)
STUDICA_TITAN_UNAVAILABLE_READER(StudicaTitan_GetAbsoluteAngle, double)
STUDICA_TITAN_UNAVAILABLE_READER(StudicaTitan_GetForwardLimit, uint8_t)
STUDICA_TITAN_UNAVAILABLE_READER(StudicaTitan_GetReverseLimit, uint8_t)
STUDICA_TITAN_UNAVAILABLE_READER(StudicaTitan_GetControllerTemperature, float)

#undef STUDICA_TITAN_UNAVAILABLE_READER

int32_t StudicaTitan_IsAvailable(StudicaTitanHandle handle,
                                 uint8_t* availableOut) {
  if (!availableOut) return STUDICA_TITAN_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  SimTitan* value;
  if (Check(handle, &value) != STUDICA_TITAN_UNAVAILABLE) {
    return STUDICA_TITAN_INVALID_ARGUMENT;
  }
  *availableOut = 0;
  return STUDICA_TITAN_OK;
}

int32_t StudicaTitan_ResetEncoder(StudicaTitanHandle handle) { return UnavailableNoOutput(handle); }
int32_t StudicaTitan_SetDistancePerTick(StudicaTitanHandle handle, double value) {
  if (!std::isfinite(value)) return STUDICA_TITAN_INVALID_ARGUMENT;
  return UnavailableNoOutput(handle);
}
#define STUDICA_TITAN_UNAVAILABLE_COMMAND(name, type) \
  int32_t name(StudicaTitanHandle handle, type value) { \
    static_cast<void>(value); \
    return UnavailableNoOutput(handle); \
  }
STUDICA_TITAN_UNAVAILABLE_COMMAND(StudicaTitan_SetEncoderReversed, uint8_t)
STUDICA_TITAN_UNAVAILABLE_COMMAND(StudicaTitan_SetTargetVelocity, float)
STUDICA_TITAN_UNAVAILABLE_COMMAND(StudicaTitan_SetTargetDistance, int32_t)
STUDICA_TITAN_UNAVAILABLE_COMMAND(StudicaTitan_SetTargetAngle, double)
STUDICA_TITAN_UNAVAILABLE_COMMAND(StudicaTitan_SetPositionHold, uint8_t)
STUDICA_TITAN_UNAVAILABLE_COMMAND(StudicaTitan_SetMotorFrequency, uint16_t)
STUDICA_TITAN_UNAVAILABLE_COMMAND(StudicaTitan_SetCurrentLimit, float)
STUDICA_TITAN_UNAVAILABLE_COMMAND(StudicaTitan_SetCurrentLimitMode, uint8_t)
STUDICA_TITAN_UNAVAILABLE_COMMAND(StudicaTitan_SetMotorStopMode, uint8_t)
STUDICA_TITAN_UNAVAILABLE_COMMAND(StudicaTitan_SetPIDType, uint8_t)
#undef STUDICA_TITAN_UNAVAILABLE_COMMAND

int32_t CopyUnavailableString(StudicaTitanHandle handle, char* out,
                              uint32_t capacity) {
  if (!out || capacity == 0) return STUDICA_TITAN_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  SimTitan* value;
  if (Check(handle, &value) != STUDICA_TITAN_UNAVAILABLE) return STUDICA_TITAN_INVALID_ARGUMENT;
  out[0] = '\0';
  return STUDICA_TITAN_UNAVAILABLE;
}

int32_t StudicaTitan_GetFirmwareVersion(StudicaTitanHandle h, char* out, uint32_t c) { return CopyUnavailableString(h, out, c); }
int32_t StudicaTitan_GetHardwareVersion(StudicaTitanHandle h, char* out, uint32_t c) { return CopyUnavailableString(h, out, c); }

void StudicaTitan_ShutdownAll(void) {
  std::scoped_lock lock{g_mutex};
  g_handles.clear();
}

}  // extern "C"
