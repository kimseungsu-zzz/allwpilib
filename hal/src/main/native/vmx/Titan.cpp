// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "studica/Titan.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "VMXPi.h"
#include "VMXRuntime.h"
#include "DriverStationInternal.h"
#include "titan.hpp"

namespace {

using DriverTitan = studica_driver::Titan;

constexpr uint16_t kDefaultMotorFrequency = 15600;

struct Controller final {
  Controller(uint8_t canId, uint16_t frequency, double dpt,
             std::shared_ptr<VMXPi> context)
      : canId{canId},
        frequency{frequency},
        vmx{std::move(context)},
        titan{std::make_unique<DriverTitan>(canId, frequency,
                                            static_cast<float>(dpt),
                                            vmx)} {
    distancePerTick.fill(dpt);
    for (uint8_t motor = 0; motor < STUDICA_TITAN_MOTOR_COUNT; ++motor) {
      titan->ConfigureEncoder(motor, distancePerTick[motor]);
    }
  }

  ~Controller() { Shutdown(); }

  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;

  void Start() {
    std::scoped_lock lock{mutex};
    if (worker.joinable()) {
      return;
    }
    stopRequested = false;
    worker = std::thread{[this] { Run(); }};
  }

  void Run() noexcept {
    std::unique_lock lock{mutex};
    while (!stopRequested) {
      cv.wait_for(lock, std::chrono::milliseconds(50));
      if (stopRequested) {
        break;
      }
      if (!enabled) {
        continue;
      }
      try {
        auto& ds = hal::vmx::GetDriverStationState();
        const bool safeToRun = ds.OutputsEnabled() && ds.IsFresh() &&
                               ds.IsProgramHeartbeatFresh() && !ds.IsEStop();
        if (!safeToRun) {
          // Driver Station disable, stale heartbeat, and e-stop are explicit
          // safety transitions.  Stop and disable the Titan rather than
          // relying on its 200 ms CAN watchdog timeout.
          speeds.fill(0.0);
          titan->SetSpeedAll(0.0);
          titan->Enable(false);
          enabled = false;
          continue;
        }
        // Titan's device watchdog is approximately 200 ms and its driver
        // documents a 150 ms keepalive.  Refresh every 50 ms, while retaining
        // the last commanded duty for every channel.
        for (uint8_t motor = 0; motor < STUDICA_TITAN_MOTOR_COUNT; ++motor) {
          titan->SetSpeed(motor, speeds[motor]);
        }
      } catch (...) {
        speeds.fill(0.0);
        enabled = false;
        try {
          titan->SetSpeedAll(0.0);
          titan->Enable(false);
        } catch (...) {
        }
      }
    }
  }

  void Shutdown() noexcept {
    std::thread toJoin;
    {
      std::scoped_lock lock{mutex};
      if (shutdownComplete) {
        return;
      }
      shutdownComplete = true;
      stopRequested = true;
      enabled = false;
      speeds.fill(0.0);
      // Explicitly command a zero output before disabling.  The adapter does
      // not rely on the Titan watchdog as a safety mechanism.
      try {
        titan->SetSpeedAll(0.0);
        titan->Enable(false);
      } catch (...) {
        // Destruction must not allow exceptions across the C ABI.
      }
      toJoin = std::move(worker);
    }
    cv.notify_all();
    if (toJoin.joinable()) {
      toJoin.join();
    }
  }

  int32_t Set(uint8_t motor, double speed) noexcept {
    if (motor >= STUDICA_TITAN_MOTOR_COUNT || !std::isfinite(speed)) {
      return STUDICA_TITAN_INVALID_ARGUMENT;
    }
    speed = std::clamp(speed, -1.0, 1.0);
    std::scoped_lock lock{mutex};
    if (shutdownComplete) {
      return STUDICA_TITAN_NOT_INITIALIZED;
    }
    speeds[motor] = speed;
    try {
      titan->SetSpeed(motor, speed);
    } catch (...) {
      return STUDICA_TITAN_INTERNAL_ERROR;
    }
    return STUDICA_TITAN_OK;
  }

  int32_t SetInverted(uint8_t motor, bool inverted) noexcept {
    if (motor >= STUDICA_TITAN_MOTOR_COUNT) {
      return STUDICA_TITAN_INVALID_ARGUMENT;
    }
    std::scoped_lock lock{mutex};
    if (shutdownComplete) return STUDICA_TITAN_NOT_INITIALIZED;
    if (motorInverted[motor] != inverted) {
      try {
        titan->InvertMotorDirection(motor);
      } catch (...) {
        return STUDICA_TITAN_INTERNAL_ERROR;
      }
      motorInverted[motor] = inverted;
    }
    return STUDICA_TITAN_OK;
  }

  int32_t Enable(bool value) noexcept {
    std::scoped_lock lock{mutex};
    if (shutdownComplete) return STUDICA_TITAN_NOT_INITIALIZED;
    try {
      if (value) {
        titan->Enable(true);
      } else {
        speeds.fill(0.0);
        titan->SetSpeedAll(0.0);
        titan->Enable(false);
      }
      enabled = value;
      return STUDICA_TITAN_OK;
    } catch (...) {
      enabled = false;
      return STUDICA_TITAN_INTERNAL_ERROR;
    }
  }

  int32_t Stop(uint8_t motor) noexcept {
    if (motor >= STUDICA_TITAN_MOTOR_COUNT) {
      return STUDICA_TITAN_INVALID_ARGUMENT;
    }
    std::scoped_lock lock{mutex};
    if (shutdownComplete) return STUDICA_TITAN_NOT_INITIALIZED;
    speeds[motor] = 0.0;
    try {
      titan->SetSpeed(motor, 0.0);
      return STUDICA_TITAN_OK;
    } catch (...) {
      return STUDICA_TITAN_INTERNAL_ERROR;
    }
  }

  int32_t Get(uint8_t motor, double* output) const noexcept {
    if (!output || motor >= STUDICA_TITAN_MOTOR_COUNT) {
      return STUDICA_TITAN_INVALID_ARGUMENT;
    }
    std::scoped_lock lock{mutex};
    if (shutdownComplete) return STUDICA_TITAN_NOT_INITIALIZED;
    *output = speeds[motor];
    return STUDICA_TITAN_OK;
  }

  int32_t GetInverted(uint8_t motor, uint8_t* output) const noexcept {
    if (!output || motor >= STUDICA_TITAN_MOTOR_COUNT) {
      return STUDICA_TITAN_INVALID_ARGUMENT;
    }
    std::scoped_lock lock{mutex};
    if (shutdownComplete) return STUDICA_TITAN_NOT_INITIALIZED;
    *output = static_cast<uint8_t>(motorInverted[motor]);
    return STUDICA_TITAN_OK;
  }

  int32_t GetEncoderCount(uint8_t motor, int32_t* output) const noexcept {
    if (!output || motor >= STUDICA_TITAN_MOTOR_COUNT) {
      return STUDICA_TITAN_INVALID_ARGUMENT;
    }
    std::scoped_lock lock{mutex};
    try {
      *output = titan->GetEncoderCount(motor);
      return STUDICA_TITAN_OK;
    } catch (...) {
      return STUDICA_TITAN_INTERNAL_ERROR;
    }
  }

  int32_t GetDistance(uint8_t motor, double* output) const noexcept {
    if (!output || motor >= STUDICA_TITAN_MOTOR_COUNT) {
      return STUDICA_TITAN_INVALID_ARGUMENT;
    }
    std::scoped_lock lock{mutex};
    try {
      *output = titan->GetEncoderDistance(motor);
      return STUDICA_TITAN_OK;
    } catch (...) {
      return STUDICA_TITAN_INTERNAL_ERROR;
    }
  }

  int32_t GetRPM(uint8_t motor, double* output) const noexcept {
    if (!output || motor >= STUDICA_TITAN_MOTOR_COUNT) {
      return STUDICA_TITAN_INVALID_ARGUMENT;
    }
    std::scoped_lock lock{mutex};
    try {
      *output = titan->GetRPM(motor);
      return STUDICA_TITAN_OK;
    } catch (...) {
      return STUDICA_TITAN_INTERNAL_ERROR;
    }
  }

  int32_t ResetEncoder(uint8_t motor) noexcept {
    if (motor >= STUDICA_TITAN_MOTOR_COUNT) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    try {
      titan->ResetEncoder(motor);
      return STUDICA_TITAN_OK;
    } catch (...) {
      return STUDICA_TITAN_INTERNAL_ERROR;
    }
  }

  int32_t SetDistancePerTick(uint8_t motor, double value) noexcept {
    if (motor >= STUDICA_TITAN_MOTOR_COUNT || !std::isfinite(value) ||
        value < 0.0) {
      return STUDICA_TITAN_INVALID_ARGUMENT;
    }
    std::scoped_lock lock{mutex};
    try {
      distancePerTick[motor] = value;
      titan->ConfigureEncoder(motor, value);
      return STUDICA_TITAN_OK;
    } catch (...) {
      return STUDICA_TITAN_INTERNAL_ERROR;
    }
  }

  int32_t SetEncoderReversed(uint8_t motor, bool reversed) noexcept {
    if (motor >= STUDICA_TITAN_MOTOR_COUNT) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    if (encoderReversed[motor] != reversed) {
      try {
        titan->InvertEncoderDirection(motor);
      } catch (...) {
        return STUDICA_TITAN_INTERNAL_ERROR;
      }
      encoderReversed[motor] = reversed;
    }
    return STUDICA_TITAN_OK;
  }

  int32_t GetAbsoluteAngle(uint8_t motor, double* output) const noexcept {
    if (!output || motor >= STUDICA_TITAN_MOTOR_COUNT) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    try {
      *output = titan->GetCypherAngle(motor);
      return STUDICA_TITAN_OK;
    } catch (...) {
      return STUDICA_TITAN_INTERNAL_ERROR;
    }
  }

  int32_t GetLimit(uint8_t motor, uint8_t direction, uint8_t* output) const noexcept {
    if (!output || motor >= STUDICA_TITAN_MOTOR_COUNT) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    try {
      *output = static_cast<uint8_t>(titan->GetLimitSwitch(motor, direction));
      return STUDICA_TITAN_OK;
    } catch (...) {
      return STUDICA_TITAN_INTERNAL_ERROR;
    }
  }

  int32_t SetTargetVelocity(uint8_t motor, float rpm) noexcept {
    if (motor >= STUDICA_TITAN_MOTOR_COUNT || !std::isfinite(rpm)) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    try { titan->SetTargetVelocity(motor, rpm); return STUDICA_TITAN_OK; }
    catch (...) { return STUDICA_TITAN_INTERNAL_ERROR; }
  }

  int32_t SetTargetDistance(uint8_t motor, int32_t counts) noexcept {
    if (motor >= STUDICA_TITAN_MOTOR_COUNT) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    try { titan->SetTargetDistance(motor, counts); return STUDICA_TITAN_OK; }
    catch (...) { return STUDICA_TITAN_INTERNAL_ERROR; }
  }

  int32_t SetTargetAngle(uint8_t motor, double angle) noexcept {
    if (motor >= STUDICA_TITAN_MOTOR_COUNT || !std::isfinite(angle)) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    try { titan->SetTargetAngle(motor, angle); return STUDICA_TITAN_OK; }
    catch (...) { return STUDICA_TITAN_INTERNAL_ERROR; }
  }

  int32_t SetPositionHold(uint8_t motor, bool hold) noexcept {
    if (motor >= STUDICA_TITAN_MOTOR_COUNT) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    try { titan->SetPositionHold(motor, hold); return STUDICA_TITAN_OK; }
    catch (...) { return STUDICA_TITAN_INTERNAL_ERROR; }
  }

  int32_t SetCurrentLimit(uint8_t motor, float limit) noexcept {
    if (motor >= STUDICA_TITAN_MOTOR_COUNT || !std::isfinite(limit) || limit < 0.0f) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    try { titan->SetCurrentLimit(motor, limit); return STUDICA_TITAN_OK; }
    catch (...) { return STUDICA_TITAN_INTERNAL_ERROR; }
  }

  int32_t SetCurrentLimitMode(uint8_t motor, uint8_t mode) noexcept {
    if (motor >= STUDICA_TITAN_MOTOR_COUNT) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    try { titan->SetCurrentLimitMode(motor, mode); return STUDICA_TITAN_OK; }
    catch (...) { return STUDICA_TITAN_INTERNAL_ERROR; }
  }

  int32_t SetMotorStopMode(uint8_t mode) noexcept {
    std::scoped_lock lock{mutex};
    try { titan->SetMotorStopMode(mode); return STUDICA_TITAN_OK; }
    catch (...) { return STUDICA_TITAN_INTERNAL_ERROR; }
  }

  int32_t SetPIDType(uint8_t type) noexcept {
    if (type > TITAN_PID_TYPE_MAX) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    try { titan->SetPIDType(type); return STUDICA_TITAN_OK; }
    catch (...) { return STUDICA_TITAN_INTERNAL_ERROR; }
  }

  int32_t CopyString(bool firmware, char* output, uint32_t capacity) const noexcept {
    if (!output || capacity == 0) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    try {
      const std::string value = firmware ? titan->GetFirmwareVersion()
                                         : titan->GetHardwareVersion();
      if (value.size() + 1 > capacity) return STUDICA_TITAN_BUFFER_TOO_SMALL;
      std::memcpy(output, value.c_str(), value.size() + 1);
      return STUDICA_TITAN_OK;
    } catch (...) { return STUDICA_TITAN_INTERNAL_ERROR; }
  }

  int32_t GetTemperature(float* output) const noexcept {
    if (!output) return STUDICA_TITAN_INVALID_ARGUMENT;
    std::scoped_lock lock{mutex};
    try { *output = titan->GetControllerTemp(); return STUDICA_TITAN_OK; }
    catch (...) { return STUDICA_TITAN_INTERNAL_ERROR; }
  }

  uint8_t canId;
  uint16_t frequency;
  std::shared_ptr<VMXPi> vmx;
  std::unique_ptr<DriverTitan> titan;
  mutable std::mutex mutex;
  std::condition_variable cv;
  std::thread worker;
  bool stopRequested = false;
  bool shutdownComplete = false;
  bool enabled = false;
  std::array<double, STUDICA_TITAN_MOTOR_COUNT> speeds{};
  std::array<bool, STUDICA_TITAN_MOTOR_COUNT> motorInverted{};
  std::array<bool, STUDICA_TITAN_MOTOR_COUNT> encoderReversed{};
  std::array<double, STUDICA_TITAN_MOTOR_COUNT> distancePerTick{};
};

struct View final {
  std::shared_ptr<Controller> controller;
  uint8_t motorPort;
};

std::mutex g_mutex;
std::unordered_map<uint8_t, std::shared_ptr<Controller>> g_controllers;
std::unordered_map<StudicaTitanHandle, std::shared_ptr<View>> g_views;
StudicaTitanHandle g_nextHandle = 1;

std::shared_ptr<View> Find(StudicaTitanHandle handle) noexcept {
  std::scoped_lock lock{g_mutex};
  auto it = g_views.find(handle);
  return it == g_views.end() ? nullptr : it->second;
}

bool HasViewFor(const std::shared_ptr<Controller>& controller) {
  for (const auto& [handle, view] : g_views) {
    static_cast<void>(handle);
    if (view->controller == controller) return true;
  }
  return false;
}

void InitializeSnapshot(StudicaTitanSnapshot& snapshot) noexcept {
  std::memset(&snapshot, 0, sizeof(snapshot));
  snapshot.structSize = sizeof(snapshot);
  snapshot.abiVersion = STUDICA_TITAN_ABI_VERSION;
  snapshot.commandedSpeed = std::numeric_limits<double>::quiet_NaN();
  snapshot.encoderDistance = std::numeric_limits<double>::quiet_NaN();
  snapshot.rpm = std::numeric_limits<double>::quiet_NaN();
  snapshot.absoluteAngleDegrees = std::numeric_limits<double>::quiet_NaN();
  snapshot.controllerTemperatureC =
      std::numeric_limits<float>::quiet_NaN();
}

template <typename Function>
int32_t Invoke(StudicaTitanHandle handle, Function&& function) noexcept {
  auto view = Find(handle);
  if (!view) return STUDICA_TITAN_INVALID_ARGUMENT;
  try {
    return function(*view);
  } catch (...) {
    return STUDICA_TITAN_INTERNAL_ERROR;
  }
}

}  // namespace

extern "C" {

int32_t StudicaTitan_Create(uint8_t canId, uint8_t motorPort,
                            uint16_t motorFrequencyHz, double distancePerTick,
                            StudicaTitanHandle* handleOut) {
  if (!handleOut || canId < STUDICA_TITAN_MIN_CAN_ID ||
      canId > STUDICA_TITAN_MAX_CAN_ID ||
      motorPort >= STUDICA_TITAN_MOTOR_COUNT || motorFrequencyHz == 0 ||
      motorFrequencyHz > 20000 || !std::isfinite(distancePerTick) ||
      distancePerTick < 0.0) {
    return STUDICA_TITAN_INVALID_ARGUMENT;
  }
  *handleOut = 0;
  if (!hal::vmx::InitializeRuntime()) return STUDICA_TITAN_UNAVAILABLE;
  auto context = hal::vmx::GetRuntimeContext();
  if (!context || !context->IsOpen()) return STUDICA_TITAN_UNAVAILABLE;

  try {
    std::shared_ptr<Controller> controller;
    {
      std::scoped_lock lock{g_mutex};
      auto it = g_controllers.find(canId);
      if (it != g_controllers.end()) {
        controller = it->second;
        if (controller->frequency != motorFrequencyHz) {
          return STUDICA_TITAN_INVALID_ARGUMENT;
        }
      } else {
        controller = std::make_shared<Controller>(
            canId, motorFrequencyHz, distancePerTick, std::move(context));
        g_controllers.emplace(canId, controller);
        controller->Start();
      }
      const auto handle = g_nextHandle++;
      g_views.emplace(handle, std::make_shared<View>(View{controller, motorPort}));
      *handleOut = handle;
    }
    return STUDICA_TITAN_OK;
  } catch (...) {
    return STUDICA_TITAN_INTERNAL_ERROR;
  }
}

void StudicaTitan_Destroy(StudicaTitanHandle handle) {
  std::shared_ptr<View> view;
  bool lastView = false;
  {
    std::scoped_lock lock{g_mutex};
    auto it = g_views.find(handle);
    if (it == g_views.end()) return;
    view = std::move(it->second);
    g_views.erase(it);
    lastView = !HasViewFor(view->controller);
    if (lastView) {
      g_controllers.erase(view->controller->canId);
    }
  }
  if (view && lastView) view->controller->Shutdown();
}

int32_t StudicaTitan_Set(StudicaTitanHandle handle, double speed) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->Set(v.motorPort, speed);
  });
}

int32_t StudicaTitan_Get(StudicaTitanHandle handle, double* speedOut) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->Get(v.motorPort, speedOut);
  });
}

int32_t StudicaTitan_SetInverted(StudicaTitanHandle handle, uint8_t inverted) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->SetInverted(v.motorPort, inverted != 0);
  });
}

int32_t StudicaTitan_GetInverted(StudicaTitanHandle handle,
                                 uint8_t* invertedOut) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->GetInverted(v.motorPort, invertedOut);
  });
}

int32_t StudicaTitan_Enable(StudicaTitanHandle handle) {
  return Invoke(handle, [](const View& v) { return v.controller->Enable(true); });
}

int32_t StudicaTitan_Disable(StudicaTitanHandle handle) {
  return Invoke(handle, [](const View& v) { return v.controller->Enable(false); });
}

int32_t StudicaTitan_StopMotor(StudicaTitanHandle handle) {
  return Invoke(handle, [](const View& v) { return v.controller->Stop(v.motorPort); });
}

int32_t StudicaTitan_GetSnapshot(StudicaTitanHandle handle,
                                 StudicaTitanSnapshot* snapshotOut) {
  if (!snapshotOut) return STUDICA_TITAN_INVALID_ARGUMENT;
  InitializeSnapshot(*snapshotOut);
  return Invoke(handle, [&](const View& v) {
    auto& c = *v.controller;
    snapshotOut->canId = c.canId;
    snapshotOut->motorPort = v.motorPort;
    snapshotOut->enabled = static_cast<uint8_t>(c.enabled);
    c.Get(v.motorPort, &snapshotOut->commandedSpeed);
    c.GetInverted(v.motorPort, &snapshotOut->inverted);
    c.GetEncoderCount(v.motorPort, &snapshotOut->encoderCount);
    c.GetDistance(v.motorPort, &snapshotOut->encoderDistance);
    c.GetRPM(v.motorPort, &snapshotOut->rpm);
    c.GetAbsoluteAngle(v.motorPort, &snapshotOut->absoluteAngleDegrees);
    c.GetLimit(v.motorPort, 0, &snapshotOut->forwardLimitTriggered);
    c.GetLimit(v.motorPort, 1, &snapshotOut->reverseLimitTriggered);
    snapshotOut->distancePerTick = c.distancePerTick[v.motorPort];
    snapshotOut->motorFrequencyHz = c.frequency;
    c.GetTemperature(&snapshotOut->controllerTemperatureC);
    c.CopyString(true, snapshotOut->firmwareVersion,
                 sizeof(snapshotOut->firmwareVersion));
    c.CopyString(false, snapshotOut->hardwareVersion,
                 sizeof(snapshotOut->hardwareVersion));
    snapshotOut->connected = 1;
    return STUDICA_TITAN_OK;
  });
}

#define STUDICA_TITAN_VIEW_CALL(name, method, type)                         \
  int32_t name(StudicaTitanHandle handle, type* output) {                   \
    return Invoke(handle, [&](const View& v) {                              \
      return v.controller->method(v.motorPort, output);                     \
    });                                                                     \
  }

STUDICA_TITAN_VIEW_CALL(StudicaTitan_GetEncoderCount, GetEncoderCount, int32_t)
STUDICA_TITAN_VIEW_CALL(StudicaTitan_GetEncoderDistance, GetDistance, double)
STUDICA_TITAN_VIEW_CALL(StudicaTitan_GetRPM, GetRPM, double)
STUDICA_TITAN_VIEW_CALL(StudicaTitan_GetAbsoluteAngle, GetAbsoluteAngle, double)

#undef STUDICA_TITAN_VIEW_CALL

// Direction-specific wrappers need to select the limit byte.
int32_t StudicaTitan_GetForwardLimit(StudicaTitanHandle handle,
                                     uint8_t* output) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->GetLimit(v.motorPort, 0, output);
  });
}
int32_t StudicaTitan_GetReverseLimit(StudicaTitanHandle handle,
                                     uint8_t* output) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->GetLimit(v.motorPort, 1, output);
  });
}

int32_t StudicaTitan_ResetEncoder(StudicaTitanHandle handle) {
  return Invoke(handle, [](const View& v) {
    return v.controller->ResetEncoder(v.motorPort);
  });
}
int32_t StudicaTitan_SetDistancePerTick(StudicaTitanHandle handle, double value) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->SetDistancePerTick(v.motorPort, value);
  });
}
int32_t StudicaTitan_SetEncoderReversed(StudicaTitanHandle handle, uint8_t value) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->SetEncoderReversed(v.motorPort, value != 0);
  });
}
int32_t StudicaTitan_SetTargetVelocity(StudicaTitanHandle handle, float value) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->SetTargetVelocity(v.motorPort, value);
  });
}
int32_t StudicaTitan_SetTargetDistance(StudicaTitanHandle handle, int32_t value) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->SetTargetDistance(v.motorPort, value);
  });
}
int32_t StudicaTitan_SetTargetAngle(StudicaTitanHandle handle, double value) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->SetTargetAngle(v.motorPort, value);
  });
}
int32_t StudicaTitan_SetPositionHold(StudicaTitanHandle handle, uint8_t value) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->SetPositionHold(v.motorPort, value != 0);
  });
}
int32_t StudicaTitan_SetMotorFrequency(StudicaTitanHandle handle,
                                       uint16_t value) {
  static_cast<void>(value);
  return Invoke(handle, [](const View&) { return STUDICA_TITAN_UNSUPPORTED; });
}
int32_t StudicaTitan_SetCurrentLimit(StudicaTitanHandle handle, float value) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->SetCurrentLimit(v.motorPort, value);
  });
}
int32_t StudicaTitan_SetCurrentLimitMode(StudicaTitanHandle handle,
                                         uint8_t value) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->SetCurrentLimitMode(v.motorPort, value);
  });
}
int32_t StudicaTitan_SetMotorStopMode(StudicaTitanHandle handle, uint8_t value) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->SetMotorStopMode(value);
  });
}
int32_t StudicaTitan_SetPIDType(StudicaTitanHandle handle, uint8_t value) {
  return Invoke(handle, [&](const View& v) { return v.controller->SetPIDType(value); });
}
int32_t StudicaTitan_GetFirmwareVersion(StudicaTitanHandle handle, char* out,
                                        uint32_t capacity) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->CopyString(true, out, capacity);
  });
}
int32_t StudicaTitan_GetHardwareVersion(StudicaTitanHandle handle, char* out,
                                        uint32_t capacity) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->CopyString(false, out, capacity);
  });
}
int32_t StudicaTitan_GetControllerTemperature(StudicaTitanHandle handle,
                                              float* output) {
  return Invoke(handle, [&](const View& v) {
    return v.controller->GetTemperature(output);
  });
}
int32_t StudicaTitan_IsAvailable(StudicaTitanHandle handle,
                                 uint8_t* output) {
  if (!output) return STUDICA_TITAN_INVALID_ARGUMENT;
  auto view = Find(handle);
  if (!view) return STUDICA_TITAN_INVALID_ARGUMENT;
  *output = static_cast<uint8_t>(hal::vmx::IsRuntimeInitialized() &&
                                 view->controller->vmx &&
                                 view->controller->vmx->IsOpen());
  return STUDICA_TITAN_OK;
}

void StudicaTitan_ShutdownAll(void) {
  std::unordered_map<uint8_t, std::shared_ptr<Controller>> controllers;
  {
    std::scoped_lock lock{g_mutex};
    g_views.clear();
    controllers.swap(g_controllers);
  }
  for (auto& [canId, controller] : controllers) {
    static_cast<void>(canId);
    controller->Shutdown();
  }
}

}  // extern "C"
