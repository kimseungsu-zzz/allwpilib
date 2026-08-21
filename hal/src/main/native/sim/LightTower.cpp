// Copyright (c) 2026 WPILib contributors.

#include "studica/LightTower.h"

#include <array>
#include <mutex>
#include <unordered_map>

namespace {
std::mutex g_mutex;
std::unordered_map<StudicaLightTowerHandle, std::array<uint8_t, 5>> g_instances;
StudicaLightTowerHandle g_nextHandle = 1;

bool Valid(const std::array<uint8_t, 5>& pins) {
  for (std::size_t i = 0; i < pins.size(); ++i) {
    if (pins[i] >= 34) return false;
    for (std::size_t j = 0; j < i; ++j) {
      if (pins[i] == pins[j]) return false;
    }
  }
  return true;
}
}

extern "C" {

int32_t StudicaLightTower_Create(uint8_t continuous, uint8_t red,
                                 uint8_t green, uint8_t yellow,
                                 uint8_t buzzer,
                                 StudicaLightTowerHandle* handleOut) {
  const std::array<uint8_t, 5> pins{continuous, red, green, yellow, buzzer};
  if (!handleOut || !Valid(pins)) return STUDICA_LIGHT_TOWER_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  const auto handle = g_nextHandle++;
  g_instances.emplace(handle, pins);
  *handleOut = handle;
  return STUDICA_LIGHT_TOWER_OK;
}

void StudicaLightTower_Destroy(StudicaLightTowerHandle handle) {
  std::scoped_lock lock{g_mutex};
  g_instances.erase(handle);
}

int32_t Unavailable(StudicaLightTowerHandle handle) {
  std::scoped_lock lock{g_mutex};
  return g_instances.count(handle) ? STUDICA_LIGHT_TOWER_UNAVAILABLE
                                    : STUDICA_LIGHT_TOWER_NOT_INITIALIZED;
}
int32_t StudicaLightTower_SetRed(StudicaLightTowerHandle h, uint8_t v) {
  static_cast<void>(v);
  return Unavailable(h);
}
int32_t StudicaLightTower_SetYellow(StudicaLightTowerHandle h, uint8_t v) {
  static_cast<void>(v);
  return Unavailable(h);
}
int32_t StudicaLightTower_SetGreen(StudicaLightTowerHandle h, uint8_t v) {
  static_cast<void>(v);
  return Unavailable(h);
}
int32_t StudicaLightTower_SetBuzzer(StudicaLightTowerHandle h, uint8_t v) {
  static_cast<void>(v);
  return Unavailable(h);
}
int32_t StudicaLightTower_SetSolid(StudicaLightTowerHandle h) {
  return Unavailable(h);
}
int32_t StudicaLightTower_SetBlink(StudicaLightTowerHandle h) {
  return Unavailable(h);
}
int32_t StudicaLightTower_Off(StudicaLightTowerHandle h) {
  return Unavailable(h);
}
int32_t StudicaLightTower_IsAvailable(StudicaLightTowerHandle handle,
                                      uint8_t* availableOut) {
  if (!availableOut) return STUDICA_LIGHT_TOWER_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  *availableOut = 0;
  return g_instances.count(handle) ? STUDICA_LIGHT_TOWER_OK
                                    : STUDICA_LIGHT_TOWER_NOT_INITIALIZED;
}

void StudicaLightTower_ShutdownAll(void) {
  std::scoped_lock lock{g_mutex};
  g_instances.clear();
}

}  // extern "C"
