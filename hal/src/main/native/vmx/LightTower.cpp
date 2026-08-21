// Copyright (c) 2026 WPILib contributors.

#include "studica/LightTower.h"

#include <array>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "VMXRuntime.h"
#include "VMXConstants.h"
#include "light_tower.hpp"

namespace {

struct LightTowerInstance {
  LightTowerInstance(std::array<uint8_t, 5> pins,
                     std::shared_ptr<VMXPi> context)
      : pins{pins},
        driver{std::make_unique<studica_driver::LightTower>(
            pins[0], pins[1], pins[2], pins[3], pins[4],
            std::move(context))} {}

  ~LightTowerInstance() {
    auto& registry = hal::vmx::GetDigitalChannelRegistry();
    for (const auto pin : pins) {
      registry.Release(static_cast<int32_t>(pin),
                       hal::vmx::DigitalChannelOwner::kLightTower);
    }
  }

  std::array<uint8_t, 5> pins;
  std::unique_ptr<studica_driver::LightTower> driver;
};

std::mutex g_mutex;
std::unordered_map<StudicaLightTowerHandle,
                   std::unique_ptr<LightTowerInstance>>
    g_instances;
StudicaLightTowerHandle g_nextHandle = 1;

LightTowerInstance* Find(StudicaLightTowerHandle handle) {
  auto it = g_instances.find(handle);
  return it == g_instances.end() ? nullptr : it->second.get();
}

bool ValidPins(const std::array<uint8_t, 5>& pins) {
  for (std::size_t i = 0; i < pins.size(); ++i) {
    if (!hal::vmx::IsPhysicalChannelValid(pins[i])) return false;
    for (std::size_t j = 0; j < i; ++j) {
      if (pins[i] == pins[j]) return false;
    }
  }
  return true;
}

bool ReservePins(const std::array<uint8_t, 5>& pins) {
  auto& registry = hal::vmx::GetDigitalChannelRegistry();
  std::size_t reserved = 0;
  for (const auto pin : pins) {
    if (!registry
             .Reserve(static_cast<int32_t>(pin),
                     hal::vmx::DigitalChannelOwner::kLightTower,
                     "Studica Light Tower")
             .reserved) {
      for (std::size_t i = 0; i < reserved; ++i) {
        registry.Release(static_cast<int32_t>(pins[i]),
                         hal::vmx::DigitalChannelOwner::kLightTower);
      }
      return false;
    }
    ++reserved;
  }
  return true;
}

template <typename Operation>
int32_t Call(StudicaLightTowerHandle handle, Operation&& operation) {
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance || !instance->driver) return STUDICA_LIGHT_TOWER_NOT_INITIALIZED;
  try {
    operation(*instance->driver);
    return STUDICA_LIGHT_TOWER_OK;
  } catch (...) {
    return STUDICA_LIGHT_TOWER_INTERNAL_ERROR;
  }
}

}  // namespace

extern "C" {

int32_t StudicaLightTower_Create(uint8_t continuous, uint8_t red,
                                 uint8_t green, uint8_t yellow,
                                 uint8_t buzzer,
                                 StudicaLightTowerHandle* handleOut) {
  const std::array<uint8_t, 5> pins{continuous, red, green, yellow, buzzer};
  if (!handleOut || !ValidPins(pins)) return STUDICA_LIGHT_TOWER_INVALID_ARGUMENT;
  auto context = hal::vmx::GetRuntimeContext();
  if (!context || !context->IsOpen()) return STUDICA_LIGHT_TOWER_UNAVAILABLE;
  if (!ReservePins(pins)) return STUDICA_LIGHT_TOWER_RESOURCE_CONFLICT;
  try {
    auto instance = std::make_unique<LightTowerInstance>(pins,
                                                           std::move(context));
    const auto handle = g_nextHandle++;
    std::scoped_lock lock{g_mutex};
    g_instances.emplace(handle, std::move(instance));
    *handleOut = handle;
    return STUDICA_LIGHT_TOWER_OK;
  } catch (...) {
    auto& registry = hal::vmx::GetDigitalChannelRegistry();
    for (const auto pin : pins) {
      registry.Release(static_cast<int32_t>(pin),
                       hal::vmx::DigitalChannelOwner::kLightTower);
    }
    return STUDICA_LIGHT_TOWER_INTERNAL_ERROR;
  }
}

void StudicaLightTower_Destroy(StudicaLightTowerHandle handle) {
  std::scoped_lock lock{g_mutex};
  g_instances.erase(handle);
}

int32_t StudicaLightTower_SetRed(StudicaLightTowerHandle handle,
                                 uint8_t enabled) {
  return Call(handle, [enabled](auto& tower) { tower.SetRed(enabled != 0); });
}
int32_t StudicaLightTower_SetYellow(StudicaLightTowerHandle handle,
                                    uint8_t enabled) {
  return Call(handle,
              [enabled](auto& tower) { tower.SetYellow(enabled != 0); });
}
int32_t StudicaLightTower_SetGreen(StudicaLightTowerHandle handle,
                                   uint8_t enabled) {
  return Call(handle,
              [enabled](auto& tower) { tower.SetGreen(enabled != 0); });
}
int32_t StudicaLightTower_SetBuzzer(StudicaLightTowerHandle handle,
                                    uint8_t enabled) {
  return Call(handle,
              [enabled](auto& tower) { tower.SetBuzzer(enabled != 0); });
}
int32_t StudicaLightTower_SetSolid(StudicaLightTowerHandle handle) {
  return Call(handle, [](auto& tower) { tower.SetContinuous(true); });
}
int32_t StudicaLightTower_SetBlink(StudicaLightTowerHandle handle) {
  return Call(handle, [](auto& tower) { tower.SetContinuous(false); });
}
int32_t StudicaLightTower_Off(StudicaLightTowerHandle handle) {
  return Call(handle, [](auto& tower) { tower.AllOff(); });
}
int32_t StudicaLightTower_IsAvailable(StudicaLightTowerHandle handle,
                                      uint8_t* availableOut) {
  if (!availableOut) return STUDICA_LIGHT_TOWER_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  *availableOut = Find(handle) ? 1 : 0;
  return STUDICA_LIGHT_TOWER_OK;
}

void StudicaLightTower_ShutdownAll(void) {
  std::scoped_lock lock{g_mutex};
  g_instances.clear();
}

}  // extern "C"
