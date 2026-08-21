// Copyright (c) 2026 WPILib contributors.

#include "studica/Colore.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "studica/ColorMatching.h"
#include "studica/Transport.h"

namespace {

struct Instance {
  StudicaColoreSnapshot snapshot{};
  StudicaColoreConfig config{};
  studica::detail::ColorMatcher matcher;
  uint8_t brightness = 0;
  int32_t status = STUDICA_COLORE_OK;
  uint32_t transport = 0;
  uint8_t canId = 0;
  std::string usbPath;
};

std::mutex g_mutex;
std::unordered_map<StudicaColoreHandle, std::unique_ptr<Instance>> g_instances;
StudicaColoreHandle g_nextHandle = 1;

Instance* Find(StudicaColoreHandle handle) {
  auto it = g_instances.find(handle);
  return it == g_instances.end() ? nullptr : it->second.get();
}

void Initialize(Instance& instance, uint32_t transport, uint8_t canId) {
  instance.snapshot = {};
  instance.snapshot.structSize = sizeof(instance.snapshot);
  instance.snapshot.abiVersion = STUDICA_COLORE_ABI_VERSION;
  instance.snapshot.transport = static_cast<uint8_t>(transport);
  instance.snapshot.connected = 0;
  instance.config = {};
  instance.config.structSize = sizeof(instance.config);
  instance.config.abiVersion = STUDICA_COLORE_ABI_VERSION;
  instance.config.transport = transport;
  instance.config.canId = canId;
}

int32_t GetComponent(Instance* instance, float* out, float value) {
  if (!out) return STUDICA_COLORE_INVALID_ARGUMENT;
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  *out = value;
  return STUDICA_COLORE_OK;
}

}  // namespace

extern "C" {

int32_t StudicaColore_CreateCAN(uint8_t canId,
                                StudicaColoreHandle* handleOut) {
  if (handleOut) *handleOut = 0;
  if (!handleOut || canId > 63) return STUDICA_COLORE_INVALID_ARGUMENT;
  if (!studica::GetStudicaTransportRegistry().ReserveCAN(
          0, canId, "Sim Studica Colore CAN"))
    return STUDICA_COLORE_RESOURCE_CONFLICT;
  std::scoped_lock lock{g_mutex};
  const auto handle = g_nextHandle++;
  auto instance = std::make_unique<Instance>();
  Initialize(*instance, 1, canId);
  instance->transport = 1;
  instance->canId = canId;
  g_instances.emplace(handle, std::move(instance));
  *handleOut = handle;
  return STUDICA_COLORE_OK;
}

int32_t StudicaColore_CreateUSB(const char* path,
                                StudicaColoreHandle* handleOut) {
  if (handleOut) *handleOut = 0;
  if (!handleOut || !path || !*path) return STUDICA_COLORE_INVALID_ARGUMENT;
  const std::string devicePath{path};
  if (!studica::GetStudicaTransportRegistry().ReserveUSB(
          devicePath, "Sim Studica Colore USB"))
    return STUDICA_COLORE_RESOURCE_CONFLICT;
  std::scoped_lock lock{g_mutex};
  const auto handle = g_nextHandle++;
  auto instance = std::make_unique<Instance>();
  Initialize(*instance, 2, 0);
  instance->transport = 2;
  instance->usbPath = devicePath;
  g_instances.emplace(handle, std::move(instance));
  *handleOut = handle;
  return STUDICA_COLORE_OK;
}

void StudicaColore_Destroy(StudicaColoreHandle handle) {
  std::scoped_lock lock{g_mutex};
  auto it = g_instances.find(handle);
  if (it == g_instances.end()) return;
  if (it->second->transport == 1) {
    studica::GetStudicaTransportRegistry().ReleaseCAN(0, it->second->canId);
  } else if (it->second->transport == 2) {
    studica::GetStudicaTransportRegistry().ReleaseUSB(it->second->usbPath);
  }
  g_instances.erase(it);
}

int32_t StudicaColore_Read(StudicaColoreHandle handle,
                           StudicaColoreSnapshot* snapshotOut) {
  if (!snapshotOut) return STUDICA_COLORE_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  *snapshotOut = instance->snapshot;
  instance->status = instance->snapshot.connected ? STUDICA_COLORE_OK
                                                  : STUDICA_COLORE_UNAVAILABLE;
  return instance->status;
}

int32_t StudicaColore_GetRed(StudicaColoreHandle handle, float* valueOut) {
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  return GetComponent(instance, valueOut, instance ? instance->snapshot.red
                                                   : 0.0F);
}
int32_t StudicaColore_GetGreen(StudicaColoreHandle handle, float* valueOut) {
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  return GetComponent(instance, valueOut, instance ? instance->snapshot.green
                                                   : 0.0F);
}
int32_t StudicaColore_GetBlue(StudicaColoreHandle handle, float* valueOut) {
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  return GetComponent(instance, valueOut, instance ? instance->snapshot.blue
                                                   : 0.0F);
}
int32_t StudicaColore_GetX(StudicaColoreHandle handle, float* valueOut) {
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  return GetComponent(instance, valueOut, instance ? instance->snapshot.x
                                                   : 0.0F);
}
int32_t StudicaColore_GetY(StudicaColoreHandle handle, float* valueOut) {
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  return GetComponent(instance, valueOut, instance ? instance->snapshot.y
                                                   : 0.0F);
}
int32_t StudicaColore_GetZ(StudicaColoreHandle handle, float* valueOut) {
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  return GetComponent(instance, valueOut, instance ? instance->snapshot.z
                                                   : 0.0F);
}

int32_t StudicaColore_SetBrightness(StudicaColoreHandle handle,
                                    int32_t percent) {
  if (percent < 0 || percent > 100) {
    std::scoped_lock lock{g_mutex};
    auto* instance = Find(handle);
    if (instance) instance->status = STUDICA_COLORE_INVALID_ARGUMENT;
    return instance ? STUDICA_COLORE_INVALID_ARGUMENT
                    : STUDICA_COLORE_NOT_INITIALIZED;
  }
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  instance->brightness = static_cast<uint8_t>(percent);
  instance->config.brightness = static_cast<uint32_t>(percent);
  return STUDICA_COLORE_OK;
}

int32_t StudicaColore_GetBrightness(StudicaColoreHandle handle,
                                    uint8_t* percentOut) {
  if (!percentOut) return STUDICA_COLORE_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  *percentOut = instance->brightness;
  return STUDICA_COLORE_OK;
}

int32_t StudicaColore_GetConfig(StudicaColoreHandle handle,
                                StudicaColoreConfig* configOut) {
  if (!configOut) return STUDICA_COLORE_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  *configOut = instance->config;
  return STUDICA_COLORE_OK;
}

int32_t StudicaColore_LearnColor(StudicaColoreHandle handle, const char* name,
                                 float threshold) {
  if (!name) return STUDICA_COLORE_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  return instance->matcher.SetReference(name, instance->snapshot.chromaticityX,
                                        instance->snapshot.chromaticityY,
                                        threshold)
             ? STUDICA_COLORE_OK
             : STUDICA_COLORE_INVALID_ARGUMENT;
}

int32_t StudicaColore_SetReference(StudicaColoreHandle handle, const char* name,
                                   float x, float y, float threshold) {
  if (!name) return STUDICA_COLORE_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  return instance->matcher.SetReference(name, x, y, threshold)
             ? STUDICA_COLORE_OK
             : STUDICA_COLORE_INVALID_ARGUMENT;
}

int32_t StudicaColore_GetLearnedReference(
    StudicaColoreHandle handle, const char* name,
    StudicaColoreMatchResult* resultOut) {
  if (!name || !resultOut) return STUDICA_COLORE_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  return instance->matcher.GetReference(name, resultOut)
             ? STUDICA_COLORE_OK
             : STUDICA_COLORE_UNAVAILABLE;
}

int32_t StudicaColore_Match(StudicaColoreHandle handle,
                            StudicaColoreMatchResult* resultOut) {
  if (!resultOut) return STUDICA_COLORE_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  return instance->matcher.Match(instance->snapshot.chromaticityX,
                                 instance->snapshot.chromaticityY, resultOut)
             ? STUDICA_COLORE_OK
             : STUDICA_COLORE_INVALID_ARGUMENT;
}

int32_t StudicaColore_IsConnected(StudicaColoreHandle handle,
                                  uint8_t* connectedOut) {
  if (!connectedOut) return STUDICA_COLORE_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  *connectedOut = instance->snapshot.connected;
  return STUDICA_COLORE_OK;
}

int32_t StudicaColore_GetLastStatus(StudicaColoreHandle handle,
                                    int32_t* statusOut) {
  if (!statusOut) return STUDICA_COLORE_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  *statusOut = instance->status;
  return STUDICA_COLORE_OK;
}

void StudicaColore_ShutdownAll(void) {
  std::scoped_lock lock{g_mutex};
  for (const auto& [handle, instance] : g_instances) {
    if (instance->transport == 1) {
      studica::GetStudicaTransportRegistry().ReleaseCAN(0, instance->canId);
    } else if (instance->transport == 2) {
      studica::GetStudicaTransportRegistry().ReleaseUSB(instance->usbPath);
    }
  }
  g_instances.clear();
}

int32_t StudicaColore_SetMockSnapshot(
    StudicaColoreHandle handle, const StudicaColoreSnapshot* snapshot) {
  if (!snapshot || snapshot->structSize < sizeof(StudicaColoreSnapshot))
    return STUDICA_COLORE_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  instance->snapshot = *snapshot;
  instance->snapshot.structSize = sizeof(instance->snapshot);
  instance->snapshot.abiVersion = STUDICA_COLORE_ABI_VERSION;
  instance->status = STUDICA_COLORE_OK;
  return STUDICA_COLORE_OK;
}

}  // extern "C"
