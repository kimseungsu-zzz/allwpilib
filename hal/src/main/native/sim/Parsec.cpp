// Copyright (c) 2026 WPILib contributors.

#include "studica/Parsec.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>

#include "studica/Transport.h"

namespace {

struct Instance {
  StudicaParsecSnapshot snapshot{};
  StudicaParsecConfig config{};
  int32_t status = STUDICA_PARSEC_OK;
  uint32_t transport = 0;
  uint8_t canId = 0;
  std::string usbPath;
};

std::mutex g_mutex;
std::unordered_map<StudicaParsecHandle, Instance> g_instances;
StudicaParsecHandle g_nextHandle = 1;

Instance* Find(StudicaParsecHandle handle) {
  auto it = g_instances.find(handle);
  return it == g_instances.end() ? nullptr : &it->second;
}

int32_t ValidateSnapshot(const StudicaParsecSnapshot& snapshot) {
  if (snapshot.zoneCount != 16 && snapshot.zoneCount != 64) {
    return STUDICA_PARSEC_INVALID_ARGUMENT;
  }
  if (snapshot.resolution != STUDICA_PARSEC_RESOLUTION_4 &&
      snapshot.resolution != STUDICA_PARSEC_RESOLUTION_8) {
    return STUDICA_PARSEC_INVALID_ARGUMENT;
  }
  if (snapshot.zoneCount != snapshot.resolution * snapshot.resolution) {
    return STUDICA_PARSEC_INVALID_ARGUMENT;
  }
  return STUDICA_PARSEC_OK;
}

void Initialize(Instance& instance, uint32_t transport, uint8_t canId) {
  instance.snapshot = {};
  instance.snapshot.structSize = sizeof(instance.snapshot);
  instance.snapshot.abiVersion = STUDICA_PARSEC_ABI_VERSION;
  instance.snapshot.resolution = STUDICA_PARSEC_RESOLUTION_4;
  instance.snapshot.zoneCount = 16;
  instance.snapshot.transport = static_cast<uint8_t>(transport);
  instance.snapshot.connected = 0;
  std::fill(std::begin(instance.snapshot.distances),
            std::end(instance.snapshot.distances), static_cast<int16_t>(-1));
  instance.config = {};
  instance.config.structSize = sizeof(instance.config);
  instance.config.abiVersion = STUDICA_PARSEC_ABI_VERSION;
  instance.config.transport = transport;
  instance.config.resolution = 4;
  instance.config.zoneCount = 16;
  instance.config.canId = canId;
  instance.config.connected = 0;
  instance.config.rawLength = 0;
}

int32_t MinDistance(const Instance& instance, int16_t* distanceOut,
                   uint8_t* validOut) {
  if (!distanceOut || !validOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  int16_t minimum = 0;
  bool found = false;
  for (uint32_t i = 0; i < instance.snapshot.zoneCount; ++i) {
    const int16_t distance = instance.snapshot.distances[i];
    if (distance >= 0 && distance <= 4000 && (!found || distance < minimum)) {
      minimum = distance;
      found = true;
    }
  }
  *distanceOut = found ? minimum : 0;
  *validOut = found ? 1 : 0;
  return STUDICA_PARSEC_OK;
}

}  // namespace

extern "C" {

int32_t StudicaParsec_CreateCAN(uint8_t canId,
                                StudicaParsecHandle* handleOut) {
  if (handleOut) *handleOut = 0;
  if (!handleOut || canId > STUDICA_PARSEC_MAX_CAN_ID)
    return STUDICA_PARSEC_INVALID_ARGUMENT;
  if (!studica::GetStudicaTransportRegistry().ReserveCAN(
          0, canId, "Sim Studica Parsec CAN"))
    return STUDICA_PARSEC_RESOURCE_CONFLICT;
  std::scoped_lock lock{g_mutex};
  const auto handle = g_nextHandle++;
  Instance instance;
  Initialize(instance, 1, canId);
  instance.transport = 1;
  instance.canId = canId;
  g_instances.emplace(handle, instance);
  *handleOut = handle;
  return STUDICA_PARSEC_OK;
}

int32_t StudicaParsec_CreateUSB(const char* path,
                                StudicaParsecHandle* handleOut) {
  if (handleOut) *handleOut = 0;
  if (!handleOut || !path || !*path) return STUDICA_PARSEC_INVALID_ARGUMENT;
  const std::string devicePath{path};
  if (!studica::GetStudicaTransportRegistry().ReserveUSB(
          devicePath, "Sim Studica Parsec USB"))
    return STUDICA_PARSEC_RESOURCE_CONFLICT;
  std::scoped_lock lock{g_mutex};
  const auto handle = g_nextHandle++;
  Instance instance;
  Initialize(instance, 2, 0);
  instance.transport = 2;
  instance.usbPath = devicePath;
  g_instances.emplace(handle, instance);
  *handleOut = handle;
  return STUDICA_PARSEC_OK;
}

void StudicaParsec_Destroy(StudicaParsecHandle handle) {
  std::scoped_lock lock{g_mutex};
  auto it = g_instances.find(handle);
  if (it == g_instances.end()) return;
  if (it->second.transport == 1) {
    studica::GetStudicaTransportRegistry().ReleaseCAN(0, it->second.canId);
  } else if (it->second.transport == 2) {
    studica::GetStudicaTransportRegistry().ReleaseUSB(it->second.usbPath);
  }
  g_instances.erase(it);
}

int32_t StudicaParsec_ReadZones(StudicaParsecHandle handle,
                                StudicaParsecSnapshot* snapshotOut) {
  if (!snapshotOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  *snapshotOut = instance->snapshot;
  instance->status = instance->snapshot.connected ? STUDICA_PARSEC_OK
                                                  : STUDICA_PARSEC_UNAVAILABLE;
  return instance->status;
}

int32_t StudicaParsec_GetResolution(StudicaParsecHandle handle,
                                    uint32_t* resolutionOut) {
  if (!resolutionOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  *resolutionOut = instance->snapshot.resolution;
  return STUDICA_PARSEC_OK;
}

int32_t StudicaParsec_GetZoneCount(StudicaParsecHandle handle,
                                   uint32_t* zoneCountOut) {
  if (!zoneCountOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  *zoneCountOut = instance->snapshot.zoneCount;
  return STUDICA_PARSEC_OK;
}

int32_t StudicaParsec_GetZoneDistance(StudicaParsecHandle handle,
                                      uint32_t zone, int16_t* distanceOut) {
  if (!distanceOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  if (zone >= instance->snapshot.zoneCount)
    return STUDICA_PARSEC_INVALID_ARGUMENT;
  *distanceOut = instance->snapshot.distances[zone];
  return STUDICA_PARSEC_OK;
}

int32_t StudicaParsec_GetMinDistance(StudicaParsecHandle handle,
                                     int16_t* distanceOut,
                                     uint8_t* validOut) {
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  return MinDistance(*instance, distanceOut, validOut);
}

int32_t StudicaParsec_GetDistances(StudicaParsecHandle handle,
                                   int16_t* distancesOut,
                                   uint32_t capacity) {
  if (!distancesOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  if (capacity < instance->snapshot.zoneCount)
    return STUDICA_PARSEC_BUFFER_TOO_SMALL;
  std::memcpy(distancesOut, instance->snapshot.distances,
              instance->snapshot.zoneCount * sizeof(int16_t));
  return static_cast<int32_t>(instance->snapshot.zoneCount);
}

int32_t StudicaParsec_GetConfig(StudicaParsecHandle handle,
                                StudicaParsecConfig* configOut) {
  if (!configOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  *configOut = instance->config;
  return STUDICA_PARSEC_OK;
}

int32_t StudicaParsec_IsConnected(StudicaParsecHandle handle,
                                  uint8_t* connectedOut) {
  if (!connectedOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  *connectedOut = instance->snapshot.connected;
  return STUDICA_PARSEC_OK;
}

int32_t StudicaParsec_GetLastStatus(StudicaParsecHandle handle,
                                    int32_t* statusOut) {
  if (!statusOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  *statusOut = instance->status;
  return STUDICA_PARSEC_OK;
}

void StudicaParsec_ShutdownAll(void) {
  std::scoped_lock lock{g_mutex};
  for (const auto& [handle, instance] : g_instances) {
    if (instance.transport == 1) {
      studica::GetStudicaTransportRegistry().ReleaseCAN(0, instance.canId);
    } else if (instance.transport == 2) {
      studica::GetStudicaTransportRegistry().ReleaseUSB(instance.usbPath);
    }
  }
  g_instances.clear();
}

int32_t StudicaParsec_SetMockSnapshot(
    StudicaParsecHandle handle, const StudicaParsecSnapshot* snapshot) {
  if (!snapshot || snapshot->structSize < sizeof(StudicaParsecSnapshot))
    return STUDICA_PARSEC_INVALID_ARGUMENT;
  const auto valid = ValidateSnapshot(*snapshot);
  if (valid != STUDICA_PARSEC_OK) return valid;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  instance->snapshot = *snapshot;
  instance->snapshot.structSize = sizeof(instance->snapshot);
  instance->snapshot.abiVersion = STUDICA_PARSEC_ABI_VERSION;
  instance->status = STUDICA_PARSEC_OK;
  return STUDICA_PARSEC_OK;
}

}  // extern "C"
