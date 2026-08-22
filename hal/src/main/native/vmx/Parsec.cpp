// Copyright (c) 2026 WPILib contributors.

#include "studica/Parsec.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "VMXPi.h"
#include "VMXRuntime.h"
#include "parsec.hpp"
#include "parsec_usb.hpp"
#include "studica/Transport.h"

namespace {

using DriverParsec = studica_driver::Parsec;
using DriverParsecUsb = studica_driver::ParsecUsb;

uint32_t ResolutionForZones(uint32_t zones) {
  return zones == 64 ? STUDICA_PARSEC_RESOLUTION_8
                     : STUDICA_PARSEC_RESOLUTION_4;
}

uint32_t ZoneCountFromMask(const uint8_t* bytes, std::size_t length) {
  if (!bytes || length < 44) return 0;
  uint64_t mask = 0;
  for (int i = 0; i < 8; ++i) {
    mask |= static_cast<uint64_t>(bytes[36 + i]) << (i * 8);
  }
  const auto count = static_cast<uint32_t>(std::popcount(mask));
  return count == 64 ? 64 : (count >= 16 ? 16 : 0);
}

struct ParsecInstance final {
  explicit ParsecInstance(uint8_t canId, std::shared_ptr<VMXPi> context)
      : kind{studica::TransportKind::kCAN}, canId{canId},
        context{std::move(context)} {
    channel = DriverParsec::GetCANChannel();
    driver = std::make_unique<DriverParsec>(canId, this->context);
    snapshot.structSize = sizeof(snapshot);
    snapshot.abiVersion = STUDICA_PARSEC_ABI_VERSION;
    snapshot.transport = static_cast<uint8_t>(kind);
    snapshot.connected = driver && this->context && this->context->IsOpen();
    std::fill(std::begin(snapshot.distances), std::end(snapshot.distances),
              static_cast<int16_t>(-1));
    config.structSize = sizeof(config);
    config.abiVersion = STUDICA_PARSEC_ABI_VERSION;
    config.transport = static_cast<uint32_t>(kind);
    config.canId = canId;
  }

  explicit ParsecInstance(std::string path)
      : kind{studica::TransportKind::kUSB}, usbPath{std::move(path)} {
    driverUsb = std::make_unique<DriverParsecUsb>(usbPath);
    snapshot.structSize = sizeof(snapshot);
    snapshot.abiVersion = STUDICA_PARSEC_ABI_VERSION;
    snapshot.transport = static_cast<uint8_t>(kind);
    snapshot.connected = driverUsb && driverUsb->IsOpen();
    std::fill(std::begin(snapshot.distances), std::end(snapshot.distances),
              static_cast<int16_t>(-1));
    config.structSize = sizeof(config);
    config.abiVersion = STUDICA_PARSEC_ABI_VERSION;
    config.transport = static_cast<uint32_t>(kind);
    if (driverUsb && driverUsb->IsOpen()) {
      // The immutable upstream driver remains the USB protocol/parser source.
      // Configuration failure does not prevent later frame reads.
      driverUsb->ConfigureStreaming(15);
    }
  }

  ~ParsecInstance() {
    if (kind == studica::TransportKind::kUSB) {
      studica::GetStudicaTransportRegistry().ReleaseUSB(usbPath);
    } else {
      studica::GetStudicaTransportRegistry().ReleaseCAN(channel, canId);
    }
  }

  int32_t Read() noexcept {
    std::scoped_lock lock{mutex};
    lastStatus = kind == studica::TransportKind::kUSB ? ReadUSB() : ReadCAN();
    return lastStatus;
  }

  int32_t ReadUSB() noexcept {
    if (!driverUsb || !driverUsb->IsOpen()) {
      snapshot.connected = 0;
      return STUDICA_PARSEC_UNAVAILABLE;
    }
    uint8_t sequence = 0;
    uint8_t zones = 0;
    std::array<int16_t, STUDICA_PARSEC_MAX_ZONES> distances{};
    const int count = driverUsb->ReadLatestFdist(
        &sequence, &zones, distances.data(), distances.size());
    if (count <= 0 || (zones != 16 && zones != 64)) {
      snapshot.connected = 1;
      return STUDICA_PARSEC_TIMEOUT;
    }
    std::copy(distances.begin(), distances.end(), std::begin(snapshot.distances));
    snapshot.resolution = ResolutionForZones(zones);
    snapshot.zoneCount = zones;
    snapshot.sequence = ++sequenceCounter;
    snapshot.connected = 1;
    snapshot.valid = 1;
    config.resolution = snapshot.resolution;
    config.zoneCount = snapshot.zoneCount;
    return STUDICA_PARSEC_OK;
  }

  int32_t ReadCAN() noexcept {
    if (!driver || !context || !context->IsOpen()) {
      snapshot.connected = 0;
      return STUDICA_PARSEC_UNAVAILABLE;
    }
    uint8_t sequence = 0;
    uint8_t zones = 0;
    std::array<int16_t, STUDICA_PARSEC_MAX_ZONES> distances{};
    int count = driver->ReadDataStreamCAN2(&sequence, &zones, distances.data(),
                                           distances.size());
    if (count <= 0) {
      // CAN-FD frames expose four 16-zone chunks; the immutable driver owns
      // frame decoding while this adapter only assembles the fixed snapshot.
      count = 0;
      for (uint32_t chunk = 0; chunk < 4; ++chunk) {
        std::array<uint8_t, 64> frame{};
        const int bytes = driver->ReadDataChunk(chunk, frame.data(), frame.size());
        if (bytes < 2) continue;
        sequence = frame[0];
        if (chunk == 0 && (frame[1] == 16 || frame[1] == 64)) zones = frame[1];
        const int samples = std::min<int>((bytes - 2) / 2, 16);
        const int offset = static_cast<int>(chunk * 16);
        for (int i = 0; i < samples && offset + i < 64; ++i) {
          const auto lo = frame[2 + i * 2];
          const auto hi = frame[3 + i * 2];
          distances[offset + i] = static_cast<int16_t>(
              static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8));
          count = std::max(count, offset + i + 1);
        }
      }
    }
    if (count <= 0 || (zones != 16 && zones != 64)) {
      snapshot.connected = 1;
      return STUDICA_PARSEC_TIMEOUT;
    }
    snapshot.resolution = ResolutionForZones(zones);
    snapshot.zoneCount = zones;
    std::copy(distances.begin(), distances.end(), std::begin(snapshot.distances));
    snapshot.sequence = ++sequenceCounter;
    snapshot.connected = 1;
    snapshot.valid = 1;
    config.resolution = snapshot.resolution;
    config.zoneCount = snapshot.zoneCount;
    return STUDICA_PARSEC_OK;
  }

  int32_t GetConfig() noexcept {
    std::scoped_lock lock{mutex};
    if (config.rawLength != 0) return lastStatus = STUDICA_PARSEC_OK;
    if (kind == studica::TransportKind::kUSB) {
      if (!driverUsb || !driverUsb->IsOpen())
        return lastStatus = STUDICA_PARSEC_UNAVAILABLE;
      std::string response;
      if (!driverUsb->RequestConfig(&response))
        return lastStatus = STUDICA_PARSEC_TIMEOUT;
      config.rawLength = static_cast<uint16_t>(
          std::min<std::size_t>(response.size(), sizeof(config.raw)));
      std::memcpy(config.raw, response.data(), config.rawLength);
      config.connected = 1;
      return lastStatus = STUDICA_PARSEC_OK;
    }
    if (!driver || !context || !context->IsOpen())
      return lastStatus = STUDICA_PARSEC_UNAVAILABLE;
    if (!driver->RequestConfig()) return lastStatus = STUDICA_PARSEC_TIMEOUT;
    context->time.DelayMilliseconds(50);
    std::array<uint8_t, 64> response{};
    if (!driver->GetConfigResponse(response.data(), response.size()))
      return lastStatus = STUDICA_PARSEC_TIMEOUT;
    config.rawLength = static_cast<uint16_t>(response.size());
    if (response[0] != 0) config.connected = 1;
    const auto zones = ZoneCountFromMask(response.data(), response.size());
    if (zones != 0) {
      config.zoneCount = zones;
      config.resolution = ResolutionForZones(zones);
      config.zoneMask = DriverParsec::ParseConfigZoneMask(response.data());
    }
    std::memcpy(config.raw, response.data(), sizeof(config.raw));
    return lastStatus = STUDICA_PARSEC_OK;
  }

  studica::TransportKind kind;
  uint8_t canId = 0;
  unsigned channel = 0;
  std::string usbPath;
  std::shared_ptr<VMXPi> context;
  std::unique_ptr<DriverParsec> driver;
  std::unique_ptr<DriverParsecUsb> driverUsb;
  StudicaParsecSnapshot snapshot{};
  StudicaParsecConfig config{};
  uint64_t sequenceCounter = 0;
  int32_t lastStatus = STUDICA_PARSEC_OK;
  mutable std::mutex mutex;
};

std::mutex g_mutex;
std::unordered_map<StudicaParsecHandle, std::unique_ptr<ParsecInstance>>
    g_instances;
StudicaParsecHandle g_nextHandle = 1;

ParsecInstance* Find(StudicaParsecHandle handle) {
  auto it = g_instances.find(handle);
  return it == g_instances.end() ? nullptr : it->second.get();
}

int32_t MinDistance(const StudicaParsecSnapshot& snapshot, int16_t* distanceOut,
                   uint8_t* validOut) {
  if (!distanceOut || !validOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  int16_t minimum = 0;
  bool found = false;
  for (uint32_t i = 0; i < snapshot.zoneCount && i < 64; ++i) {
    const auto distance = snapshot.distances[i];
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
  auto context = hal::vmx::GetRuntimeContext();
  if (!context || !context->IsOpen()) return STUDICA_PARSEC_UNAVAILABLE;
  const unsigned channel = DriverParsec::GetCANChannel();
  if (!studica::GetStudicaTransportRegistry().ReserveCAN(
          channel, canId, "Studica Parsec CAN"))
    return STUDICA_PARSEC_RESOURCE_CONFLICT;
  try {
    auto instance = std::make_unique<ParsecInstance>(canId, std::move(context));
    std::scoped_lock lock{g_mutex};
    const auto handle = g_nextHandle++;
    g_instances.emplace(handle, std::move(instance));
    *handleOut = handle;
    return STUDICA_PARSEC_OK;
  } catch (...) {
    studica::GetStudicaTransportRegistry().ReleaseCAN(channel, canId);
    return STUDICA_PARSEC_INTERNAL_ERROR;
  }
}

int32_t StudicaParsec_CreateUSB(const char* path,
                                StudicaParsecHandle* handleOut) {
  if (handleOut) *handleOut = 0;
  if (!handleOut || !path || !*path) return STUDICA_PARSEC_INVALID_ARGUMENT;
  const std::string devicePath{path};
  if (!studica::GetStudicaTransportRegistry().ReserveUSB(
          devicePath, "Studica Parsec USB"))
    return STUDICA_PARSEC_RESOURCE_CONFLICT;
  try {
    auto instance = std::make_unique<ParsecInstance>(devicePath);
    if (!instance->driverUsb || !instance->driverUsb->IsOpen()) {
      studica::GetStudicaTransportRegistry().ReleaseUSB(devicePath);
      return STUDICA_PARSEC_UNAVAILABLE;
    }
    std::scoped_lock lock{g_mutex};
    const auto handle = g_nextHandle++;
    g_instances.emplace(handle, std::move(instance));
    *handleOut = handle;
    return STUDICA_PARSEC_OK;
  } catch (...) {
    studica::GetStudicaTransportRegistry().ReleaseUSB(devicePath);
    return STUDICA_PARSEC_INTERNAL_ERROR;
  }
}

void StudicaParsec_Destroy(StudicaParsecHandle handle) {
  std::scoped_lock lock{g_mutex};
  g_instances.erase(handle);
}

int32_t StudicaParsec_ReadZones(StudicaParsecHandle handle,
                                StudicaParsecSnapshot* snapshotOut) {
  if (!snapshotOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  const auto status = instance->Read();
  *snapshotOut = instance->snapshot;
  return status;
}

int32_t StudicaParsec_GetResolution(StudicaParsecHandle handle,
                                    uint32_t* resolutionOut) {
  if (!resolutionOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  std::scoped_lock instanceLock{instance->mutex};
  *resolutionOut = instance->snapshot.resolution;
  return STUDICA_PARSEC_OK;
}

int32_t StudicaParsec_GetZoneCount(StudicaParsecHandle handle,
                                   uint32_t* zoneCountOut) {
  if (!zoneCountOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  std::scoped_lock instanceLock{instance->mutex};
  *zoneCountOut = instance->snapshot.zoneCount;
  return STUDICA_PARSEC_OK;
}

int32_t StudicaParsec_GetZoneDistance(StudicaParsecHandle handle,
                                      uint32_t zone, int16_t* distanceOut) {
  if (!distanceOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  std::scoped_lock instanceLock{instance->mutex};
  if (zone >= instance->snapshot.zoneCount) return STUDICA_PARSEC_INVALID_ARGUMENT;
  *distanceOut = instance->snapshot.distances[zone];
  return STUDICA_PARSEC_OK;
}

int32_t StudicaParsec_GetMinDistance(StudicaParsecHandle handle,
                                     int16_t* distanceOut,
                                     uint8_t* validOut) {
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  std::scoped_lock instanceLock{instance->mutex};
  return MinDistance(instance->snapshot, distanceOut, validOut);
}

int32_t StudicaParsec_GetDistances(StudicaParsecHandle handle,
                                   int16_t* distancesOut,
                                   uint32_t capacity) {
  if (!distancesOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  std::scoped_lock instanceLock{instance->mutex};
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
  const auto status = instance->GetConfig();
  *configOut = instance->config;
  return status;
}

int32_t StudicaParsec_IsConnected(StudicaParsecHandle handle,
                                  uint8_t* connectedOut) {
  if (!connectedOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  std::scoped_lock instanceLock{instance->mutex};
  *connectedOut = instance->snapshot.connected;
  return STUDICA_PARSEC_OK;
}

int32_t StudicaParsec_GetLastStatus(StudicaParsecHandle handle,
                                    int32_t* statusOut) {
  if (!statusOut) return STUDICA_PARSEC_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_PARSEC_NOT_INITIALIZED;
  std::scoped_lock instanceLock{instance->mutex};
  *statusOut = instance->lastStatus;
  return STUDICA_PARSEC_OK;
}

void StudicaParsec_ShutdownAll(void) {
  std::scoped_lock lock{g_mutex};
  g_instances.clear();
}

}  // extern "C"
