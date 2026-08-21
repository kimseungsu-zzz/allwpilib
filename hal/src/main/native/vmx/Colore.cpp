// Copyright (c) 2026 WPILib contributors.

#include "studica/Colore.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "VMXRuntime.h"
#include "colore.hpp"
#include "colore_usb.hpp"
#include "studica/ColorMatching.h"
#include "studica/Transport.h"

namespace {

using DriverColore = studica_driver::Colore;
using DriverColoreUsb = studica_driver::ColoreUsb;

float Clamp01(float value) {
  if (!std::isfinite(value)) return 0.0F;
  return std::clamp(value, 0.0F, 1.0F);
}

void XYZToSRGB(float x, float y, float z, float& red, float& green,
              float& blue) {
  red = Clamp01(3.2406F * x - 1.5372F * y - 0.4986F * z);
  green = Clamp01(-0.9689F * x + 1.8758F * y + 0.0415F * z);
  blue = Clamp01(0.0557F * x - 0.2040F * y + 1.0570F * z);
}

void SetChromaticity(StudicaColoreSnapshot& snapshot) {
  const float sum = snapshot.x + snapshot.y + snapshot.z;
  if (std::isfinite(sum) && sum > 0.0F) {
    snapshot.chromaticityX = snapshot.x / sum;
    snapshot.chromaticityY = snapshot.y / sum;
  } else {
    snapshot.chromaticityX = 0.0F;
    snapshot.chromaticityY = 0.0F;
  }
}

struct ColoreInstance final {
  explicit ColoreInstance(uint8_t canId, std::shared_ptr<VMXPi> context)
      : kind{studica::TransportKind::kCAN}, canId{canId},
        context{std::move(context)} {
    driver = std::make_unique<DriverColore>(canId, this->context);
    snapshot.structSize = sizeof(snapshot);
    snapshot.abiVersion = STUDICA_COLORE_ABI_VERSION;
    snapshot.transport = static_cast<uint8_t>(kind);
    snapshot.connected = driver && this->context && this->context->IsOpen();
    config.structSize = sizeof(config);
    config.abiVersion = STUDICA_COLORE_ABI_VERSION;
    config.transport = static_cast<uint32_t>(kind);
    config.canId = canId;
  }

  explicit ColoreInstance(std::string path)
      : kind{studica::TransportKind::kUSB}, usbPath{std::move(path)} {
    driverUsb = std::make_unique<DriverColoreUsb>(usbPath);
    snapshot.structSize = sizeof(snapshot);
    snapshot.abiVersion = STUDICA_COLORE_ABI_VERSION;
    snapshot.transport = static_cast<uint8_t>(kind);
    snapshot.connected = driverUsb && driverUsb->IsOpen();
    config.structSize = sizeof(config);
    config.abiVersion = STUDICA_COLORE_ABI_VERSION;
    config.transport = static_cast<uint32_t>(kind);
    if (driverUsb && driverUsb->IsOpen()) {
      driverUsb->ConfigureStreaming("xyz", 100);
    }
  }

  ~ColoreInstance() {
    if (kind == studica::TransportKind::kUSB) {
      studica::GetStudicaTransportRegistry().ReleaseUSB(usbPath);
    } else {
      studica::GetStudicaTransportRegistry().ReleaseCAN(
          DriverColore::GetCANChannel(), canId);
    }
  }

  int32_t Read() noexcept {
    std::scoped_lock lock{mutex};
    if (kind == studica::TransportKind::kUSB) return ReadUSB();
    return ReadCAN();
  }

  int32_t ReadUSB() noexcept {
    if (!driverUsb || !driverUsb->IsOpen()) {
      snapshot.connected = 0;
      lastStatus = STUDICA_COLORE_UNAVAILABLE;
      return lastStatus;
    }
    DriverColoreUsb::Sample sample;
    if (!driverUsb->ReadLatest(&sample)) {
      snapshot.connected = 1;
      lastStatus = STUDICA_COLORE_TIMEOUT;
      return lastStatus;
    }
    if (sample.has_xyz) {
      snapshot.x = sample.x;
      snapshot.y = sample.y;
      snapshot.z = sample.z;
    }
    if (sample.has_srgb) {
      snapshot.red = static_cast<float>(sample.r) / 255.0F;
      snapshot.green = static_cast<float>(sample.g) / 255.0F;
      snapshot.blue = static_cast<float>(sample.b) / 255.0F;
    } else {
      XYZToSRGB(snapshot.x, snapshot.y, snapshot.z, snapshot.red,
                snapshot.green, snapshot.blue);
    }
    SetChromaticity(snapshot);
    snapshot.sequence = sample.seq;
    snapshot.connected = 1;
    snapshot.valid = sample.has_xyz || sample.has_srgb;
    lastStatus = snapshot.valid ? STUDICA_COLORE_OK : STUDICA_COLORE_TIMEOUT;
    return lastStatus;
  }

  int32_t ReadCAN() noexcept {
    if (!driver || !context || !context->IsOpen()) {
      snapshot.connected = 0;
      lastStatus = STUDICA_COLORE_UNAVAILABLE;
      return lastStatus;
    }
    if (!configured) {
      driver->SetColorFormat(DriverColore::ColorFormat::XYZ);
      configured = true;
    }
    std::array<uint8_t, 8> frame{};
    const int count = driver->Read(COLORE_CAN_TELEM_XYZ, frame.data(), frame.size());
    if (count < 8) {
      snapshot.connected = 1;
      lastStatus = STUDICA_COLORE_TIMEOUT;
      return lastStatus;
    }
    const auto read16 = [&frame](int offset) {
      const auto raw = static_cast<uint16_t>(frame[offset]) |
                       (static_cast<uint16_t>(frame[offset + 1]) << 8);
      return static_cast<int16_t>(raw);
    };
    snapshot.x = static_cast<float>(read16(2)) / 10000.0F;
    snapshot.y = static_cast<float>(read16(4)) / 10000.0F;
    snapshot.z = static_cast<float>(read16(6)) / 10000.0F;
    XYZToSRGB(snapshot.x, snapshot.y, snapshot.z, snapshot.red,
              snapshot.green, snapshot.blue);
    SetChromaticity(snapshot);
    snapshot.sequence = static_cast<uint64_t>(
        static_cast<uint16_t>(frame[0]) |
        (static_cast<uint16_t>(frame[1]) << 8));
    snapshot.connected = 1;
    snapshot.valid = 1;
    lastStatus = STUDICA_COLORE_OK;
    return lastStatus;
  }

  int32_t SetBrightness(int32_t percent) noexcept {
    if (percent < 0 || percent > 100) {
      lastStatus = STUDICA_COLORE_INVALID_ARGUMENT;
      return lastStatus;
    }
    std::scoped_lock lock{mutex};
    bool ok = false;
    if (kind == studica::TransportKind::kUSB) {
      ok = driverUsb && driverUsb->SendCommand(
                             "BRIGHTNESS," + std::to_string(percent));
    } else {
      ok = driver && driver->SetBrightness(static_cast<uint8_t>(percent));
    }
    if (ok) {
      brightness = static_cast<uint8_t>(percent);
      config.brightness = brightness;
      lastStatus = STUDICA_COLORE_OK;
    } else {
      lastStatus = STUDICA_COLORE_UNAVAILABLE;
    }
    return lastStatus;
  }

  int32_t GetConfig() noexcept {
    std::scoped_lock lock{mutex};
    if (kind == studica::TransportKind::kUSB) {
      if (!driverUsb || !driverUsb->IsOpen()) return STUDICA_COLORE_UNAVAILABLE;
      std::string response;
      if (!driverUsb->RequestConfig(&response)) return STUDICA_COLORE_TIMEOUT;
      config.rawLength = static_cast<uint32_t>(
          std::min<std::size_t>(response.size(), sizeof(config.raw)));
      std::memcpy(config.raw, response.data(), config.rawLength);
      return STUDICA_COLORE_OK;
    }
    if (!driver || !context || !context->IsOpen())
      return STUDICA_COLORE_UNAVAILABLE;
    uint32_t value = 0;
    if (driver->GetConfig(4, value)) brightness = static_cast<uint8_t>(value);
    config.brightness = brightness;
    config.colorFormat = 4;  // XYZ, selected by the adapter.
    return STUDICA_COLORE_OK;
  }

  studica::TransportKind kind;
  uint8_t canId = 0;
  std::string usbPath;
  std::shared_ptr<VMXPi> context;
  std::unique_ptr<DriverColore> driver;
  std::unique_ptr<DriverColoreUsb> driverUsb;
  studica::detail::ColorMatcher matcher;
  StudicaColoreSnapshot snapshot{};
  StudicaColoreConfig config{};
  uint8_t brightness = 0;
  bool configured = false;
  int32_t lastStatus = STUDICA_COLORE_OK;
  mutable std::mutex mutex;
};

std::mutex g_mutex;
std::unordered_map<StudicaColoreHandle, std::unique_ptr<ColoreInstance>>
    g_instances;
StudicaColoreHandle g_nextHandle = 1;

ColoreInstance* Find(StudicaColoreHandle handle) {
  auto it = g_instances.find(handle);
  return it == g_instances.end() ? nullptr : it->second.get();
}

int32_t GetComponent(ColoreInstance* instance, float* out, float value) {
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
  auto context = hal::vmx::GetRuntimeContext();
  if (!context || !context->IsOpen()) return STUDICA_COLORE_UNAVAILABLE;
  const auto channel = DriverColore::GetCANChannel();
  if (!studica::GetStudicaTransportRegistry().ReserveCAN(
          channel, canId, "Studica Colore CAN"))
    return STUDICA_COLORE_RESOURCE_CONFLICT;
  try {
    auto instance = std::make_unique<ColoreInstance>(canId, std::move(context));
    std::scoped_lock lock{g_mutex};
    const auto handle = g_nextHandle++;
    g_instances.emplace(handle, std::move(instance));
    *handleOut = handle;
    return STUDICA_COLORE_OK;
  } catch (...) {
    studica::GetStudicaTransportRegistry().ReleaseCAN(channel, canId);
    return STUDICA_COLORE_INTERNAL_ERROR;
  }
}

int32_t StudicaColore_CreateUSB(const char* path,
                                StudicaColoreHandle* handleOut) {
  if (handleOut) *handleOut = 0;
  if (!handleOut || !path || !*path) return STUDICA_COLORE_INVALID_ARGUMENT;
  const std::string devicePath{path};
  if (!studica::GetStudicaTransportRegistry().ReserveUSB(
          devicePath, "Studica Colore USB"))
    return STUDICA_COLORE_RESOURCE_CONFLICT;
  try {
    auto instance = std::make_unique<ColoreInstance>(devicePath);
    if (!instance->driverUsb || !instance->driverUsb->IsOpen()) {
      studica::GetStudicaTransportRegistry().ReleaseUSB(devicePath);
      return STUDICA_COLORE_UNAVAILABLE;
    }
    std::scoped_lock lock{g_mutex};
    const auto handle = g_nextHandle++;
    g_instances.emplace(handle, std::move(instance));
    *handleOut = handle;
    return STUDICA_COLORE_OK;
  } catch (...) {
    studica::GetStudicaTransportRegistry().ReleaseUSB(devicePath);
    return STUDICA_COLORE_INTERNAL_ERROR;
  }
}

void StudicaColore_Destroy(StudicaColoreHandle handle) {
  std::scoped_lock lock{g_mutex};
  g_instances.erase(handle);
}

int32_t StudicaColore_Read(StudicaColoreHandle handle,
                           StudicaColoreSnapshot* snapshotOut) {
  if (!snapshotOut) return STUDICA_COLORE_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  const auto status = instance->Read();
  *snapshotOut = instance->snapshot;
  return status;
}

#define STUDICA_COLORE_COMPONENT(Name, field)                                  \
  int32_t StudicaColore_Get##Name(StudicaColoreHandle handle, float* out) {     \
    std::scoped_lock lock{g_mutex};                                             \
    auto* instance = Find(handle);                                              \
    return GetComponent(instance, out, instance ? instance->snapshot.field    \
                                                 : 0.0F);                       \
  }

STUDICA_COLORE_COMPONENT(Red, red)
STUDICA_COLORE_COMPONENT(Green, green)
STUDICA_COLORE_COMPONENT(Blue, blue)
STUDICA_COLORE_COMPONENT(X, x)
STUDICA_COLORE_COMPONENT(Y, y)
STUDICA_COLORE_COMPONENT(Z, z)

#undef STUDICA_COLORE_COMPONENT

int32_t StudicaColore_SetBrightness(StudicaColoreHandle handle,
                                    int32_t percent) {
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COLORE_NOT_INITIALIZED;
  return instance->SetBrightness(percent);
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
  const auto status = instance->GetConfig();
  *configOut = instance->config;
  return status;
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
  *statusOut = instance->lastStatus;
  return STUDICA_COLORE_OK;
}

void StudicaColore_ShutdownAll(void) {
  std::scoped_lock lock{g_mutex};
  g_instances.clear();
}

}  // extern "C"
