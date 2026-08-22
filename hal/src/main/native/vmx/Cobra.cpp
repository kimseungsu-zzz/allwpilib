// Copyright (c) 2026 WPILib contributors.

#include "studica/Cobra.h"

#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "DigitalChannelRegistry.h"
#include "VMXChannelCapabilities.h"
#include "VMXRuntime.h"
#include "cobra.hpp"

namespace {

struct CobraInstance {
  CobraInstance(double referenceVoltage, std::shared_ptr<VMXPi> context)
      : referenceVoltage{referenceVoltage},
        driver{std::make_unique<studica_driver::Cobra>(
            std::move(context), static_cast<int>(referenceVoltage))} {}

  ~CobraInstance() {
    auto& registry = hal::vmx::GetDigitalChannelRegistry();
    auto map = hal::vmx::GetVMXCommDIOChannelMap();
    registry.ReleaseShared(map.i2cSDA, hal::vmx::DigitalChannelOwner::kCobra);
    registry.ReleaseShared(map.i2cSCL, hal::vmx::DigitalChannelOwner::kCobra);
  }

  double referenceVoltage;
  std::unique_ptr<studica_driver::Cobra> driver;
};

std::mutex g_mutex;
std::unordered_map<StudicaCobraHandle, std::unique_ptr<CobraInstance>>
    g_instances;
StudicaCobraHandle g_nextHandle = 1;

CobraInstance* Find(StudicaCobraHandle handle) {
  auto it = g_instances.find(handle);
  return it == g_instances.end() ? nullptr : it->second.get();
}

bool ValidReferenceVoltage(double value) {
  return std::isfinite(value) && value > 0.0 && value <= 10.0 &&
         std::floor(value) == value;
}

bool ReserveI2CBus() {
  auto map = hal::vmx::GetVMXCommDIOChannelMap();
  if (!map.valid || !map.i2cValid || map.i2cSDA == map.i2cSCL) {
    return false;
  }
  auto& registry = hal::vmx::GetDigitalChannelRegistry();
  auto sda = registry.ReserveShared(
      map.i2cSDA, hal::vmx::DigitalChannelOwner::kCobra,
      "Studica Cobra I2C SDA", hal::vmx::DigitalChannelOwner::kI2C);
  if (!sda.reserved) {
    return false;
  }
  auto scl = registry.ReserveShared(
      map.i2cSCL, hal::vmx::DigitalChannelOwner::kCobra,
      "Studica Cobra I2C SCL", hal::vmx::DigitalChannelOwner::kI2C);
  if (!scl.reserved) {
    registry.ReleaseShared(map.i2cSDA,
                           hal::vmx::DigitalChannelOwner::kCobra);
    return false;
  }
  return true;
}

}  // namespace

extern "C" {

int32_t StudicaCobra_Create(double referenceVoltage,
                           StudicaCobraHandle* handleOut) {
  if (!handleOut || !ValidReferenceVoltage(referenceVoltage)) {
    return STUDICA_COBRA_INVALID_ARGUMENT;
  }
  auto context = hal::vmx::GetRuntimeContext();
  if (!context || !context->IsOpen()) {
    return STUDICA_COBRA_UNAVAILABLE;
  }
  if (!ReserveI2CBus()) {
    return STUDICA_COBRA_RESOURCE_CONFLICT;
  }
  try {
    auto instance = std::make_unique<CobraInstance>(referenceVoltage,
                                                     std::move(context));
    const auto handle = g_nextHandle++;
    std::scoped_lock lock{g_mutex};
    g_instances.emplace(handle, std::move(instance));
    *handleOut = handle;
    return STUDICA_COBRA_OK;
  } catch (...) {
    auto map = hal::vmx::GetVMXCommDIOChannelMap();
    auto& registry = hal::vmx::GetDigitalChannelRegistry();
    registry.ReleaseShared(map.i2cSDA,
                           hal::vmx::DigitalChannelOwner::kCobra);
    registry.ReleaseShared(map.i2cSCL,
                           hal::vmx::DigitalChannelOwner::kCobra);
    return STUDICA_COBRA_INTERNAL_ERROR;
  }
}

void StudicaCobra_Destroy(StudicaCobraHandle handle) {
  std::scoped_lock lock{g_mutex};
  g_instances.erase(handle);
}

int32_t StudicaCobra_GetRaw(StudicaCobraHandle handle, uint8_t channel,
                            int32_t* rawOut) {
  if (!rawOut || channel >= STUDICA_COBRA_CHANNEL_COUNT) {
    return STUDICA_COBRA_INVALID_ARGUMENT;
  }
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance || !instance->driver) return STUDICA_COBRA_NOT_INITIALIZED;
  try {
    const auto raw = instance->driver->GetRawValue(channel);
    if (raw < 0) return STUDICA_COBRA_UNAVAILABLE;
    *rawOut = raw;
    return STUDICA_COBRA_OK;
  } catch (...) {
    return STUDICA_COBRA_INTERNAL_ERROR;
  }
}

int32_t StudicaCobra_GetVoltage(StudicaCobraHandle handle, uint8_t channel,
                                double* voltageOut) {
  if (!voltageOut || channel >= STUDICA_COBRA_CHANNEL_COUNT) {
    return STUDICA_COBRA_INVALID_ARGUMENT;
  }
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance || !instance->driver) return STUDICA_COBRA_NOT_INITIALIZED;
  try {
    const auto raw = instance->driver->GetRawValue(channel);
    if (raw < 0) return STUDICA_COBRA_UNAVAILABLE;
    *voltageOut = instance->driver->GetVoltage(channel);
    return std::isfinite(*voltageOut) ? STUDICA_COBRA_OK
                                      : STUDICA_COBRA_UNAVAILABLE;
  } catch (...) {
    return STUDICA_COBRA_INTERNAL_ERROR;
  }
}

int32_t StudicaCobra_GetChannelCount(StudicaCobraHandle handle,
                                     uint8_t* countOut) {
  if (!countOut) return STUDICA_COBRA_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  if (!Find(handle)) return STUDICA_COBRA_NOT_INITIALIZED;
  *countOut = STUDICA_COBRA_CHANNEL_COUNT;
  return STUDICA_COBRA_OK;
}

int32_t StudicaCobra_GetReferenceVoltage(StudicaCobraHandle handle,
                                         double* voltageOut) {
  if (!voltageOut) return STUDICA_COBRA_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto* instance = Find(handle);
  if (!instance) return STUDICA_COBRA_NOT_INITIALIZED;
  *voltageOut = instance->referenceVoltage;
  return STUDICA_COBRA_OK;
}

int32_t StudicaCobra_IsAvailable(StudicaCobraHandle handle,
                                 uint8_t* availableOut) {
  if (!availableOut) return STUDICA_COBRA_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  *availableOut = Find(handle) ? 1 : 0;
  return STUDICA_COBRA_OK;
}

void StudicaCobra_ShutdownAll(void) {
  std::scoped_lock lock{g_mutex};
  g_instances.clear();
}

}  // extern "C"
