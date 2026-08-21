// Copyright (c) 2026 WPILib contributors.

#include "studica/Cobra.h"

#include <cmath>
#include <mutex>
#include <unordered_map>

namespace {
struct SimCobra {
  double referenceVoltage;
};
std::mutex g_mutex;
std::unordered_map<StudicaCobraHandle, SimCobra> g_instances;
StudicaCobraHandle g_nextHandle = 1;
}

extern "C" {

int32_t StudicaCobra_Create(double referenceVoltage,
                            StudicaCobraHandle* handleOut) {
  if (!handleOut || !std::isfinite(referenceVoltage) || referenceVoltage <= 0.0 ||
      referenceVoltage > 10.0 || std::floor(referenceVoltage) != referenceVoltage) {
    return STUDICA_COBRA_INVALID_ARGUMENT;
  }
  std::scoped_lock lock{g_mutex};
  const auto handle = g_nextHandle++;
  g_instances.emplace(handle, SimCobra{referenceVoltage});
  *handleOut = handle;
  return STUDICA_COBRA_OK;
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
  return g_instances.count(handle) ? STUDICA_COBRA_UNAVAILABLE
                                    : STUDICA_COBRA_NOT_INITIALIZED;
}

int32_t StudicaCobra_GetVoltage(StudicaCobraHandle handle, uint8_t channel,
                                double* voltageOut) {
  if (!voltageOut || channel >= STUDICA_COBRA_CHANNEL_COUNT) {
    return STUDICA_COBRA_INVALID_ARGUMENT;
  }
  std::scoped_lock lock{g_mutex};
  return g_instances.count(handle) ? STUDICA_COBRA_UNAVAILABLE
                                    : STUDICA_COBRA_NOT_INITIALIZED;
}

int32_t StudicaCobra_GetChannelCount(StudicaCobraHandle handle,
                                     uint8_t* countOut) {
  if (!countOut) return STUDICA_COBRA_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  if (!g_instances.count(handle)) return STUDICA_COBRA_NOT_INITIALIZED;
  *countOut = STUDICA_COBRA_CHANNEL_COUNT;
  return STUDICA_COBRA_OK;
}

int32_t StudicaCobra_GetReferenceVoltage(StudicaCobraHandle handle,
                                         double* voltageOut) {
  if (!voltageOut) return STUDICA_COBRA_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  auto it = g_instances.find(handle);
  if (it == g_instances.end()) return STUDICA_COBRA_NOT_INITIALIZED;
  *voltageOut = it->second.referenceVoltage;
  return STUDICA_COBRA_OK;
}

int32_t StudicaCobra_IsAvailable(StudicaCobraHandle handle,
                                 uint8_t* availableOut) {
  if (!availableOut) return STUDICA_COBRA_INVALID_ARGUMENT;
  std::scoped_lock lock{g_mutex};
  *availableOut = 0;
  return g_instances.count(handle) ? STUDICA_COBRA_OK
                                    : STUDICA_COBRA_INVALID_ARGUMENT;
}

void StudicaCobra_ShutdownAll(void) {
  std::scoped_lock lock{g_mutex};
  g_instances.clear();
}

}  // extern "C"
