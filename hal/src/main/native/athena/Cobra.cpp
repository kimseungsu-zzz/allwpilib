// Copyright (c) 2026 WPILib contributors.

#include "studica/Cobra.h"

#include <cmath>

extern "C" {
int32_t StudicaCobra_Create(double referenceVoltage,
                            StudicaCobraHandle* handleOut) {
  if (!handleOut || !std::isfinite(referenceVoltage) || referenceVoltage <= 0.0) {
    return STUDICA_COBRA_INVALID_ARGUMENT;
  }
  *handleOut = 0;
  return STUDICA_COBRA_UNAVAILABLE;
}
void StudicaCobra_Destroy(StudicaCobraHandle) {}
int32_t StudicaCobra_GetRaw(StudicaCobraHandle, uint8_t, int32_t* out) {
  if (!out) return STUDICA_COBRA_INVALID_ARGUMENT;
  return STUDICA_COBRA_UNAVAILABLE;
}
int32_t StudicaCobra_GetVoltage(StudicaCobraHandle, uint8_t, double* out) {
  if (!out) return STUDICA_COBRA_INVALID_ARGUMENT;
  return STUDICA_COBRA_UNAVAILABLE;
}
int32_t StudicaCobra_GetChannelCount(StudicaCobraHandle, uint8_t* out) {
  if (!out) return STUDICA_COBRA_INVALID_ARGUMENT;
  return STUDICA_COBRA_UNAVAILABLE;
}
int32_t StudicaCobra_GetReferenceVoltage(StudicaCobraHandle, double* out) {
  if (!out) return STUDICA_COBRA_INVALID_ARGUMENT;
  return STUDICA_COBRA_UNAVAILABLE;
}
int32_t StudicaCobra_IsAvailable(StudicaCobraHandle, uint8_t* out) {
  if (!out) return STUDICA_COBRA_INVALID_ARGUMENT;
  *out = 0;
  return STUDICA_COBRA_OK;
}
void StudicaCobra_ShutdownAll(void) {}
}
