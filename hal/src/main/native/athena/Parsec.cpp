// Copyright (c) 2026 WPILib contributors.

#include "studica/Parsec.h"

extern "C" {

int32_t StudicaParsec_CreateCAN(uint8_t, StudicaParsecHandle* handleOut) {
  if (handleOut) *handleOut = 0;
  return STUDICA_PARSEC_UNSUPPORTED;
}
int32_t StudicaParsec_CreateUSB(const char*, StudicaParsecHandle* handleOut) {
  if (handleOut) *handleOut = 0;
  return STUDICA_PARSEC_UNSUPPORTED;
}
void StudicaParsec_Destroy(StudicaParsecHandle) {}
int32_t StudicaParsec_ReadZones(StudicaParsecHandle,
                                StudicaParsecSnapshot*) {
  return STUDICA_PARSEC_UNSUPPORTED;
}
int32_t StudicaParsec_GetResolution(StudicaParsecHandle, uint32_t*) {
  return STUDICA_PARSEC_UNSUPPORTED;
}
int32_t StudicaParsec_GetZoneCount(StudicaParsecHandle, uint32_t*) {
  return STUDICA_PARSEC_UNSUPPORTED;
}
int32_t StudicaParsec_GetZoneDistance(StudicaParsecHandle, uint32_t, int16_t*) {
  return STUDICA_PARSEC_UNSUPPORTED;
}
int32_t StudicaParsec_GetMinDistance(StudicaParsecHandle, int16_t*, uint8_t*) {
  return STUDICA_PARSEC_UNSUPPORTED;
}
int32_t StudicaParsec_GetDistances(StudicaParsecHandle, int16_t*, uint32_t) {
  return STUDICA_PARSEC_UNSUPPORTED;
}
int32_t StudicaParsec_GetConfig(StudicaParsecHandle, StudicaParsecConfig*) {
  return STUDICA_PARSEC_UNSUPPORTED;
}
int32_t StudicaParsec_IsConnected(StudicaParsecHandle, uint8_t*) {
  return STUDICA_PARSEC_UNSUPPORTED;
}
int32_t StudicaParsec_GetLastStatus(StudicaParsecHandle, int32_t*) {
  return STUDICA_PARSEC_UNSUPPORTED;
}
void StudicaParsec_ShutdownAll(void) {}
int32_t StudicaParsec_SetMockSnapshot(
    StudicaParsecHandle, const StudicaParsecSnapshot*) {
  return STUDICA_PARSEC_UNSUPPORTED;
}

}  // extern "C"
