// Copyright (c) 2026 WPILib contributors.

#include "studica/Colore.h"

extern "C" {

int32_t StudicaColore_CreateCAN(uint8_t, StudicaColoreHandle* handleOut) {
  if (handleOut) *handleOut = 0;
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_CreateUSB(const char*, StudicaColoreHandle* handleOut) {
  if (handleOut) *handleOut = 0;
  return STUDICA_COLORE_UNSUPPORTED;
}
void StudicaColore_Destroy(StudicaColoreHandle) {}
int32_t StudicaColore_Read(StudicaColoreHandle, StudicaColoreSnapshot*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_GetRed(StudicaColoreHandle, float*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_GetGreen(StudicaColoreHandle, float*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_GetBlue(StudicaColoreHandle, float*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_GetX(StudicaColoreHandle, float*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_GetY(StudicaColoreHandle, float*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_GetZ(StudicaColoreHandle, float*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_SetBrightness(StudicaColoreHandle, int32_t) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_GetBrightness(StudicaColoreHandle, uint8_t*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_GetConfig(StudicaColoreHandle, StudicaColoreConfig*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_LearnColor(StudicaColoreHandle, const char*, float) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_SetReference(StudicaColoreHandle, const char*, float,
                                   float, float) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_GetLearnedReference(StudicaColoreHandle, const char*,
                                          StudicaColoreMatchResult*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_Match(StudicaColoreHandle, StudicaColoreMatchResult*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_IsConnected(StudicaColoreHandle, uint8_t*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
int32_t StudicaColore_GetLastStatus(StudicaColoreHandle, int32_t*) {
  return STUDICA_COLORE_UNSUPPORTED;
}
void StudicaColore_ShutdownAll(void) {}
int32_t StudicaColore_SetMockSnapshot(
    StudicaColoreHandle, const StudicaColoreSnapshot*) {
  return STUDICA_COLORE_UNSUPPORTED;
}

}  // extern "C"
