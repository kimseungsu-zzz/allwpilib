// Copyright (c) 2026 WPILib contributors.

#include "studica/LightTower.h"

extern "C" {
int32_t StudicaLightTower_Create(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t,
                                 StudicaLightTowerHandle* out) {
  if (!out) return STUDICA_LIGHT_TOWER_INVALID_ARGUMENT;
  *out = 0;
  return STUDICA_LIGHT_TOWER_UNAVAILABLE;
}
void StudicaLightTower_Destroy(StudicaLightTowerHandle) {}
int32_t StudicaLightTower_SetRed(StudicaLightTowerHandle, uint8_t) {
  return STUDICA_LIGHT_TOWER_UNAVAILABLE;
}
int32_t StudicaLightTower_SetYellow(StudicaLightTowerHandle, uint8_t) {
  return STUDICA_LIGHT_TOWER_UNAVAILABLE;
}
int32_t StudicaLightTower_SetGreen(StudicaLightTowerHandle, uint8_t) {
  return STUDICA_LIGHT_TOWER_UNAVAILABLE;
}
int32_t StudicaLightTower_SetBuzzer(StudicaLightTowerHandle, uint8_t) {
  return STUDICA_LIGHT_TOWER_UNAVAILABLE;
}
int32_t StudicaLightTower_SetSolid(StudicaLightTowerHandle) {
  return STUDICA_LIGHT_TOWER_UNAVAILABLE;
}
int32_t StudicaLightTower_SetBlink(StudicaLightTowerHandle) {
  return STUDICA_LIGHT_TOWER_UNAVAILABLE;
}
int32_t StudicaLightTower_Off(StudicaLightTowerHandle) {
  return STUDICA_LIGHT_TOWER_UNAVAILABLE;
}
int32_t StudicaLightTower_IsAvailable(StudicaLightTowerHandle, uint8_t* out) {
  if (!out) return STUDICA_LIGHT_TOWER_INVALID_ARGUMENT;
  *out = 0;
  return STUDICA_LIGHT_TOWER_OK;
}
void StudicaLightTower_ShutdownAll(void) {}
}
