// Copyright (c) 2026 WPILib contributors.

#include "studica/LightTower.h"

#include <utility>

namespace studica {

LightTower::LightTower(uint8_t continuous, uint8_t red, uint8_t green,
                       uint8_t yellow, uint8_t buzzer) noexcept {
  m_lastStatus = StudicaLightTower_Create(continuous, red, green, yellow,
                                          buzzer, &m_handle);
}

LightTower::~LightTower() { StudicaLightTower_Destroy(m_handle); }

LightTower::LightTower(LightTower&& other) noexcept
    : m_handle{std::exchange(other.m_handle, 0)},
      m_lastStatus{std::exchange(other.m_lastStatus,
                                 STUDICA_LIGHT_TOWER_NOT_INITIALIZED)} {}

LightTower& LightTower::operator=(LightTower&& other) noexcept {
  if (this == &other) return *this;
  StudicaLightTower_Destroy(m_handle);
  m_handle = std::exchange(other.m_handle, 0);
  m_lastStatus = std::exchange(other.m_lastStatus,
                               STUDICA_LIGHT_TOWER_NOT_INITIALIZED);
  return *this;
}

bool LightTower::SetRed(bool enabled) noexcept {
  m_lastStatus = StudicaLightTower_SetRed(m_handle, enabled);
  return m_lastStatus == STUDICA_LIGHT_TOWER_OK;
}
bool LightTower::SetYellow(bool enabled) noexcept {
  m_lastStatus = StudicaLightTower_SetYellow(m_handle, enabled);
  return m_lastStatus == STUDICA_LIGHT_TOWER_OK;
}
bool LightTower::SetGreen(bool enabled) noexcept {
  m_lastStatus = StudicaLightTower_SetGreen(m_handle, enabled);
  return m_lastStatus == STUDICA_LIGHT_TOWER_OK;
}
bool LightTower::SetBuzzer(bool enabled) noexcept {
  m_lastStatus = StudicaLightTower_SetBuzzer(m_handle, enabled);
  return m_lastStatus == STUDICA_LIGHT_TOWER_OK;
}
bool LightTower::SetSolid() noexcept {
  m_lastStatus = StudicaLightTower_SetSolid(m_handle);
  return m_lastStatus == STUDICA_LIGHT_TOWER_OK;
}
bool LightTower::SetBlink() noexcept {
  m_lastStatus = StudicaLightTower_SetBlink(m_handle);
  return m_lastStatus == STUDICA_LIGHT_TOWER_OK;
}
bool LightTower::Off() noexcept {
  m_lastStatus = StudicaLightTower_Off(m_handle);
  return m_lastStatus == STUDICA_LIGHT_TOWER_OK;
}
bool LightTower::IsAvailable() const noexcept {
  uint8_t value = 0;
  m_lastStatus = StudicaLightTower_IsAvailable(m_handle, &value);
  return m_lastStatus == STUDICA_LIGHT_TOWER_OK && value != 0;
}
int32_t LightTower::GetLastStatus() const noexcept { return m_lastStatus; }

}  // namespace studica
