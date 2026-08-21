// Copyright (c) 2026 WPILib contributors.

#include "studica/Parsec.h"

#include <algorithm>
#include <utility>

namespace studica {

Parsec::Parsec(uint8_t canId) noexcept {
  m_lastStatus = StudicaParsec_CreateCAN(canId, &m_handle);
}

Parsec::Parsec(std::string usbPath) noexcept {
  m_lastStatus = StudicaParsec_CreateUSB(usbPath.c_str(), &m_handle);
}

Parsec::~Parsec() { StudicaParsec_Destroy(m_handle); }

Parsec::Parsec(Parsec&& other) noexcept
    : m_handle{std::exchange(other.m_handle, 0)},
      m_lastStatus{std::exchange(other.m_lastStatus,
                                 STUDICA_PARSEC_NOT_INITIALIZED)} {}

Parsec& Parsec::operator=(Parsec&& other) noexcept {
  if (this == &other) return *this;
  StudicaParsec_Destroy(m_handle);
  m_handle = std::exchange(other.m_handle, 0);
  m_lastStatus = std::exchange(other.m_lastStatus,
                               STUDICA_PARSEC_NOT_INITIALIZED);
  return *this;
}

uint32_t Parsec::GetResolution() const noexcept {
  uint32_t value = 0;
  m_lastStatus = StudicaParsec_GetResolution(m_handle, &value);
  return m_lastStatus == STUDICA_PARSEC_OK ? value : 0;
}

uint32_t Parsec::GetZoneCount() const noexcept {
  uint32_t value = 0;
  m_lastStatus = StudicaParsec_GetZoneCount(m_handle, &value);
  return m_lastStatus == STUDICA_PARSEC_OK ? value : 0;
}

int16_t Parsec::GetDistance(uint32_t zone) const noexcept {
  int16_t value = -2;
  m_lastStatus = StudicaParsec_GetZoneDistance(m_handle, zone, &value);
  return m_lastStatus == STUDICA_PARSEC_OK ? value : -2;
}

std::array<int16_t, STUDICA_PARSEC_MAX_ZONES> Parsec::GetDistances()
    const noexcept {
  std::array<int16_t, STUDICA_PARSEC_MAX_ZONES> values{};
  std::fill(values.begin(), values.end(), static_cast<int16_t>(-2));
  m_lastStatus = StudicaParsec_GetDistances(m_handle, values.data(), values.size());
  if (m_lastStatus >= 0) {
    m_lastStatus = STUDICA_PARSEC_OK;
  }
  return values;
}

bool Parsec::GetMinDistance(int16_t* distanceOut) const noexcept {
  uint8_t valid = 0;
  m_lastStatus = StudicaParsec_GetMinDistance(m_handle, distanceOut, &valid);
  return m_lastStatus == STUDICA_PARSEC_OK && valid != 0;
}

bool Parsec::Read() noexcept {
  StudicaParsecSnapshot snapshot{};
  m_lastStatus = StudicaParsec_ReadZones(m_handle, &snapshot);
  return m_lastStatus == STUDICA_PARSEC_OK;
}

bool Parsec::GetConfig(StudicaParsecConfig* configOut) const noexcept {
  m_lastStatus = StudicaParsec_GetConfig(m_handle, configOut);
  return m_lastStatus == STUDICA_PARSEC_OK;
}

bool Parsec::IsConnected() const noexcept {
  uint8_t value = 0;
  m_lastStatus = StudicaParsec_IsConnected(m_handle, &value);
  return m_lastStatus == STUDICA_PARSEC_OK && value != 0;
}

int32_t Parsec::GetLastStatus() const noexcept { return m_lastStatus; }

}  // namespace studica
