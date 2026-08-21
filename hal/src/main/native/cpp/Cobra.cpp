// Copyright (c) 2026 WPILib contributors.

#include "studica/Cobra.h"

#include <utility>

namespace studica {

Cobra::Cobra(double referenceVoltage) noexcept {
  m_lastStatus = StudicaCobra_Create(referenceVoltage, &m_handle);
}

Cobra::~Cobra() { StudicaCobra_Destroy(m_handle); }

Cobra::Cobra(Cobra&& other) noexcept
    : m_handle{std::exchange(other.m_handle, 0)},
      m_lastStatus{std::exchange(other.m_lastStatus,
                                 STUDICA_COBRA_NOT_INITIALIZED)} {}

Cobra& Cobra::operator=(Cobra&& other) noexcept {
  if (this == &other) return *this;
  StudicaCobra_Destroy(m_handle);
  m_handle = std::exchange(other.m_handle, 0);
  m_lastStatus = std::exchange(other.m_lastStatus,
                               STUDICA_COBRA_NOT_INITIALIZED);
  return *this;
}

int32_t Cobra::GetRaw(uint8_t channel) const noexcept {
  int32_t value = 0;
  m_lastStatus = StudicaCobra_GetRaw(m_handle, channel, &value);
  return m_lastStatus == STUDICA_COBRA_OK ? value : -1;
}

double Cobra::GetVoltage(uint8_t channel) const noexcept {
  double value = 0.0;
  m_lastStatus = StudicaCobra_GetVoltage(m_handle, channel, &value);
  return m_lastStatus == STUDICA_COBRA_OK ? value : 0.0;
}

uint8_t Cobra::GetChannelCount() const noexcept {
  uint8_t value = 0;
  m_lastStatus = StudicaCobra_GetChannelCount(m_handle, &value);
  return m_lastStatus == STUDICA_COBRA_OK ? value : 0;
}

double Cobra::GetReferenceVoltage() const noexcept {
  double value = 0.0;
  m_lastStatus = StudicaCobra_GetReferenceVoltage(m_handle, &value);
  return m_lastStatus == STUDICA_COBRA_OK ? value : 0.0;
}

bool Cobra::IsAvailable() const noexcept {
  uint8_t value = 0;
  m_lastStatus = StudicaCobra_IsAvailable(m_handle, &value);
  return m_lastStatus == STUDICA_COBRA_OK && value != 0;
}

int32_t Cobra::GetLastStatus() const noexcept { return m_lastStatus; }

}  // namespace studica
