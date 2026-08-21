// Copyright (c) 2026 WPILib contributors.

#include "studica/Colore.h"

#include <utility>

namespace studica {

Colore::Colore(uint8_t canId) noexcept {
  m_lastStatus = StudicaColore_CreateCAN(canId, &m_handle);
}

Colore::Colore(std::string usbPath) noexcept {
  m_lastStatus = StudicaColore_CreateUSB(usbPath.c_str(), &m_handle);
}

Colore::~Colore() { StudicaColore_Destroy(m_handle); }

Colore::Colore(Colore&& other) noexcept
    : m_handle{std::exchange(other.m_handle, 0)},
      m_lastStatus{std::exchange(other.m_lastStatus,
                                 STUDICA_COLORE_NOT_INITIALIZED)} {}

Colore& Colore::operator=(Colore&& other) noexcept {
  if (this == &other) return *this;
  StudicaColore_Destroy(m_handle);
  m_handle = std::exchange(other.m_handle, 0);
  m_lastStatus = std::exchange(other.m_lastStatus,
                               STUDICA_COLORE_NOT_INITIALIZED);
  return *this;
}

bool Colore::Read() noexcept {
  StudicaColoreSnapshot snapshot{};
  m_lastStatus = StudicaColore_Read(m_handle, &snapshot);
  return m_lastStatus == STUDICA_COLORE_OK;
}

StudicaColoreSnapshot Colore::GetSnapshot() const noexcept {
  StudicaColoreSnapshot snapshot{};
  m_lastStatus = StudicaColore_Read(m_handle, &snapshot);
  return snapshot;
}

#define STUDICA_COLORE_FACADE(Name)                                     \
  float Colore::Get##Name() const noexcept {                            \
    float value = 0.0F;                                                 \
    m_lastStatus = StudicaColore_Get##Name(m_handle, &value);           \
    return m_lastStatus == STUDICA_COLORE_OK ? value : 0.0F;            \
  }

STUDICA_COLORE_FACADE(Red)
STUDICA_COLORE_FACADE(Green)
STUDICA_COLORE_FACADE(Blue)
STUDICA_COLORE_FACADE(X)
STUDICA_COLORE_FACADE(Y)
STUDICA_COLORE_FACADE(Z)

#undef STUDICA_COLORE_FACADE

bool Colore::SetBrightness(uint8_t percent) noexcept {
  m_lastStatus = StudicaColore_SetBrightness(m_handle, percent);
  return m_lastStatus == STUDICA_COLORE_OK;
}

uint8_t Colore::GetBrightness() const noexcept {
  uint8_t value = 0;
  m_lastStatus = StudicaColore_GetBrightness(m_handle, &value);
  return value;
}

bool Colore::GetConfig(StudicaColoreConfig* configOut) const noexcept {
  m_lastStatus = StudicaColore_GetConfig(m_handle, configOut);
  return m_lastStatus == STUDICA_COLORE_OK;
}

bool Colore::LearnColor(const std::string& name, float threshold) noexcept {
  m_lastStatus = StudicaColore_LearnColor(m_handle, name.c_str(), threshold);
  return m_lastStatus == STUDICA_COLORE_OK;
}

bool Colore::SetReference(const std::string& name, float x, float y,
                          float threshold) noexcept {
  m_lastStatus =
      StudicaColore_SetReference(m_handle, name.c_str(), x, y, threshold);
  return m_lastStatus == STUDICA_COLORE_OK;
}

bool Colore::GetLearnedReference(
    const std::string& name, StudicaColoreMatchResult* resultOut) const noexcept {
  m_lastStatus = StudicaColore_GetLearnedReference(m_handle, name.c_str(),
                                                   resultOut);
  return m_lastStatus == STUDICA_COLORE_OK;
}

bool Colore::Match(StudicaColoreMatchResult* resultOut) const noexcept {
  m_lastStatus = StudicaColore_Match(m_handle, resultOut);
  return m_lastStatus == STUDICA_COLORE_OK;
}

bool Colore::IsConnected() const noexcept {
  uint8_t value = 0;
  m_lastStatus = StudicaColore_IsConnected(m_handle, &value);
  return m_lastStatus == STUDICA_COLORE_OK && value != 0;
}

int32_t Colore::GetLastStatus() const noexcept { return m_lastStatus; }

}  // namespace studica
