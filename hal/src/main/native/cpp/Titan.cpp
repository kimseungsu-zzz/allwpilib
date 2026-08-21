// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "studica/Titan.h"

#include <limits>

namespace {
template <typename T>
T ReadValue(StudicaTitanHandle handle, T fallback,
            int32_t (*reader)(StudicaTitanHandle, T*)) noexcept {
  T value = fallback;
  return reader(handle, &value) == STUDICA_TITAN_OK ? value : fallback;
}
}  // namespace

namespace studica {

TitanQuad::TitanQuad(uint8_t canId, uint8_t motorPort,
                     uint16_t motorFrequencyHz,
                     double distancePerTick) noexcept {
  m_lastStatus = StudicaTitan_Create(canId, motorPort, motorFrequencyHz,
                                     distancePerTick, &m_handle);
}

TitanQuad::~TitanQuad() { StudicaTitan_Destroy(m_handle); }

TitanQuad::TitanQuad(TitanQuad&& other) noexcept
    : m_handle{other.m_handle}, m_lastStatus{other.m_lastStatus} {
  other.m_handle = 0;
  other.m_lastStatus = STUDICA_TITAN_NOT_INITIALIZED;
}

TitanQuad& TitanQuad::operator=(TitanQuad&& other) noexcept {
  if (this != &other) {
    StudicaTitan_Destroy(m_handle);
    m_handle = other.m_handle;
    m_lastStatus = other.m_lastStatus;
    other.m_handle = 0;
    other.m_lastStatus = STUDICA_TITAN_NOT_INITIALIZED;
  }
  return *this;
}

void TitanQuad::Set(double speed) noexcept {
  m_lastStatus = StudicaTitan_Set(m_handle, speed);
}

double TitanQuad::Get() const noexcept {
  return ReadValue(m_handle, std::numeric_limits<double>::quiet_NaN(),
                   StudicaTitan_Get);
}

void TitanQuad::SetInverted(bool inverted) noexcept {
  m_lastStatus = StudicaTitan_SetInverted(m_handle, inverted);
}

bool TitanQuad::GetInverted() const noexcept {
  return ReadValue<uint8_t>(m_handle, 0, StudicaTitan_GetInverted) != 0;
}

void TitanQuad::Enable() noexcept {
  m_lastStatus = StudicaTitan_Enable(m_handle);
}

void TitanQuad::Disable() noexcept {
  m_lastStatus = StudicaTitan_Disable(m_handle);
}

void TitanQuad::StopMotor() noexcept {
  m_lastStatus = StudicaTitan_StopMotor(m_handle);
}

void TitanQuad::SetTargetVelocity(float rpm) noexcept {
  m_lastStatus = StudicaTitan_SetTargetVelocity(m_handle, rpm);
}

void TitanQuad::SetTargetDistance(int32_t encoderCounts) noexcept {
  m_lastStatus = StudicaTitan_SetTargetDistance(m_handle, encoderCounts);
}

void TitanQuad::SetTargetAngle(double angleDegrees) noexcept {
  m_lastStatus = StudicaTitan_SetTargetAngle(m_handle, angleDegrees);
}

void TitanQuad::SetPositionHold(bool hold) noexcept {
  m_lastStatus = StudicaTitan_SetPositionHold(m_handle, hold);
}

void TitanQuad::SetCurrentLimit(float amps) noexcept {
  m_lastStatus = StudicaTitan_SetCurrentLimit(m_handle, amps);
}

void TitanQuad::SetCurrentLimitMode(uint8_t mode) noexcept {
  m_lastStatus = StudicaTitan_SetCurrentLimitMode(m_handle, mode);
}

void TitanQuad::SetMotorStopMode(uint8_t mode) noexcept {
  m_lastStatus = StudicaTitan_SetMotorStopMode(m_handle, mode);
}

void TitanQuad::SetPIDType(uint8_t type) noexcept {
  m_lastStatus = StudicaTitan_SetPIDType(m_handle, type);
}

std::string TitanQuad::GetFirmwareVersion() const {
  char buffer[64]{};
  return StudicaTitan_GetFirmwareVersion(m_handle, buffer, sizeof(buffer)) ==
                 STUDICA_TITAN_OK
             ? std::string{buffer}
             : std::string{};
}

std::string TitanQuad::GetHardwareVersion() const {
  char buffer[64]{};
  return StudicaTitan_GetHardwareVersion(m_handle, buffer, sizeof(buffer)) ==
                 STUDICA_TITAN_OK
             ? std::string{buffer}
             : std::string{};
}

float TitanQuad::GetControllerTemperature() const noexcept {
  float value = std::numeric_limits<float>::quiet_NaN();
  StudicaTitan_GetControllerTemperature(m_handle, &value);
  return value;
}

bool TitanQuad::IsAvailable() const noexcept {
  return ReadValue<uint8_t>(m_handle, 0, StudicaTitan_IsAvailable) != 0;
}

int32_t TitanQuad::GetLastStatus() const noexcept { return m_lastStatus; }

TitanQuadEncoder::TitanQuadEncoder(uint8_t canId, uint8_t motorPort,
                                   uint16_t motorFrequencyHz,
                                   double distancePerTick) noexcept {
  m_lastStatus = StudicaTitan_Create(canId, motorPort, motorFrequencyHz,
                                     distancePerTick, &m_handle);
}

TitanQuadEncoder::~TitanQuadEncoder() { StudicaTitan_Destroy(m_handle); }

TitanQuadEncoder::TitanQuadEncoder(TitanQuadEncoder&& other) noexcept
    : m_handle{other.m_handle}, m_lastStatus{other.m_lastStatus} {
  other.m_handle = 0;
  other.m_lastStatus = STUDICA_TITAN_NOT_INITIALIZED;
}

TitanQuadEncoder& TitanQuadEncoder::operator=(TitanQuadEncoder&& other) noexcept {
  if (this != &other) {
    StudicaTitan_Destroy(m_handle);
    m_handle = other.m_handle;
    m_lastStatus = other.m_lastStatus;
    other.m_handle = 0;
    other.m_lastStatus = STUDICA_TITAN_NOT_INITIALIZED;
  }
  return *this;
}

int32_t TitanQuadEncoder::GetRaw() const noexcept {
  return ReadValue<int32_t>(m_handle, 0, StudicaTitan_GetEncoderCount);
}

double TitanQuadEncoder::GetDistance() const noexcept {
  return ReadValue<double>(m_handle, std::numeric_limits<double>::quiet_NaN(),
                           StudicaTitan_GetEncoderDistance);
}

double TitanQuadEncoder::GetRPM() const noexcept {
  return ReadValue<double>(m_handle, std::numeric_limits<double>::quiet_NaN(),
                           StudicaTitan_GetRPM);
}

double TitanQuadEncoder::GetAbsoluteAngle() const noexcept {
  return ReadValue<double>(m_handle, std::numeric_limits<double>::quiet_NaN(),
                           StudicaTitan_GetAbsoluteAngle);
}

bool TitanQuadEncoder::GetForwardLimit() const noexcept {
  return ReadValue<uint8_t>(m_handle, 0, StudicaTitan_GetForwardLimit) != 0;
}

bool TitanQuadEncoder::GetReverseLimit() const noexcept {
  return ReadValue<uint8_t>(m_handle, 0, StudicaTitan_GetReverseLimit) != 0;
}

void TitanQuadEncoder::SetDistancePerTick(double value) noexcept {
  m_lastStatus = StudicaTitan_SetDistancePerTick(m_handle, value);
}

void TitanQuadEncoder::SetReverseDirection(bool reverse) noexcept {
  m_lastStatus = StudicaTitan_SetEncoderReversed(m_handle, reverse);
}

bool TitanQuadEncoder::Reset() noexcept {
  m_lastStatus = StudicaTitan_ResetEncoder(m_handle);
  return m_lastStatus == STUDICA_TITAN_OK;
}

bool TitanQuadEncoder::IsAvailable() const noexcept {
  return ReadValue<uint8_t>(m_handle, 0, StudicaTitan_IsAvailable) != 0;
}

int32_t TitanQuadEncoder::GetLastStatus() const noexcept { return m_lastStatus; }

}  // namespace studica
