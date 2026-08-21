// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#ifdef __cplusplus
#include <cstdint>
#include <string>
#else
#include <stdint.h>
#endif

// Stable vendor ABI for the Studica Titan Quad controller.  The ABI is kept
// independent of WPILib and of the C++ type used by the imported driver so it
// can be consumed by JNI and Python ctypes without a C++ ABI dependency.
#define STUDICA_TITAN_ABI_VERSION 1u
#define STUDICA_TITAN_DEFAULT_CAN_ID 42u
#define STUDICA_TITAN_MIN_CAN_ID 1u
#define STUDICA_TITAN_MAX_CAN_ID 62u
#define STUDICA_TITAN_MOTOR_COUNT 4u
#define STUDICA_TITAN_KEEPALIVE_MS 150u

typedef uint32_t StudicaTitanHandle;

#ifdef __cplusplus
enum StudicaTitanStatus : int32_t {
#else
typedef enum StudicaTitanStatus {
#endif
  STUDICA_TITAN_OK = 0,
  STUDICA_TITAN_INVALID_ARGUMENT = -22,
  STUDICA_TITAN_UNAVAILABLE = -19,
  STUDICA_TITAN_UNSUPPORTED = -95,
  STUDICA_TITAN_NOT_INITIALIZED = -107,
  STUDICA_TITAN_TIMEOUT = -110,
  STUDICA_TITAN_BUFFER_TOO_SMALL = -75,
  STUDICA_TITAN_INTERNAL_ERROR = -5,
#ifdef __cplusplus
};
#else
} StudicaTitanStatus;
#endif

typedef struct StudicaTitanSnapshot {
  uint32_t structSize;
  uint32_t abiVersion;
  uint8_t canId;
  uint8_t motorPort;
  uint8_t enabled;
  uint8_t inverted;
  double commandedSpeed;
  int32_t encoderCount;
  double encoderDistance;
  double rpm;
  double absoluteAngleDegrees;
  uint8_t forwardLimitTriggered;
  uint8_t reverseLimitTriggered;
  uint8_t connected;
  uint8_t reserved;
  double distancePerTick;
  uint16_t motorFrequencyHz;
  uint16_t reserved2;
  float controllerTemperatureC;
  char firmwareVersion[64];
  char hardwareVersion[64];
} StudicaTitanSnapshot;

#ifdef __cplusplus

namespace studica {

/** C++ compatibility facade for the historical Studica TitanQuad API. */
class TitanQuad final {
 public:
  explicit TitanQuad(uint8_t canId = STUDICA_TITAN_DEFAULT_CAN_ID,
                     uint8_t motorPort = 0, uint16_t motorFrequencyHz = 15600,
                     double distancePerTick = 0.0) noexcept;
  ~TitanQuad();

  TitanQuad(const TitanQuad&) = delete;
  TitanQuad& operator=(const TitanQuad&) = delete;
  TitanQuad(TitanQuad&& other) noexcept;
  TitanQuad& operator=(TitanQuad&& other) noexcept;

  void Set(double speed) noexcept;
  double Get() const noexcept;
  void SetInverted(bool inverted) noexcept;
  bool GetInverted() const noexcept;
  void Enable() noexcept;
  void Disable() noexcept;
  void StopMotor() noexcept;
  void SetTargetVelocity(float rpm) noexcept;
  void SetTargetDistance(int32_t encoderCounts) noexcept;
  void SetTargetAngle(double angleDegrees) noexcept;
  void SetPositionHold(bool hold) noexcept;
  void SetCurrentLimit(float amps) noexcept;
  void SetCurrentLimitMode(uint8_t mode) noexcept;
  void SetMotorStopMode(uint8_t mode) noexcept;
  void SetPIDType(uint8_t type) noexcept;
  std::string GetFirmwareVersion() const;
  std::string GetHardwareVersion() const;
  float GetControllerTemperature() const noexcept;

  bool IsAvailable() const noexcept;
  int32_t GetLastStatus() const noexcept;
  StudicaTitanHandle GetHandle() const noexcept { return m_handle; }

 private:
  StudicaTitanHandle m_handle = 0;
  mutable int32_t m_lastStatus = STUDICA_TITAN_NOT_INITIALIZED;
};

/** Titan channel encoder/absolute encoder compatibility facade. */
class TitanQuadEncoder final {
 public:
  explicit TitanQuadEncoder(uint8_t canId = STUDICA_TITAN_DEFAULT_CAN_ID,
                            uint8_t motorPort = 0,
                            uint16_t motorFrequencyHz = 15600,
                            double distancePerTick = 1.0) noexcept;
  ~TitanQuadEncoder();

  TitanQuadEncoder(const TitanQuadEncoder&) = delete;
  TitanQuadEncoder& operator=(const TitanQuadEncoder&) = delete;
  TitanQuadEncoder(TitanQuadEncoder&& other) noexcept;
  TitanQuadEncoder& operator=(TitanQuadEncoder&& other) noexcept;

  int32_t GetRaw() const noexcept;
  double GetDistance() const noexcept;
  double GetRPM() const noexcept;
  double GetAbsoluteAngle() const noexcept;
  bool GetForwardLimit() const noexcept;
  bool GetReverseLimit() const noexcept;
  void SetDistancePerTick(double value) noexcept;
  void SetReverseDirection(bool reverse) noexcept;
  bool Reset() noexcept;
  bool IsAvailable() const noexcept;
  int32_t GetLastStatus() const noexcept;

 private:
  StudicaTitanHandle m_handle = 0;
  mutable int32_t m_lastStatus = STUDICA_TITAN_NOT_INITIALIZED;
};

}  // namespace studica

extern "C" {
#endif

int32_t StudicaTitan_Create(uint8_t canId, uint8_t motorPort,
                            uint16_t motorFrequencyHz, double distancePerTick,
                            StudicaTitanHandle* handleOut);
void StudicaTitan_Destroy(StudicaTitanHandle handle);
int32_t StudicaTitan_Set(StudicaTitanHandle handle, double speed);
int32_t StudicaTitan_Get(StudicaTitanHandle handle, double* speedOut);
int32_t StudicaTitan_SetInverted(StudicaTitanHandle handle, uint8_t inverted);
int32_t StudicaTitan_GetInverted(StudicaTitanHandle handle,
                                 uint8_t* invertedOut);
int32_t StudicaTitan_Enable(StudicaTitanHandle handle);
int32_t StudicaTitan_Disable(StudicaTitanHandle handle);
int32_t StudicaTitan_StopMotor(StudicaTitanHandle handle);
int32_t StudicaTitan_GetSnapshot(StudicaTitanHandle handle,
                                 StudicaTitanSnapshot* snapshotOut);
int32_t StudicaTitan_GetEncoderCount(StudicaTitanHandle handle,
                                     int32_t* countOut);
int32_t StudicaTitan_GetEncoderDistance(StudicaTitanHandle handle,
                                        double* distanceOut);
int32_t StudicaTitan_GetRPM(StudicaTitanHandle handle, double* rpmOut);
int32_t StudicaTitan_ResetEncoder(StudicaTitanHandle handle);
int32_t StudicaTitan_SetDistancePerTick(StudicaTitanHandle handle,
                                        double distancePerTick);
int32_t StudicaTitan_SetEncoderReversed(StudicaTitanHandle handle,
                                        uint8_t reversed);
int32_t StudicaTitan_GetAbsoluteAngle(StudicaTitanHandle handle,
                                      double* angleDegreesOut);
int32_t StudicaTitan_GetForwardLimit(StudicaTitanHandle handle,
                                     uint8_t* triggeredOut);
int32_t StudicaTitan_GetReverseLimit(StudicaTitanHandle handle,
                                     uint8_t* triggeredOut);
int32_t StudicaTitan_SetTargetVelocity(StudicaTitanHandle handle, float rpm);
int32_t StudicaTitan_SetTargetDistance(StudicaTitanHandle handle,
                                       int32_t encoderCounts);
int32_t StudicaTitan_SetTargetAngle(StudicaTitanHandle handle,
                                    double angleDegrees);
int32_t StudicaTitan_SetPositionHold(StudicaTitanHandle handle, uint8_t hold);
int32_t StudicaTitan_SetMotorFrequency(StudicaTitanHandle handle,
                                       uint16_t frequencyHz);
int32_t StudicaTitan_SetCurrentLimit(StudicaTitanHandle handle,
                                     float limitAmps);
int32_t StudicaTitan_SetCurrentLimitMode(StudicaTitanHandle handle,
                                         uint8_t mode);
int32_t StudicaTitan_SetMotorStopMode(StudicaTitanHandle handle, uint8_t mode);
int32_t StudicaTitan_SetPIDType(StudicaTitanHandle handle, uint8_t type);
int32_t StudicaTitan_GetFirmwareVersion(StudicaTitanHandle handle, char* out,
                                        uint32_t capacity);
int32_t StudicaTitan_GetHardwareVersion(StudicaTitanHandle handle, char* out,
                                        uint32_t capacity);
int32_t StudicaTitan_GetControllerTemperature(StudicaTitanHandle handle,
                                              float* temperatureOut);
int32_t StudicaTitan_IsAvailable(StudicaTitanHandle handle,
                                 uint8_t* availableOut);
void StudicaTitan_ShutdownAll(void);

#ifdef __cplusplus
}
#endif
