// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include "studica/Titan.h"

namespace {

jlongArray MakeLongPair(JNIEnv* env, jlong first, jint status) {
  auto result = env->NewLongArray(2);
  if (!result) return nullptr;
  const jlong values[2] = {first, static_cast<jlong>(status)};
  env->SetLongArrayRegion(result, 0, 2, values);
  return result;
}

jdoubleArray MakeDoublePair(JNIEnv* env, jdouble first, jint status) {
  auto result = env->NewDoubleArray(2);
  if (!result) return nullptr;
  const jdouble values[2] = {first, static_cast<jdouble>(status)};
  env->SetDoubleArrayRegion(result, 0, 2, values);
  return result;
}

}  // namespace

extern "C" {

JNIEXPORT jlongArray JNICALL Java_com_studica_frc_TitanQuad_create(
    JNIEnv* env, jclass, jint canId, jint motorPort, jint frequency,
    jdouble distancePerTick) {
  StudicaTitanHandle handle = 0;
  const auto status = StudicaTitan_Create(
      static_cast<uint8_t>(canId), static_cast<uint8_t>(motorPort),
      static_cast<uint16_t>(frequency), distancePerTick, &handle);
  return MakeLongPair(env, static_cast<jlong>(handle), status);
}

JNIEXPORT void JNICALL Java_com_studica_frc_TitanQuad_destroy(JNIEnv*, jclass,
                                                               jlong handle) {
  StudicaTitan_Destroy(static_cast<StudicaTitanHandle>(handle));
}

JNIEXPORT jint JNICALL Java_com_studica_frc_TitanQuad_set(JNIEnv*, jclass,
                                                           jlong handle,
                                                           jdouble speed) {
  return StudicaTitan_Set(static_cast<StudicaTitanHandle>(handle), speed);
}

JNIEXPORT jdoubleArray JNICALL Java_com_studica_frc_TitanQuad_get(
    JNIEnv* env, jclass, jlong handle) {
  double speed = 0.0;
  const auto status = StudicaTitan_Get(
      static_cast<StudicaTitanHandle>(handle), &speed);
  return MakeDoublePair(env, speed, status);
}

JNIEXPORT jint JNICALL Java_com_studica_frc_TitanQuad_setInverted(
    JNIEnv*, jclass, jlong handle, jboolean inverted) {
  return StudicaTitan_SetInverted(static_cast<StudicaTitanHandle>(handle),
                                  static_cast<uint8_t>(inverted));
}

JNIEXPORT jlongArray JNICALL Java_com_studica_frc_TitanQuad_getInverted(
    JNIEnv* env, jclass, jlong handle) {
  uint8_t inverted = 0;
  const auto status = StudicaTitan_GetInverted(
      static_cast<StudicaTitanHandle>(handle), &inverted);
  return MakeLongPair(env, inverted, status);
}

JNIEXPORT jint JNICALL Java_com_studica_frc_TitanQuad_enable(JNIEnv*, jclass,
                                                              jlong handle) {
  return StudicaTitan_Enable(static_cast<StudicaTitanHandle>(handle));
}

JNIEXPORT jint JNICALL Java_com_studica_frc_TitanQuad_disable(JNIEnv*, jclass,
                                                               jlong handle) {
  return StudicaTitan_Disable(static_cast<StudicaTitanHandle>(handle));
}

JNIEXPORT jint JNICALL Java_com_studica_frc_TitanQuad_stopMotor(
    JNIEnv*, jclass, jlong handle) {
  return StudicaTitan_StopMotor(static_cast<StudicaTitanHandle>(handle));
}

JNIEXPORT jint JNICALL Java_com_studica_frc_TitanQuad_isAvailable(
    JNIEnv*, jclass, jlong handle) {
  uint8_t available = 0;
  const auto status = StudicaTitan_IsAvailable(
      static_cast<StudicaTitanHandle>(handle), &available);
  return status == STUDICA_TITAN_OK ? static_cast<jint>(available) : 0;
}

JNIEXPORT jint JNICALL
Java_com_studica_frc_TitanQuad_setTargetVelocity(JNIEnv*, jclass, jlong handle,
                                                  jfloat rpm) {
  return StudicaTitan_SetTargetVelocity(
      static_cast<StudicaTitanHandle>(handle), rpm);
}

JNIEXPORT jint JNICALL
Java_com_studica_frc_TitanQuad_setTargetDistance(JNIEnv*, jclass, jlong handle,
                                                  jint counts) {
  return StudicaTitan_SetTargetDistance(
      static_cast<StudicaTitanHandle>(handle), counts);
}

JNIEXPORT jint JNICALL
Java_com_studica_frc_TitanQuad_setTargetAngle(JNIEnv*, jclass, jlong handle,
                                               jdouble angle) {
  return StudicaTitan_SetTargetAngle(
      static_cast<StudicaTitanHandle>(handle), angle);
}

JNIEXPORT jint JNICALL
Java_com_studica_frc_TitanQuad_setPositionHold(JNIEnv*, jclass, jlong handle,
                                                jboolean hold) {
  return StudicaTitan_SetPositionHold(
      static_cast<StudicaTitanHandle>(handle), static_cast<uint8_t>(hold));
}

JNIEXPORT jint JNICALL
Java_com_studica_frc_TitanQuad_setCurrentLimit(JNIEnv*, jclass, jlong handle,
                                                jfloat amps) {
  return StudicaTitan_SetCurrentLimit(
      static_cast<StudicaTitanHandle>(handle), amps);
}

JNIEXPORT jint JNICALL
Java_com_studica_frc_TitanQuad_setCurrentLimitMode(JNIEnv*, jclass,
                                                    jlong handle, jint mode) {
  return StudicaTitan_SetCurrentLimitMode(
      static_cast<StudicaTitanHandle>(handle), static_cast<uint8_t>(mode));
}

JNIEXPORT jint JNICALL
Java_com_studica_frc_TitanQuad_setMotorStopMode(JNIEnv*, jclass, jlong handle,
                                                 jint mode) {
  return StudicaTitan_SetMotorStopMode(
      static_cast<StudicaTitanHandle>(handle), static_cast<uint8_t>(mode));
}

JNIEXPORT jint JNICALL Java_com_studica_frc_TitanQuad_setPIDType(
    JNIEnv*, jclass, jlong handle, jint type) {
  return StudicaTitan_SetPIDType(static_cast<StudicaTitanHandle>(handle),
                                 static_cast<uint8_t>(type));
}

JNIEXPORT jstring JNICALL Java_com_studica_frc_TitanQuad_getFirmwareVersion(
    JNIEnv* env, jclass, jlong handle) {
  char buffer[64]{};
  if (StudicaTitan_GetFirmwareVersion(
          static_cast<StudicaTitanHandle>(handle), buffer, sizeof(buffer)) !=
      STUDICA_TITAN_OK) {
    buffer[0] = '\0';
  }
  return env->NewStringUTF(buffer);
}

JNIEXPORT jstring JNICALL Java_com_studica_frc_TitanQuad_getHardwareVersion(
    JNIEnv* env, jclass, jlong handle) {
  char buffer[64]{};
  if (StudicaTitan_GetHardwareVersion(
          static_cast<StudicaTitanHandle>(handle), buffer, sizeof(buffer)) !=
      STUDICA_TITAN_OK) {
    buffer[0] = '\0';
  }
  return env->NewStringUTF(buffer);
}

JNIEXPORT jfloat JNICALL
Java_com_studica_frc_TitanQuad_getControllerTemperature(JNIEnv*, jclass,
                                                         jlong handle) {
  float temperature = 0.0F;
  StudicaTitan_GetControllerTemperature(
      static_cast<StudicaTitanHandle>(handle), &temperature);
  return temperature;
}

JNIEXPORT jint JNICALL Java_com_studica_frc_TitanQuadEncoder_getRaw(
    JNIEnv*, jclass, jlong handle) {
  int32_t count = 0;
  StudicaTitan_GetEncoderCount(static_cast<StudicaTitanHandle>(handle), &count);
  return count;
}

JNIEXPORT jdouble JNICALL Java_com_studica_frc_TitanQuadEncoder_getDistance(
    JNIEnv*, jclass, jlong handle) {
  double value = 0.0;
  StudicaTitan_GetEncoderDistance(static_cast<StudicaTitanHandle>(handle),
                                  &value);
  return value;
}

JNIEXPORT jdouble JNICALL Java_com_studica_frc_TitanQuadEncoder_getRPM(
    JNIEnv*, jclass, jlong handle) {
  double value = 0.0;
  StudicaTitan_GetRPM(static_cast<StudicaTitanHandle>(handle), &value);
  return value;
}

JNIEXPORT jdouble JNICALL
Java_com_studica_frc_TitanQuadEncoder_getAbsoluteAngle(JNIEnv*, jclass,
                                                         jlong handle) {
  double value = 0.0;
  StudicaTitan_GetAbsoluteAngle(static_cast<StudicaTitanHandle>(handle),
                                &value);
  return value;
}

JNIEXPORT jint JNICALL Java_com_studica_frc_TitanQuadEncoder_getForwardLimit(
    JNIEnv*, jclass, jlong handle) {
  uint8_t value = 0;
  StudicaTitan_GetForwardLimit(static_cast<StudicaTitanHandle>(handle), &value);
  return value;
}

JNIEXPORT jint JNICALL Java_com_studica_frc_TitanQuadEncoder_getReverseLimit(
    JNIEnv*, jclass, jlong handle) {
  uint8_t value = 0;
  StudicaTitan_GetReverseLimit(static_cast<StudicaTitanHandle>(handle), &value);
  return value;
}

JNIEXPORT jint JNICALL
Java_com_studica_frc_TitanQuadEncoder_setDistancePerTick(JNIEnv*, jclass,
                                                           jlong handle,
                                                           jdouble value) {
  return StudicaTitan_SetDistancePerTick(
      static_cast<StudicaTitanHandle>(handle), value);
}

JNIEXPORT jint JNICALL
Java_com_studica_frc_TitanQuadEncoder_setReverseDirection(JNIEnv*, jclass,
                                                           jlong handle,
                                                           jboolean reverse) {
  return StudicaTitan_SetEncoderReversed(
      static_cast<StudicaTitanHandle>(handle), static_cast<uint8_t>(reverse));
}

JNIEXPORT jint JNICALL Java_com_studica_frc_TitanQuadEncoder_reset(
    JNIEnv*, jclass, jlong handle) {
  return StudicaTitan_ResetEncoder(static_cast<StudicaTitanHandle>(handle));
}

}  // extern "C"
