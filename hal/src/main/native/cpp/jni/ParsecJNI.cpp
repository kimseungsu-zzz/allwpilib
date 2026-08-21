// Copyright (c) 2026 WPILib contributors.

#include <jni.h>

#include <algorithm>
#include <array>
#include <string>

#include "studica/Parsec.h"

namespace {

jlongArray Pair(JNIEnv* env, jlong handle, jint status) {
  auto result = env->NewLongArray(2);
  if (!result) return nullptr;
  const jlong values[2] = {handle, static_cast<jlong>(status)};
  env->SetLongArrayRegion(result, 0, 2, values);
  return result;
}

jbyteArray Bytes(JNIEnv* env, const uint8_t* data, std::size_t length) {
  auto result = env->NewByteArray(static_cast<jsize>(length));
  if (!result) return nullptr;
  env->SetByteArrayRegion(result, 0, static_cast<jsize>(length),
                          reinterpret_cast<const jbyte*>(data));
  return result;
}

}  // namespace

extern "C" {

JNIEXPORT jlongArray JNICALL Java_com_studica_frc_Parsec_createCAN(
    JNIEnv* env, jclass, jint canId) {
  StudicaParsecHandle handle = 0;
  const auto status = StudicaParsec_CreateCAN(static_cast<uint8_t>(canId),
                                              &handle);
  return Pair(env, static_cast<jlong>(handle), status);
}

JNIEXPORT jlongArray JNICALL Java_com_studica_frc_Parsec_createUSB(
    JNIEnv* env, jclass, jstring path) {
  if (!path) return Pair(env, 0, STUDICA_PARSEC_INVALID_ARGUMENT);
  const char* chars = env->GetStringUTFChars(path, nullptr);
  StudicaParsecHandle handle = 0;
  const auto status = StudicaParsec_CreateUSB(chars, &handle);
  env->ReleaseStringUTFChars(path, chars);
  return Pair(env, static_cast<jlong>(handle), status);
}

JNIEXPORT void JNICALL Java_com_studica_frc_Parsec_destroy(JNIEnv*, jclass,
                                                           jlong handle) {
  StudicaParsec_Destroy(static_cast<StudicaParsecHandle>(handle));
}

JNIEXPORT jint JNICALL Java_com_studica_frc_Parsec_read(JNIEnv*, jclass,
                                                        jlong handle) {
  StudicaParsecSnapshot snapshot{};
  return StudicaParsec_ReadZones(static_cast<StudicaParsecHandle>(handle),
                                 &snapshot);
}

JNIEXPORT jint JNICALL Java_com_studica_frc_Parsec_getResolution(JNIEnv*, jclass,
                                                                  jlong handle) {
  uint32_t value = 0;
  StudicaParsec_GetResolution(static_cast<StudicaParsecHandle>(handle), &value);
  return static_cast<jint>(value);
}

JNIEXPORT jint JNICALL Java_com_studica_frc_Parsec_getZoneCount(JNIEnv*, jclass,
                                                                jlong handle) {
  uint32_t value = 0;
  StudicaParsec_GetZoneCount(static_cast<StudicaParsecHandle>(handle), &value);
  return static_cast<jint>(value);
}

JNIEXPORT jshortArray JNICALL Java_com_studica_frc_Parsec_getDistances(
    JNIEnv* env, jclass, jlong handle) {
  std::array<int16_t, STUDICA_PARSEC_MAX_ZONES> values{};
  std::fill(values.begin(), values.end(), static_cast<int16_t>(-2));
  const auto count = StudicaParsec_GetDistances(
      static_cast<StudicaParsecHandle>(handle), values.data(), values.size());
  if (count < 0) return env->NewShortArray(0);
  auto result = env->NewShortArray(STUDICA_PARSEC_MAX_ZONES);
  if (!result) return nullptr;
  env->SetShortArrayRegion(result, 0, STUDICA_PARSEC_MAX_ZONES,
                           reinterpret_cast<const jshort*>(values.data()));
  return result;
}

JNIEXPORT jintArray JNICALL Java_com_studica_frc_Parsec_getMinDistance(
    JNIEnv* env, jclass, jlong handle) {
  int16_t distance = 0;
  uint8_t valid = 0;
  const auto status = StudicaParsec_GetMinDistance(
      static_cast<StudicaParsecHandle>(handle), &distance, &valid);
  auto result = env->NewIntArray(3);
  if (!result) return nullptr;
  const jint values[3] = {distance, valid, status};
  env->SetIntArrayRegion(result, 0, 3, values);
  return result;
}

JNIEXPORT jbyteArray JNICALL Java_com_studica_frc_Parsec_getConfig(
    JNIEnv* env, jclass, jlong handle) {
  StudicaParsecConfig config{};
  const auto status = StudicaParsec_GetConfig(
      static_cast<StudicaParsecHandle>(handle), &config);
  if (status != STUDICA_PARSEC_OK) return env->NewByteArray(0);
  return Bytes(env, config.raw, config.rawLength);
}

JNIEXPORT jboolean JNICALL Java_com_studica_frc_Parsec_isConnected(
    JNIEnv*, jclass, jlong handle) {
  uint8_t connected = 0;
  const auto status = StudicaParsec_IsConnected(
      static_cast<StudicaParsecHandle>(handle), &connected);
  return status == STUDICA_PARSEC_OK && connected ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jint JNICALL Java_com_studica_frc_Parsec_getLastStatus(
    JNIEnv*, jclass, jlong handle) {
  int32_t status = STUDICA_PARSEC_NOT_INITIALIZED;
  StudicaParsec_GetLastStatus(static_cast<StudicaParsecHandle>(handle), &status);
  return status;
}

}  // extern "C"
