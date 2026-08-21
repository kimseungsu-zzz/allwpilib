// Copyright (c) 2026 WPILib contributors.

#include <jni.h>

#include <algorithm>

#include "studica/Colore.h"

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

JNIEXPORT jlongArray JNICALL Java_com_studica_frc_Colore_createCAN(
    JNIEnv* env, jclass, jint canId) {
  StudicaColoreHandle handle = 0;
  const auto status = StudicaColore_CreateCAN(static_cast<uint8_t>(canId),
                                              &handle);
  return Pair(env, static_cast<jlong>(handle), status);
}

JNIEXPORT jlongArray JNICALL Java_com_studica_frc_Colore_createUSB(
    JNIEnv* env, jclass, jstring path) {
  if (!path) return Pair(env, 0, STUDICA_COLORE_INVALID_ARGUMENT);
  const char* chars = env->GetStringUTFChars(path, nullptr);
  StudicaColoreHandle handle = 0;
  const auto status = StudicaColore_CreateUSB(chars, &handle);
  env->ReleaseStringUTFChars(path, chars);
  return Pair(env, static_cast<jlong>(handle), status);
}

JNIEXPORT void JNICALL Java_com_studica_frc_Colore_destroy(JNIEnv*, jclass,
                                                           jlong handle) {
  StudicaColore_Destroy(static_cast<StudicaColoreHandle>(handle));
}

JNIEXPORT jint JNICALL Java_com_studica_frc_Colore_read(JNIEnv*, jclass,
                                                        jlong handle) {
  StudicaColoreSnapshot snapshot{};
  return StudicaColore_Read(static_cast<StudicaColoreHandle>(handle),
                            &snapshot);
}

#define STUDICA_COLORE_JNI_GET(Name)                                      \
  JNIEXPORT jfloat JNICALL Java_com_studica_frc_Colore_get##Name(          \
      JNIEnv*, jclass, jlong handle) {                                    \
    float value = 0.0F;                                                   \
    StudicaColore_Get##Name(static_cast<StudicaColoreHandle>(handle),      \
                            &value);                                     \
    return value;                                                         \
  }

STUDICA_COLORE_JNI_GET(Red)
STUDICA_COLORE_JNI_GET(Green)
STUDICA_COLORE_JNI_GET(Blue)
STUDICA_COLORE_JNI_GET(X)
STUDICA_COLORE_JNI_GET(Y)
STUDICA_COLORE_JNI_GET(Z)

#undef STUDICA_COLORE_JNI_GET

JNIEXPORT jint JNICALL Java_com_studica_frc_Colore_setBrightness(
    JNIEnv*, jclass, jlong handle, jint percent) {
  return StudicaColore_SetBrightness(static_cast<StudicaColoreHandle>(handle),
                                     percent);
}

JNIEXPORT jint JNICALL Java_com_studica_frc_Colore_getBrightness(
    JNIEnv*, jclass, jlong handle) {
  uint8_t value = 0;
  StudicaColore_GetBrightness(static_cast<StudicaColoreHandle>(handle), &value);
  return value;
}

JNIEXPORT jbyteArray JNICALL Java_com_studica_frc_Colore_getConfig(
    JNIEnv* env, jclass, jlong handle) {
  StudicaColoreConfig config{};
  const auto status = StudicaColore_GetConfig(
      static_cast<StudicaColoreHandle>(handle), &config);
  if (status != STUDICA_COLORE_OK) return env->NewByteArray(0);
  return Bytes(env, config.raw, config.rawLength);
}

JNIEXPORT jint JNICALL Java_com_studica_frc_Colore_learnColor(
    JNIEnv* env, jclass, jlong handle, jstring name, jfloat threshold) {
  if (!name) return STUDICA_COLORE_INVALID_ARGUMENT;
  const char* chars = env->GetStringUTFChars(name, nullptr);
  const auto status = StudicaColore_LearnColor(
      static_cast<StudicaColoreHandle>(handle), chars, threshold);
  env->ReleaseStringUTFChars(name, chars);
  return status;
}

JNIEXPORT jint JNICALL Java_com_studica_frc_Colore_setReference(
    JNIEnv* env, jclass, jlong handle, jstring name, jfloat x, jfloat y,
    jfloat threshold) {
  if (!name) return STUDICA_COLORE_INVALID_ARGUMENT;
  const char* chars = env->GetStringUTFChars(name, nullptr);
  const auto status = StudicaColore_SetReference(
      static_cast<StudicaColoreHandle>(handle), chars, x, y, threshold);
  env->ReleaseStringUTFChars(name, chars);
  return status;
}

JNIEXPORT jstring JNICALL Java_com_studica_frc_Colore_matchLabel(
    JNIEnv* env, jclass, jlong handle) {
  StudicaColoreMatchResult result{};
  if (StudicaColore_Match(static_cast<StudicaColoreHandle>(handle), &result) !=
      STUDICA_COLORE_OK) {
    return env->NewStringUTF("");
  }
  return env->NewStringUTF(result.label);
}

JNIEXPORT jfloat JNICALL Java_com_studica_frc_Colore_matchConfidence(
    JNIEnv*, jclass, jlong handle) {
  StudicaColoreMatchResult result{};
  if (StudicaColore_Match(static_cast<StudicaColoreHandle>(handle), &result) !=
      STUDICA_COLORE_OK) {
    return 0.0F;
  }
  return result.confidence;
}

JNIEXPORT jboolean JNICALL Java_com_studica_frc_Colore_isConnected(
    JNIEnv*, jclass, jlong handle) {
  uint8_t connected = 0;
  const auto status = StudicaColore_IsConnected(
      static_cast<StudicaColoreHandle>(handle), &connected);
  return status == STUDICA_COLORE_OK && connected ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jint JNICALL Java_com_studica_frc_Colore_getLastStatus(
    JNIEnv*, jclass, jlong handle) {
  int32_t status = STUDICA_COLORE_NOT_INITIALIZED;
  StudicaColore_GetLastStatus(static_cast<StudicaColoreHandle>(handle), &status);
  return status;
}

}  // extern "C"
