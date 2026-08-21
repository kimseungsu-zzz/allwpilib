// Copyright (c) 2026 WPILib contributors.

#include <jni.h>

#include "studica/Cobra.h"

namespace {
jlongArray Pair(JNIEnv* env, jlong value, jint status) {
  auto result = env->NewLongArray(2);
  if (!result) return nullptr;
  const jlong values[2] = {value, static_cast<jlong>(status)};
  env->SetLongArrayRegion(result, 0, 2, values);
  return result;
}
}  // namespace

extern "C" {

JNIEXPORT jlongArray JNICALL Java_com_studica_frc_Cobra_create(
    JNIEnv* env, jclass, jdouble referenceVoltage) {
  StudicaCobraHandle handle = 0;
  const auto status = StudicaCobra_Create(referenceVoltage, &handle);
  return Pair(env, static_cast<jlong>(handle), status);
}
JNIEXPORT void JNICALL Java_com_studica_frc_Cobra_destroy(JNIEnv*, jclass,
                                                          jlong handle) {
  StudicaCobra_Destroy(static_cast<StudicaCobraHandle>(handle));
}
JNIEXPORT jint JNICALL Java_com_studica_frc_Cobra_getRaw(JNIEnv*, jclass,
                                                          jlong handle,
                                                          jint channel) {
  int32_t value = -1;
  StudicaCobra_GetRaw(static_cast<StudicaCobraHandle>(handle),
                      static_cast<uint8_t>(channel), &value);
  return value;
}
JNIEXPORT jdouble JNICALL Java_com_studica_frc_Cobra_getVoltage(
    JNIEnv*, jclass, jlong handle, jint channel) {
  double value = 0.0;
  StudicaCobra_GetVoltage(static_cast<StudicaCobraHandle>(handle),
                          static_cast<uint8_t>(channel), &value);
  return value;
}
JNIEXPORT jint JNICALL Java_com_studica_frc_Cobra_getChannelCount(
    JNIEnv*, jclass, jlong handle) {
  uint8_t value = 0;
  StudicaCobra_GetChannelCount(static_cast<StudicaCobraHandle>(handle), &value);
  return value;
}
JNIEXPORT jdouble JNICALL Java_com_studica_frc_Cobra_getReferenceVoltage(
    JNIEnv*, jclass, jlong handle) {
  double value = 0.0;
  StudicaCobra_GetReferenceVoltage(static_cast<StudicaCobraHandle>(handle),
                                   &value);
  return value;
}
JNIEXPORT jboolean JNICALL Java_com_studica_frc_Cobra_isAvailable(
    JNIEnv*, jclass, jlong handle) {
  uint8_t value = 0;
  StudicaCobra_IsAvailable(static_cast<StudicaCobraHandle>(handle), &value);
  return value != 0;
}

}  // extern "C"
