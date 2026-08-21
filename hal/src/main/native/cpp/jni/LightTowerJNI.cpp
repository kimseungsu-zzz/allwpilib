// Copyright (c) 2026 WPILib contributors.

#include <jni.h>

#include "studica/LightTower.h"

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

JNIEXPORT jlongArray JNICALL Java_com_studica_frc_LightTower_create(
    JNIEnv* env, jclass, jint continuous, jint red, jint green, jint yellow,
    jint buzzer) {
  StudicaLightTowerHandle handle = 0;
  const auto status = StudicaLightTower_Create(
      static_cast<uint8_t>(continuous), static_cast<uint8_t>(red),
      static_cast<uint8_t>(green), static_cast<uint8_t>(yellow),
      static_cast<uint8_t>(buzzer), &handle);
  return Pair(env, static_cast<jlong>(handle), status);
}
JNIEXPORT void JNICALL Java_com_studica_frc_LightTower_destroy(
    JNIEnv*, jclass, jlong handle) {
  StudicaLightTower_Destroy(static_cast<StudicaLightTowerHandle>(handle));
}
#define LIGHT_TOWER_JNI_BOOL(name, function)                                  \
  JNIEXPORT jint JNICALL Java_com_studica_frc_LightTower_##name(              \
      JNIEnv*, jclass, jlong handle, jboolean value) {                        \
    return function(static_cast<StudicaLightTowerHandle>(handle),             \
                    static_cast<uint8_t>(value));                             \
  }
LIGHT_TOWER_JNI_BOOL(setRed, StudicaLightTower_SetRed)
LIGHT_TOWER_JNI_BOOL(setYellow, StudicaLightTower_SetYellow)
LIGHT_TOWER_JNI_BOOL(setGreen, StudicaLightTower_SetGreen)
LIGHT_TOWER_JNI_BOOL(setBuzzer, StudicaLightTower_SetBuzzer)
#undef LIGHT_TOWER_JNI_BOOL

JNIEXPORT jint JNICALL Java_com_studica_frc_LightTower_setSolid(
    JNIEnv*, jclass, jlong handle) {
  return StudicaLightTower_SetSolid(
      static_cast<StudicaLightTowerHandle>(handle));
}
JNIEXPORT jint JNICALL Java_com_studica_frc_LightTower_setBlink(
    JNIEnv*, jclass, jlong handle) {
  return StudicaLightTower_SetBlink(
      static_cast<StudicaLightTowerHandle>(handle));
}
JNIEXPORT jint JNICALL Java_com_studica_frc_LightTower_off(JNIEnv*, jclass,
                                                            jlong handle) {
  return StudicaLightTower_Off(
      static_cast<StudicaLightTowerHandle>(handle));
}
JNIEXPORT jboolean JNICALL Java_com_studica_frc_LightTower_isAvailable(
    JNIEnv*, jclass, jlong handle) {
  uint8_t value = 0;
  StudicaLightTower_IsAvailable(
      static_cast<StudicaLightTowerHandle>(handle), &value);
  return value != 0;
}

}  // extern "C"
