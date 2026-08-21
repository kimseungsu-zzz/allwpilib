// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify and/or share it under the WPILib
// BSD license file in the root directory of this project.

#include <jni.h>

#include "studica/VMXIMU.h"
#include "studica_vmx_VMXIMU.h"

extern "C" {

JNIEXPORT jlong JNICALL Java_studica_vmx_VMXIMU_create(JNIEnv*, jclass) {
  StudicaVMXIMUHandle handle = 0;
  if (StudicaVMXIMU_Create(&handle) != STUDICA_VMX_IMU_OK) {
    return 0;
  }
  return static_cast<jlong>(handle);
}

JNIEXPORT void JNICALL Java_studica_vmx_VMXIMU_destroy(JNIEnv*, jclass,
                                                       jlong handle) {
  StudicaVMXIMU_Destroy(static_cast<StudicaVMXIMUHandle>(handle));
}

JNIEXPORT jbyteArray JNICALL Java_studica_vmx_VMXIMU_readSnapshot(
    JNIEnv* env, jclass, jlong handle) {
  StudicaVMXIMUSnapshot snapshot;
  if (StudicaVMXIMU_ReadSnapshot(
          static_cast<StudicaVMXIMUHandle>(handle), &snapshot) !=
      STUDICA_VMX_IMU_OK) {
    return nullptr;
  }
  auto result = env->NewByteArray(static_cast<jsize>(sizeof(snapshot)));
  if (!result) {
    return nullptr;
  }
  env->SetByteArrayRegion(result, 0, static_cast<jsize>(sizeof(snapshot)),
                          reinterpret_cast<const jbyte*>(&snapshot));
  return result;
}

JNIEXPORT jint JNICALL Java_studica_vmx_VMXIMU_zeroYaw(JNIEnv*, jclass,
                                                        jlong handle) {
  return static_cast<jint>(StudicaVMXIMU_ZeroYaw(
      static_cast<StudicaVMXIMUHandle>(handle)));
}

JNIEXPORT jint JNICALL Java_studica_vmx_VMXIMU_reset(JNIEnv*, jclass,
                                                      jlong handle) {
  return static_cast<jint>(StudicaVMXIMU_Reset(
      static_cast<StudicaVMXIMUHandle>(handle)));
}

}  // extern "C"
