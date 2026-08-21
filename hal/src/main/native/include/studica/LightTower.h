// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of the WPILib
// BSD license file in the root directory of this project.

#pragma once

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

// Stable vendor ABI for the Studica five-output Light Tower.  Pin numbers are
// VMX physical DIO channels, not WPILib logical aliases.  The adapter owns
// resource claims and delegates all output operations to the immutable
// studica_driver::LightTower class.
#define STUDICA_LIGHT_TOWER_ABI_VERSION 1u

typedef uint32_t StudicaLightTowerHandle;

#ifdef __cplusplus
enum StudicaLightTowerStatus : int32_t {
#else
typedef enum StudicaLightTowerStatus {
#endif
  STUDICA_LIGHT_TOWER_OK = 0,
  STUDICA_LIGHT_TOWER_INVALID_ARGUMENT = -22,
  STUDICA_LIGHT_TOWER_UNAVAILABLE = -19,
  STUDICA_LIGHT_TOWER_NOT_INITIALIZED = -107,
  STUDICA_LIGHT_TOWER_RESOURCE_CONFLICT = -16,
  STUDICA_LIGHT_TOWER_INTERNAL_ERROR = -5,
#ifdef __cplusplus
};
#else
} StudicaLightTowerStatus;
#endif

#ifdef __cplusplus
namespace studica {

class LightTower final {
 public:
  LightTower(uint8_t continuous, uint8_t red, uint8_t green, uint8_t yellow,
             uint8_t buzzer) noexcept;
  ~LightTower();

  LightTower(const LightTower&) = delete;
  LightTower& operator=(const LightTower&) = delete;
  LightTower(LightTower&& other) noexcept;
  LightTower& operator=(LightTower&& other) noexcept;

  bool SetRed(bool enabled) noexcept;
  bool SetYellow(bool enabled) noexcept;
  bool SetGreen(bool enabled) noexcept;
  bool SetBuzzer(bool enabled) noexcept;
  bool SetSolid() noexcept;
  bool SetBlink() noexcept;
  bool Off() noexcept;
  bool IsAvailable() const noexcept;
  int32_t GetLastStatus() const noexcept;

 private:
  StudicaLightTowerHandle m_handle = 0;
  mutable int32_t m_lastStatus = STUDICA_LIGHT_TOWER_NOT_INITIALIZED;
};

}  // namespace studica

extern "C" {
#endif

int32_t StudicaLightTower_Create(uint8_t continuous, uint8_t red,
                                 uint8_t green, uint8_t yellow,
                                 uint8_t buzzer,
                                 StudicaLightTowerHandle* handleOut);
void StudicaLightTower_Destroy(StudicaLightTowerHandle handle);
int32_t StudicaLightTower_SetRed(StudicaLightTowerHandle handle,
                                 uint8_t enabled);
int32_t StudicaLightTower_SetYellow(StudicaLightTowerHandle handle,
                                    uint8_t enabled);
int32_t StudicaLightTower_SetGreen(StudicaLightTowerHandle handle,
                                   uint8_t enabled);
int32_t StudicaLightTower_SetBuzzer(StudicaLightTowerHandle handle,
                                    uint8_t enabled);
int32_t StudicaLightTower_SetSolid(StudicaLightTowerHandle handle);
int32_t StudicaLightTower_SetBlink(StudicaLightTowerHandle handle);
int32_t StudicaLightTower_Off(StudicaLightTowerHandle handle);
int32_t StudicaLightTower_IsAvailable(StudicaLightTowerHandle handle,
                                      uint8_t* availableOut);
void StudicaLightTower_ShutdownAll(void);

#ifdef __cplusplus
}
#endif
