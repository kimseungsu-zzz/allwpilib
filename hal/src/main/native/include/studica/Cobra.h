// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of the WPILib
// BSD license file in the root directory of this project.

#pragma once

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

// Stable vendor ABI for the four-channel Studica Cobra reflectance array.
// The implementation is an adapter around the immutable Studica driver; it
// deliberately does not turn Cobra into a WPILib I2C device class.
#define STUDICA_COBRA_ABI_VERSION 1u
#define STUDICA_COBRA_CHANNEL_COUNT 4u
#define STUDICA_COBRA_DEFAULT_REFERENCE_VOLTAGE 5.0

typedef uint32_t StudicaCobraHandle;

#ifdef __cplusplus
enum StudicaCobraStatus : int32_t {
#else
typedef enum StudicaCobraStatus {
#endif
  STUDICA_COBRA_OK = 0,
  STUDICA_COBRA_INVALID_ARGUMENT = -22,
  STUDICA_COBRA_UNAVAILABLE = -19,
  STUDICA_COBRA_NOT_INITIALIZED = -107,
  STUDICA_COBRA_RESOURCE_CONFLICT = -16,
  STUDICA_COBRA_INTERNAL_ERROR = -5,
#ifdef __cplusplus
};
#else
} StudicaCobraStatus;
#endif

#ifdef __cplusplus
namespace studica {

class Cobra final {
 public:
  explicit Cobra(double referenceVoltage =
                     STUDICA_COBRA_DEFAULT_REFERENCE_VOLTAGE) noexcept;
  ~Cobra();

  Cobra(const Cobra&) = delete;
  Cobra& operator=(const Cobra&) = delete;
  Cobra(Cobra&& other) noexcept;
  Cobra& operator=(Cobra&& other) noexcept;

  int32_t GetRaw(uint8_t channel) const noexcept;
  double GetVoltage(uint8_t channel) const noexcept;
  uint8_t GetChannelCount() const noexcept;
  double GetReferenceVoltage() const noexcept;
  bool IsAvailable() const noexcept;
  int32_t GetLastStatus() const noexcept;

 private:
  StudicaCobraHandle m_handle = 0;
  mutable int32_t m_lastStatus = STUDICA_COBRA_NOT_INITIALIZED;
};

}  // namespace studica

extern "C" {
#endif

int32_t StudicaCobra_Create(double referenceVoltage,
                           StudicaCobraHandle* handleOut);
void StudicaCobra_Destroy(StudicaCobraHandle handle);
int32_t StudicaCobra_GetRaw(StudicaCobraHandle handle, uint8_t channel,
                            int32_t* rawOut);
int32_t StudicaCobra_GetVoltage(StudicaCobraHandle handle, uint8_t channel,
                                double* voltageOut);
int32_t StudicaCobra_GetChannelCount(StudicaCobraHandle handle,
                                     uint8_t* countOut);
int32_t StudicaCobra_GetReferenceVoltage(StudicaCobraHandle handle,
                                         double* voltageOut);
int32_t StudicaCobra_IsAvailable(StudicaCobraHandle handle,
                                 uint8_t* availableOut);
void StudicaCobra_ShutdownAll(void);

#ifdef __cplusplus
}
#endif
