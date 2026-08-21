// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of the WPILib
// BSD license file in the root directory of this project.

#pragma once

#ifdef __cplusplus
#include <cstdint>
#include <string>
#else
#include <stdint.h>
#endif
#define STUDICA_COLORE_ABI_VERSION 1u
#define STUDICA_COLORE_LABEL_CAPACITY 32u

typedef uint32_t StudicaColoreHandle;

#ifdef __cplusplus
enum StudicaColoreStatus : int32_t {
#else
typedef enum StudicaColoreStatus {
#endif
  STUDICA_COLORE_OK = 0,
  STUDICA_COLORE_INVALID_ARGUMENT = -22,
  STUDICA_COLORE_UNAVAILABLE = -19,
  STUDICA_COLORE_UNSUPPORTED = -95,
  STUDICA_COLORE_NOT_INITIALIZED = -107,
  STUDICA_COLORE_TIMEOUT = -110,
  STUDICA_COLORE_RESOURCE_CONFLICT = -16,
  STUDICA_COLORE_INTERNAL_ERROR = -5,
#ifdef __cplusplus
};
#else
} StudicaColoreStatus;
#endif

typedef struct StudicaColoreSnapshot {
  uint32_t structSize;
  uint32_t abiVersion;
  float red;
  float green;
  float blue;
  float x;
  float y;
  float z;
  float chromaticityX;
  float chromaticityY;
  uint64_t sequence;
  uint32_t flags;
  uint8_t connected;
  uint8_t valid;
  uint8_t transport;
  uint8_t reserved;
} StudicaColoreSnapshot;

typedef struct StudicaColoreConfig {
  uint32_t structSize;
  uint32_t abiVersion;
  uint32_t transport;
  uint32_t canId;
  uint32_t brightness;
  uint32_t colorFormat;
  uint32_t rawLength;
  uint8_t raw[128];
} StudicaColoreConfig;

typedef struct StudicaColoreMatchResult {
  uint32_t structSize;
  char label[STUDICA_COLORE_LABEL_CAPACITY];
  float confidence;
  float measuredX;
  float measuredY;
  uint8_t valid;
  uint8_t reserved[3];
} StudicaColoreMatchResult;

#ifdef __cplusplus
namespace studica {

/** Vendor wrapper for Studica's CIE-XYZ/sRGB Colore sensor. */
class Colore final {
 public:
  explicit Colore(uint8_t canId) noexcept;
  explicit Colore(std::string usbPath) noexcept;
  ~Colore();

  Colore(const Colore&) = delete;
  Colore& operator=(const Colore&) = delete;
  Colore(Colore&& other) noexcept;
  Colore& operator=(Colore&& other) noexcept;

  bool Read() noexcept;
  StudicaColoreSnapshot GetSnapshot() const noexcept;
  float GetRed() const noexcept;
  float GetGreen() const noexcept;
  float GetBlue() const noexcept;
  float GetX() const noexcept;
  float GetY() const noexcept;
  float GetZ() const noexcept;
  bool SetBrightness(uint8_t percent) noexcept;
  uint8_t GetBrightness() const noexcept;
  bool GetConfig(StudicaColoreConfig* configOut) const noexcept;
  bool LearnColor(const std::string& name, float threshold = 0.05F) noexcept;
  bool SetReference(const std::string& name, float x, float y,
                   float threshold = 0.05F) noexcept;
  bool GetLearnedReference(const std::string& name,
                          StudicaColoreMatchResult* resultOut) const noexcept;
  bool Match(StudicaColoreMatchResult* resultOut) const noexcept;
  bool IsConnected() const noexcept;
  int32_t GetLastStatus() const noexcept;

 private:
  StudicaColoreHandle m_handle = 0;
  mutable int32_t m_lastStatus = STUDICA_COLORE_NOT_INITIALIZED;
};

}  // namespace studica

extern "C" {
#endif

int32_t StudicaColore_CreateCAN(uint8_t canId, StudicaColoreHandle* handleOut);
int32_t StudicaColore_CreateUSB(const char* path,
                                StudicaColoreHandle* handleOut);
void StudicaColore_Destroy(StudicaColoreHandle handle);
int32_t StudicaColore_Read(StudicaColoreHandle handle,
                           StudicaColoreSnapshot* snapshotOut);
int32_t StudicaColore_GetRed(StudicaColoreHandle handle, float* valueOut);
int32_t StudicaColore_GetGreen(StudicaColoreHandle handle, float* valueOut);
int32_t StudicaColore_GetBlue(StudicaColoreHandle handle, float* valueOut);
int32_t StudicaColore_GetX(StudicaColoreHandle handle, float* valueOut);
int32_t StudicaColore_GetY(StudicaColoreHandle handle, float* valueOut);
int32_t StudicaColore_GetZ(StudicaColoreHandle handle, float* valueOut);
int32_t StudicaColore_SetBrightness(StudicaColoreHandle handle,
                                    int32_t percent);
int32_t StudicaColore_GetBrightness(StudicaColoreHandle handle,
                                    uint8_t* percentOut);
int32_t StudicaColore_GetConfig(StudicaColoreHandle handle,
                                StudicaColoreConfig* configOut);
int32_t StudicaColore_LearnColor(StudicaColoreHandle handle, const char* name,
                                 float threshold);
int32_t StudicaColore_SetReference(StudicaColoreHandle handle, const char* name,
                                   float x, float y, float threshold);
int32_t StudicaColore_GetLearnedReference(
    StudicaColoreHandle handle, const char* name,
    StudicaColoreMatchResult* resultOut);
int32_t StudicaColore_Match(StudicaColoreHandle handle,
                            StudicaColoreMatchResult* resultOut);
int32_t StudicaColore_IsConnected(StudicaColoreHandle handle,
                                  uint8_t* connectedOut);
int32_t StudicaColore_GetLastStatus(StudicaColoreHandle handle,
                                    int32_t* statusOut);
void StudicaColore_ShutdownAll(void);

/* Host-only fixture hook. Learned references are intentionally in-memory. */
int32_t StudicaColore_SetMockSnapshot(StudicaColoreHandle handle,
                                      const StudicaColoreSnapshot* snapshot);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
static_assert(sizeof(StudicaColoreHandle) == sizeof(uint32_t));
#endif
