// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of the WPILib
// BSD license file in the root directory of this project.

#pragma once

#ifdef __cplusplus
#include <array>
#include <cstdint>
#include <string>
#else
#include <stdint.h>
#endif
#define STUDICA_PARSEC_ABI_VERSION 1u
#define STUDICA_PARSEC_MAX_ZONES 64u
#define STUDICA_PARSEC_MIN_CAN_ID 0u
#define STUDICA_PARSEC_MAX_CAN_ID 63u
#define STUDICA_PARSEC_RESOLUTION_4 4u
#define STUDICA_PARSEC_RESOLUTION_8 8u

typedef uint32_t StudicaParsecHandle;

#ifdef __cplusplus
enum StudicaParsecStatus : int32_t {
#else
typedef enum StudicaParsecStatus {
#endif
  STUDICA_PARSEC_OK = 0,
  STUDICA_PARSEC_INVALID_ARGUMENT = -22,
  STUDICA_PARSEC_UNAVAILABLE = -19,
  STUDICA_PARSEC_UNSUPPORTED = -95,
  STUDICA_PARSEC_NOT_INITIALIZED = -107,
  STUDICA_PARSEC_TIMEOUT = -110,
  STUDICA_PARSEC_RESOURCE_CONFLICT = -16,
  STUDICA_PARSEC_BUFFER_TOO_SMALL = -75,
  STUDICA_PARSEC_INTERNAL_ERROR = -5,
#ifdef __cplusplus
};
#else
} StudicaParsecStatus;
#endif

typedef struct StudicaParsecSnapshot {
  uint32_t structSize;
  uint32_t abiVersion;
  uint32_t resolution;
  uint32_t zoneCount;
  uint64_t sequence;
  int16_t distances[STUDICA_PARSEC_MAX_ZONES];
  uint8_t connected;
  uint8_t valid;
  uint8_t transport;
  uint8_t reserved;
} StudicaParsecSnapshot;

typedef struct StudicaParsecConfig {
  uint32_t structSize;
  uint32_t abiVersion;
  uint32_t transport;
  uint32_t resolution;
  uint32_t zoneCount;
  uint8_t canId;
  uint8_t connected;
  uint16_t rawLength;
  uint64_t zoneMask;
  uint8_t raw[64];
} StudicaParsecConfig;

#ifdef __cplusplus
namespace studica {

/** Vendor wrapper for the Studica Parsec 4x4/8x8 distance sensor. */
class Parsec final {
 public:
  explicit Parsec(uint8_t canId) noexcept;
  explicit Parsec(std::string usbPath) noexcept;
  ~Parsec();

  Parsec(const Parsec&) = delete;
  Parsec& operator=(const Parsec&) = delete;
  Parsec(Parsec&& other) noexcept;
  Parsec& operator=(Parsec&& other) noexcept;

  uint32_t GetResolution() const noexcept;
  uint32_t GetZoneCount() const noexcept;
  int16_t GetDistance(uint32_t zone) const noexcept;
  std::array<int16_t, STUDICA_PARSEC_MAX_ZONES> GetDistances() const noexcept;
  bool GetMinDistance(int16_t* distanceOut) const noexcept;
  bool Read() noexcept;
  bool GetConfig(StudicaParsecConfig* configOut) const noexcept;
  bool IsConnected() const noexcept;
  int32_t GetLastStatus() const noexcept;

 private:
  StudicaParsecHandle m_handle = 0;
  mutable int32_t m_lastStatus = STUDICA_PARSEC_NOT_INITIALIZED;
};

}  // namespace studica

extern "C" {
#endif

int32_t StudicaParsec_CreateCAN(uint8_t canId, StudicaParsecHandle* handleOut);
int32_t StudicaParsec_CreateUSB(const char* path,
                                StudicaParsecHandle* handleOut);
void StudicaParsec_Destroy(StudicaParsecHandle handle);
int32_t StudicaParsec_ReadZones(StudicaParsecHandle handle,
                                StudicaParsecSnapshot* snapshotOut);
int32_t StudicaParsec_GetResolution(StudicaParsecHandle handle,
                                    uint32_t* resolutionOut);
int32_t StudicaParsec_GetZoneCount(StudicaParsecHandle handle,
                                   uint32_t* zoneCountOut);
int32_t StudicaParsec_GetZoneDistance(StudicaParsecHandle handle,
                                      uint32_t zone, int16_t* distanceOut);
int32_t StudicaParsec_GetMinDistance(StudicaParsecHandle handle,
                                     int16_t* distanceOut, uint8_t* validOut);
int32_t StudicaParsec_GetDistances(StudicaParsecHandle handle,
                                   int16_t* distancesOut,
                                   uint32_t capacity);
int32_t StudicaParsec_GetConfig(StudicaParsecHandle handle,
                                StudicaParsecConfig* configOut);
int32_t StudicaParsec_IsConnected(StudicaParsecHandle handle,
                                  uint8_t* connectedOut);
int32_t StudicaParsec_GetLastStatus(StudicaParsecHandle handle,
                                    int32_t* statusOut);
void StudicaParsec_ShutdownAll(void);

/* Host-only fixture hooks. They are not hardware APIs and never exist on VMX. */
int32_t StudicaParsec_SetMockSnapshot(StudicaParsecHandle handle,
                                      const StudicaParsecSnapshot* snapshot);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
static_assert(sizeof(StudicaParsecHandle) == sizeof(uint32_t));
static_assert(sizeof(StudicaParsecSnapshot) >=
              sizeof(uint32_t) * 4 + sizeof(uint64_t) +
                  sizeof(int16_t) * STUDICA_PARSEC_MAX_ZONES + 4);
#endif
