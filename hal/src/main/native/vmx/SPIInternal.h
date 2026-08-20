// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include "DigitalChannelRegistry.h"
#include "VMXChannelCapabilities.h"
#include "hal/SPITypes.h"

namespace hal::vmx {

enum class SPIResult {
  kOk,
  kPortOutOfRange,
  kUnsupportedPort,
  kNotInitialized,
  kInvalidSize,
  kNullPointer,
  kInvalidClockRate,
  kInvalidMode,
  kUnsupportedConfig,
  kNoResources,
  kResourceConflict,
  kHardwareFailure,
  kAutoUnsupported,
};

struct SPIPortConfig {
  int32_t clockRate = 500000;
  HAL_SPIMode mode = HAL_SPI_kMode0;
  bool chipSelectActiveLow = true;

  friend constexpr bool operator==(const SPIPortConfig& lhs,
                                   const SPIPortConfig& rhs) noexcept {
    return lhs.clockRate == rhs.clockRate && lhs.mode == rhs.mode &&
           lhs.chipSelectActiveLow == rhs.chipSelectActiveLow;
  }
};

constexpr int32_t kVMXSPIMinClockRate = 500000;
constexpr int32_t kVMXSPIMaxClockRate = 10000000;
constexpr int32_t kVMXSPIChannelCount = 4;

constexpr SPIResult ValidateSPIPort(HAL_SPIPort port) noexcept {
  if (port < HAL_SPI_kOnboardCS0 || port > HAL_SPI_kMXP) {
    return SPIResult::kPortOutOfRange;
  }
  // VMX has one physical SPI connector.  All WPILib onboard CS0..CS3 and
  // MXP identifiers are aliases for that bus and share one registry claim.
  return SPIResult::kOk;
}

constexpr bool IsValidSPIConfig(const SPIPortConfig& config) noexcept {
  const auto mode = static_cast<int32_t>(config.mode);
  return config.clockRate >= kVMXSPIMinClockRate &&
         config.clockRate <= kVMXSPIMaxClockRate && mode >= 0 && mode <= 3;
}

class SPIBackend {
 public:
  virtual ~SPIBackend() = default;

  virtual bool Transaction(uint8_t* dataToSend, uint8_t* dataReceived,
                           uint16_t size) noexcept = 0;
  virtual bool Write(uint8_t* dataToSend, uint16_t size) noexcept = 0;
  virtual bool Read(uint8_t* dataReceived, uint16_t size) noexcept = 0;
  virtual bool Reconfigure(const SPIPortConfig& config) noexcept = 0;
  virtual int32_t GetHandle() const noexcept { return 0; }
};

using SPIBackendFactory = std::function<std::unique_ptr<SPIBackend>(
    HAL_SPIPort, const VMXCommDIOChannelMap&, const SPIPortConfig&)>;
using SPICommDIOMapProvider = std::function<VMXCommDIOChannelMap()>;

struct SPIPortState {
  mutable std::mutex mutex;
  bool initialized = false;
  int32_t referenceCount = 0;
  int32_t handle = 0;
  SPIPortConfig config;
  VMXCommDIOChannelMap channels;
  std::unique_ptr<SPIBackend> backend;
};

class SPIManager final {
 public:
  explicit SPIManager(
      SPIBackendFactory factory = {},
      DigitalChannelRegistry& registry = GetDigitalChannelRegistry(),
      SPICommDIOMapProvider mapProvider =
          [] { return kDefaultVMXCommDIOChannelMap; })
      : m_factory{std::move(factory)},
        m_registry{registry},
        m_mapProvider{std::move(mapProvider)} {}

  ~SPIManager() {
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    ReleaseChannels(state);
    state.backend.reset();
    ResetState(state);
  }

  SPIResult Initialize(HAL_SPIPort port) noexcept {
    auto validation = ValidateSPIPort(port);
    if (validation != SPIResult::kOk) {
      return validation;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (state.initialized) {
      ++state.referenceCount;
      return SPIResult::kOk;
    }

    VMXCommDIOChannelMap channels;
    try {
      channels = m_mapProvider ? m_mapProvider()
                               : kDefaultVMXCommDIOChannelMap;
    } catch (...) {
      return SPIResult::kHardwareFailure;
    }
    if (!IsValidChannelMap(channels)) {
      return SPIResult::kUnsupportedConfig;
    }
    if (!ReserveChannels(channels)) {
      return SPIResult::kResourceConflict;
    }

    std::unique_ptr<SPIBackend> backend;
    try {
      backend = m_factory ? m_factory(port, channels, state.config) : nullptr;
    } catch (...) {
      backend.reset();
    }
    if (!backend) {
      ReleaseChannels(state, channels);
      return SPIResult::kHardwareFailure;
    }

    state.channels = channels;
    state.backend = std::move(backend);
    state.initialized = true;
    state.referenceCount = 1;
    state.handle = state.backend->GetHandle();
    if (state.handle == 0) {
      // A backend may not expose the SDK handle (host mocks do not need one),
      // but HAL_GetSPIHandle still needs a stable nonzero value.
      state.handle = static_cast<int32_t>(port) + 1;
    }
    return SPIResult::kOk;
  }

  SPIResult Close(HAL_SPIPort port) noexcept {
    auto validation = ValidateSPIPort(port);
    if (validation != SPIResult::kOk) {
      return validation;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (!state.initialized) {
      return SPIResult::kOk;
    }
    if (state.referenceCount > 1) {
      --state.referenceCount;
      return SPIResult::kOk;
    }
    state.backend.reset();
    ReleaseChannels(state);
    ResetState(state);
    return SPIResult::kOk;
  }

  SPIResult Transaction(HAL_SPIPort port, const uint8_t* dataToSend,
                        uint8_t* dataReceived, int32_t size,
                        int32_t& transferred) noexcept {
    transferred = -1;
    auto validation = ValidateTransfer(port, size, dataToSend, dataReceived,
                                       true, true);
    if (validation != SPIResult::kOk) {
      return validation;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (!state.initialized || !state.backend) {
      return SPIResult::kNotInitialized;
    }
    bool success = false;
    try {
      success = state.backend->Transaction(
          const_cast<uint8_t*>(dataToSend), dataReceived,
          static_cast<uint16_t>(size));
    } catch (...) {
      success = false;
    }
    if (!success) {
      return SPIResult::kHardwareFailure;
    }
    transferred = size;
    return SPIResult::kOk;
  }

  SPIResult Write(HAL_SPIPort port, const uint8_t* dataToSend, int32_t size,
                  int32_t& transferred) noexcept {
    transferred = -1;
    auto validation = ValidateTransfer(port, size, dataToSend, nullptr, true,
                                       false);
    if (validation != SPIResult::kOk) {
      return validation;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (!state.initialized || !state.backend) {
      return SPIResult::kNotInitialized;
    }
    bool success = false;
    try {
      success = state.backend->Write(const_cast<uint8_t*>(dataToSend),
                                     static_cast<uint16_t>(size));
    } catch (...) {
      success = false;
    }
    if (!success) {
      return SPIResult::kHardwareFailure;
    }
    transferred = size;
    return SPIResult::kOk;
  }

  SPIResult Read(HAL_SPIPort port, uint8_t* dataReceived, int32_t size,
                 int32_t& transferred) noexcept {
    transferred = -1;
    auto validation = ValidateTransfer(port, size, nullptr, dataReceived,
                                       false, true);
    if (validation != SPIResult::kOk) {
      return validation;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (!state.initialized || !state.backend) {
      return SPIResult::kNotInitialized;
    }
    bool success = false;
    try {
      success = state.backend->Read(dataReceived, static_cast<uint16_t>(size));
    } catch (...) {
      success = false;
    }
    if (!success) {
      return SPIResult::kHardwareFailure;
    }
    transferred = size;
    return SPIResult::kOk;
  }

  SPIResult SetClockRate(HAL_SPIPort port, int32_t clockRate) noexcept {
    auto validation = ValidateSPIPort(port);
    if (validation != SPIResult::kOk) {
      return validation;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (!state.initialized || !state.backend) {
      return SPIResult::kNotInitialized;
    }
    if (clockRate < kVMXSPIMinClockRate || clockRate > kVMXSPIMaxClockRate) {
      return SPIResult::kInvalidClockRate;
    }
    auto updated = state.config;
    updated.clockRate = clockRate;
    return Reconfigure(state, updated);
  }

  SPIResult SetMode(HAL_SPIPort port, HAL_SPIMode mode) noexcept {
    auto validation = ValidateSPIPort(port);
    if (validation != SPIResult::kOk) {
      return validation;
    }
    if (static_cast<int32_t>(mode) < 0 || static_cast<int32_t>(mode) > 3) {
      return SPIResult::kInvalidMode;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (!state.initialized || !state.backend) {
      return SPIResult::kNotInitialized;
    }
    auto updated = state.config;
    updated.mode = mode;
    return Reconfigure(state, updated);
  }

  HAL_SPIMode GetMode(HAL_SPIPort port) const noexcept {
    if (ValidateSPIPort(port) != SPIResult::kOk) {
      return HAL_SPI_kMode0;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    return state.config.mode;
  }

  SPIResult SetChipSelectActiveLow(HAL_SPIPort port, bool activeLow) noexcept {
    auto validation = ValidateSPIPort(port);
    if (validation != SPIResult::kOk) {
      return validation;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (!state.initialized || !state.backend) {
      return SPIResult::kNotInitialized;
    }
    auto updated = state.config;
    updated.chipSelectActiveLow = activeLow;
    return Reconfigure(state, updated);
  }

  int32_t GetHandle(HAL_SPIPort port) const noexcept {
    if (ValidateSPIPort(port) != SPIResult::kOk) {
      return 0;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    return state.initialized ? state.handle : 0;
  }

  void SetHandle(HAL_SPIPort port, int32_t handle) noexcept {
    if (ValidateSPIPort(port) != SPIResult::kOk) {
      return;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (state.initialized) {
      state.handle = handle;
    }
  }

  SPIResult AutoUnsupported(HAL_SPIPort port) const noexcept {
    auto validation = ValidateSPIPort(port);
    return validation == SPIResult::kOk ? SPIResult::kAutoUnsupported
                                        : validation;
  }

 private:
  static bool IsValidChannelMap(const VMXCommDIOChannelMap& map) noexcept {
    if (!map.valid || !map.spiValid) {
      return false;
    }
    const std::array<int32_t, kVMXSPIChannelCount> channels{
        map.spiCLK, map.spiMOSI, map.spiMISO, map.spiCS};
    for (auto channel : channels) {
      if (!IsPhysicalChannelValid(channel)) {
        return false;
      }
    }
    return channels[0] != channels[1] && channels[0] != channels[2] &&
           channels[0] != channels[3] && channels[1] != channels[2] &&
           channels[1] != channels[3] && channels[2] != channels[3];
  }

  bool ReserveChannels(const VMXCommDIOChannelMap& map) {
    const std::array<int32_t, kVMXSPIChannelCount> channels{
        map.spiCLK, map.spiMOSI, map.spiMISO, map.spiCS};
    size_t reserved = 0;
    for (; reserved < channels.size(); ++reserved) {
      if (!m_registry
               .Reserve(channels[reserved], DigitalChannelOwner::kSPI,
                       "VMX SPI")
               .reserved) {
        for (size_t i = 0; i < reserved; ++i) {
          m_registry.Release(channels[i], DigitalChannelOwner::kSPI);
        }
        return false;
      }
    }
    return true;
  }

  void ReleaseChannels(const SPIPortState& state) noexcept {
    if (!state.initialized) {
      return;
    }
    ReleaseChannels(*this, state.channels);
  }

  void ReleaseChannels(const VMXCommDIOChannelMap& map) noexcept {
    ReleaseChannels(*this, map);
  }

  static void ReleaseChannels(SPIManager& manager,
                              const VMXCommDIOChannelMap& map) noexcept {
    for (auto channel : {map.spiCLK, map.spiMOSI, map.spiMISO, map.spiCS}) {
      manager.m_registry.Release(channel, DigitalChannelOwner::kSPI);
    }
  }

  void ReleaseChannels(const SPIPortState& state,
                       const VMXCommDIOChannelMap& map) noexcept {
    static_cast<void>(state);
    ReleaseChannels(*this, map);
  }

  static void ResetState(SPIPortState& state) noexcept {
    state.initialized = false;
    state.referenceCount = 0;
    state.handle = 0;
    state.config = {};
    state.channels = {};
    state.backend.reset();
  }

  SPIResult Reconfigure(SPIPortState& state,
                        const SPIPortConfig& updated) noexcept {
    if (updated == state.config) {
      return SPIResult::kOk;
    }
    if (!IsValidSPIConfig(updated)) {
      return static_cast<int32_t>(updated.mode) < 0 ||
                     static_cast<int32_t>(updated.mode) > 3
                 ? SPIResult::kInvalidMode
                 : SPIResult::kInvalidClockRate;
    }
    bool success = false;
    try {
      success = state.backend->Reconfigure(updated);
    } catch (...) {
      success = false;
    }
    if (!success) {
      return SPIResult::kHardwareFailure;
    }
    state.config = updated;
    return SPIResult::kOk;
  }

  static SPIResult ValidateTransfer(HAL_SPIPort port, int32_t size,
                                    const uint8_t* sendData,
                                    const uint8_t* receiveData, bool checkSend,
                                    bool checkReceive) noexcept {
    auto validation = ValidateSPIPort(port);
    if (validation != SPIResult::kOk) {
      return validation;
    }
    if (size < 0 || size > UINT16_MAX) {
      return SPIResult::kInvalidSize;
    }
    if (size > 0 && ((checkSend && !sendData) ||
                     (checkReceive && !receiveData))) {
      return SPIResult::kNullPointer;
    }
    return SPIResult::kOk;
  }

  SPIBackendFactory m_factory;
  DigitalChannelRegistry& m_registry;
  SPICommDIOMapProvider m_mapProvider;
  // One physical VMX SPI bus shared by all five WPILib port aliases.
  mutable SPIPortState m_port;
};

}  // namespace hal::vmx
