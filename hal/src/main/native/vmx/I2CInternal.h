// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include "hal/I2CTypes.h"
#include "DigitalChannelRegistry.h"
#include "VMXChannelCapabilities.h"

namespace hal::vmx {

enum class I2CResult {
  kOk,
  kPortOutOfRange,
  kUnsupportedPort,
  kInvalidAddress,
  kInvalidSize,
  kNullPointer,
  kNotInitialized,
  kNoResources,
  kResourceConflict,
  kHardwareFailure,
};

class I2CBackend {
 public:
  virtual ~I2CBackend() = default;

  // The HAL transaction uses the WPILib buffer semantics directly.  The
  // backend adapts those buffers to the VMX SDK's blocking calls.
  virtual bool Transaction(uint8_t deviceAddress, const uint8_t* dataToSend,
                           uint16_t sendSize, uint8_t* dataReceived,
                           uint16_t receiveSize) noexcept = 0;
  virtual bool Write(uint8_t deviceAddress, const uint8_t* dataToSend,
                     int32_t sendSize) noexcept = 0;
  virtual bool Read(uint8_t deviceAddress, uint8_t* dataReceived,
                    int32_t receiveSize) noexcept = 0;

  virtual bool GetPhysicalChannels(int32_t& sda,
                                   int32_t& scl) const noexcept {
    static_cast<void>(sda);
    static_cast<void>(scl);
    return false;
  }
};

using I2CBackendFactory = std::function<std::unique_ptr<I2CBackend>()>;
using CommDIOMapProvider = std::function<VMXCommDIOChannelMap()>;

struct I2CPortState {
  std::mutex mutex;
  int32_t referenceCount = 0;
  int32_t sdaChannel = -1;
  int32_t sclChannel = -1;
  std::unique_ptr<I2CBackend> backend;
};

constexpr I2CResult ValidateI2CPort(HAL_I2CPort port) noexcept {
  if (port < HAL_I2C_kOnboard || port > HAL_I2C_kMXP) {
    return I2CResult::kPortOutOfRange;
  }
  // VMX exposes one physical I2C connector.  WPILib's onboard and MXP
  // identifiers are compatibility aliases for that same resource and must
  // therefore share one lifecycle/registry reservation.
  return I2CResult::kOk;
}

class I2CManager final {
 public:
  explicit I2CManager(
      I2CBackendFactory factory = {},
      DigitalChannelRegistry& registry = GetDigitalChannelRegistry(),
      CommDIOMapProvider mapProvider =
          [] { return kDefaultVMXCommDIOChannelMap; })
      : m_factory{std::move(factory)},
        m_registry{registry},
        m_mapProvider{std::move(mapProvider)} {}

  ~I2CManager() {
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (state.referenceCount > 0) {
      m_registry.Release(state.sdaChannel, DigitalChannelOwner::kI2C);
      m_registry.Release(state.sclChannel, DigitalChannelOwner::kI2C);
    }
    state.referenceCount = 0;
    state.backend.reset();
  }

  I2CResult Initialize(HAL_I2CPort port) noexcept {
    auto portResult = ValidateI2CPort(port);
    if (portResult != I2CResult::kOk) {
      return portResult;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (state.referenceCount == 0) {
      // The official VMX map places the MXP I2C SDA/SCL pair at CommDIO
      // physical 32/33. Reserve it before opening the SDK resource so a
      // generic logical DIO/PWM allocation gets a deterministic conflict
      // rather than an opaque SDK activation failure.
      VMXCommDIOChannelMap map;
      try {
        map = m_mapProvider ? m_mapProvider() : kDefaultVMXCommDIOChannelMap;
      } catch (...) {
        return I2CResult::kHardwareFailure;
      }
      if (!map.valid || !map.i2cValid || !IsPhysicalChannelValid(map.i2cSDA) ||
          !IsPhysicalChannelValid(map.i2cSCL) || map.i2cSDA == map.i2cSCL) {
        return I2CResult::kHardwareFailure;
      }
      int32_t sda = map.i2cSDA;
      int32_t scl = map.i2cSCL;
      auto sdaReservation = m_registry.Reserve(
          sda, DigitalChannelOwner::kI2C, "VMX I2C SDA");
      if (!sdaReservation.reserved) {
        return I2CResult::kResourceConflict;
      }
      auto sclReservation = m_registry.Reserve(
          scl, DigitalChannelOwner::kI2C, "VMX I2C SCL");
      if (!sclReservation.reserved) {
        m_registry.Release(sda, DigitalChannelOwner::kI2C);
        return I2CResult::kResourceConflict;
      }
      try {
        state.backend = m_factory ? m_factory() : nullptr;
      } catch (...) {
        state.backend.reset();
      }
      if (!state.backend) {
        m_registry.Release(sda, DigitalChannelOwner::kI2C);
        m_registry.Release(scl, DigitalChannelOwner::kI2C);
        return I2CResult::kHardwareFailure;
      }
      int32_t actualSda = sda;
      int32_t actualScl = scl;
      if (state.backend->GetPhysicalChannels(actualSda, actualScl) &&
          (actualSda != sda || actualScl != scl)) {
        m_registry.Release(sda, DigitalChannelOwner::kI2C);
        m_registry.Release(scl, DigitalChannelOwner::kI2C);
        auto actualSdaReservation = m_registry.Reserve(
            actualSda, DigitalChannelOwner::kI2C, "VMX I2C SDA");
        auto actualSclReservation = m_registry.Reserve(
            actualScl, DigitalChannelOwner::kI2C, "VMX I2C SCL");
        if (!actualSdaReservation.reserved ||
            !actualSclReservation.reserved) {
          if (actualSdaReservation.reserved) {
            m_registry.Release(actualSda, DigitalChannelOwner::kI2C);
          }
          if (actualSclReservation.reserved) {
            m_registry.Release(actualScl, DigitalChannelOwner::kI2C);
          }
          state.backend.reset();
          return I2CResult::kResourceConflict;
        }
        sda = actualSda;
        scl = actualScl;
      }
      state.sdaChannel = sda;
      state.sclChannel = scl;
    }
    ++state.referenceCount;
    return I2CResult::kOk;
  }

  I2CResult Close(HAL_I2CPort port) noexcept {
    auto portResult = ValidateI2CPort(port);
    if (portResult != I2CResult::kOk) {
      return portResult;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (state.referenceCount <= 0) {
      return I2CResult::kOk;
    }
    --state.referenceCount;
    if (state.referenceCount == 0) {
      m_registry.Release(state.sdaChannel, DigitalChannelOwner::kI2C);
      m_registry.Release(state.sclChannel, DigitalChannelOwner::kI2C);
      state.backend.reset();
      state.sdaChannel = -1;
      state.sclChannel = -1;
    }
    return I2CResult::kOk;
  }

  I2CResult Transaction(HAL_I2CPort port, int32_t deviceAddress,
                        const uint8_t* dataToSend, int32_t sendSize,
                        uint8_t* dataReceived, int32_t receiveSize) noexcept {
    auto validation = ValidateTransfer(port, deviceAddress, dataToSend,
                                       sendSize, dataReceived, receiveSize);
    if (validation != I2CResult::kOk) {
      return validation;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (state.referenceCount <= 0 || !state.backend) {
      return I2CResult::kNotInitialized;
    }
    try {
      return state.backend->Transaction(
                 static_cast<uint8_t>(deviceAddress), dataToSend,
                 static_cast<uint16_t>(sendSize), dataReceived,
                 static_cast<uint16_t>(receiveSize))
                 ? I2CResult::kOk
                 : I2CResult::kHardwareFailure;
    } catch (...) {
      return I2CResult::kHardwareFailure;
    }
  }

  I2CResult Write(HAL_I2CPort port, int32_t deviceAddress,
                  const uint8_t* dataToSend, int32_t sendSize) noexcept {
    auto validation = ValidateTransfer(port, deviceAddress, dataToSend,
                                       sendSize, nullptr, 0);
    if (validation != I2CResult::kOk) {
      return validation;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (state.referenceCount <= 0 || !state.backend) {
      return I2CResult::kNotInitialized;
    }
    try {
      return state.backend->Write(static_cast<uint8_t>(deviceAddress),
                                  dataToSend, sendSize)
                 ? I2CResult::kOk
                 : I2CResult::kHardwareFailure;
    } catch (...) {
      return I2CResult::kHardwareFailure;
    }
  }

  I2CResult Read(HAL_I2CPort port, int32_t deviceAddress,
                 uint8_t* dataReceived, int32_t receiveSize) noexcept {
    auto validation = ValidateTransfer(port, deviceAddress, nullptr, 0,
                                       dataReceived, receiveSize);
    if (validation != I2CResult::kOk) {
      return validation;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    if (state.referenceCount <= 0 || !state.backend) {
      return I2CResult::kNotInitialized;
    }
    try {
      return state.backend->Read(static_cast<uint8_t>(deviceAddress),
                                 dataReceived, receiveSize)
                 ? I2CResult::kOk
                 : I2CResult::kHardwareFailure;
    } catch (...) {
      return I2CResult::kHardwareFailure;
    }
  }

  int32_t GetReferenceCount(HAL_I2CPort port) const noexcept {
    auto portResult = ValidateI2CPort(port);
    if (portResult != I2CResult::kOk) {
      return 0;
    }
    auto& state = m_port;
    std::scoped_lock lock{state.mutex};
    return state.referenceCount;
  }

 private:
  static I2CResult ValidateTransfer(HAL_I2CPort port, int32_t deviceAddress,
                                    const uint8_t* dataToSend,
                                    int32_t sendSize, uint8_t* dataReceived,
                                    int32_t receiveSize) noexcept {
    auto portResult = ValidateI2CPort(port);
    if (portResult != I2CResult::kOk) {
      return portResult;
    }
    if (deviceAddress < 0 || deviceAddress > 0x7f) {
      return I2CResult::kInvalidAddress;
    }
    if (sendSize < 0 || receiveSize < 0 ||
        sendSize > UINT16_MAX || receiveSize > UINT16_MAX) {
      return I2CResult::kInvalidSize;
    }
    if ((sendSize > 0 && dataToSend == nullptr) ||
        (receiveSize > 0 && dataReceived == nullptr)) {
      return I2CResult::kNullPointer;
    }
    return I2CResult::kOk;
  }

  I2CBackendFactory m_factory;
  DigitalChannelRegistry& m_registry;
  CommDIOMapProvider m_mapProvider;
  // One physical VMX I2C bus shared by kOnboard and kMXP aliases.
  mutable I2CPortState m_port;
};

}  // namespace hal::vmx
