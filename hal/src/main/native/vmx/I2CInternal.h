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
};

using I2CBackendFactory = std::function<std::unique_ptr<I2CBackend>()>;

struct I2CPortState {
  std::mutex mutex;
  int32_t referenceCount = 0;
  std::unique_ptr<I2CBackend> backend;
};

constexpr I2CResult ValidateI2CPort(HAL_I2CPort port) noexcept {
  if (port < HAL_I2C_kOnboard || port > HAL_I2C_kMXP) {
    return I2CResult::kPortOutOfRange;
  }
  return port == HAL_I2C_kMXP ? I2CResult::kOk : I2CResult::kUnsupportedPort;
}

class I2CManager final {
 public:
  explicit I2CManager(I2CBackendFactory factory = {})
      : m_factory{std::move(factory)} {}

  I2CResult Initialize(HAL_I2CPort port) noexcept {
    auto portResult = ValidateI2CPort(port);
    if (portResult != I2CResult::kOk) {
      return portResult;
    }
    auto& state = m_ports[static_cast<size_t>(port)];
    std::scoped_lock lock{state.mutex};
    if (state.referenceCount == 0) {
      try {
        state.backend = m_factory ? m_factory() : nullptr;
      } catch (...) {
        state.backend.reset();
      }
      if (!state.backend) {
        return I2CResult::kHardwareFailure;
      }
    }
    ++state.referenceCount;
    return I2CResult::kOk;
  }

  I2CResult Close(HAL_I2CPort port) noexcept {
    auto portResult = ValidateI2CPort(port);
    if (portResult != I2CResult::kOk) {
      return portResult;
    }
    auto& state = m_ports[static_cast<size_t>(port)];
    std::scoped_lock lock{state.mutex};
    if (state.referenceCount <= 0) {
      return I2CResult::kOk;
    }
    --state.referenceCount;
    if (state.referenceCount == 0) {
      state.backend.reset();
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
    auto& state = m_ports[static_cast<size_t>(port)];
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
    auto& state = m_ports[static_cast<size_t>(port)];
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
    auto& state = m_ports[static_cast<size_t>(port)];
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
    auto& state = m_ports[static_cast<size_t>(port)];
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
  mutable std::array<I2CPortState, 2> m_ports;
};

}  // namespace hal::vmx
