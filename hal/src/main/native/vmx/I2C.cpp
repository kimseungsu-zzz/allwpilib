// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/I2C.h"

#include <memory>
#include <string_view>

#include "VMXPi.h"

#include "HALInitializer.h"
#include "HALInternal.h"
#include "I2CInternal.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"

namespace hal::vmx {
namespace {

class DriverI2CBackend final : public I2CBackend {
 public:
  explicit DriverI2CBackend(std::shared_ptr<VMXPi> context)
      : m_context{std::move(context)} {
    if (!m_context || !m_context->IsOpen()) {
      return;
    }

    auto sda = m_context->io.GetSoleChannelIndex(
        VMXChannelCapability::I2C_SDA);
    auto scl = m_context->io.GetSoleChannelIndex(
        VMXChannelCapability::I2C_SCL);
    ::VMXChannelInfo sdaChannel{sda, VMXChannelCapability::I2C_SDA};
    ::VMXChannelInfo sclChannel{scl, VMXChannelCapability::I2C_SCL};
    if (!sdaChannel.IsValid() || !sclChannel.IsValid()) {
      return;
    }

    I2CConfig config;
    VMXErrorCode error;
    m_initialized = m_context->io.ActivateDualchannelResource(
        sdaChannel, sclChannel, &config, m_resourceHandle, &error);
    if (m_initialized) {
      m_sda = sda;
      m_scl = scl;
    }
  }

  ~DriverI2CBackend() override {
    if (m_initialized) {
      VMXErrorCode error;
      m_context->io.DeallocateResource(m_resourceHandle, &error);
    }
  }

  bool IsInitialized() const noexcept { return m_initialized; }

  bool Transaction(uint8_t deviceAddress, const uint8_t* dataToSend,
                   uint16_t sendSize, uint8_t* dataReceived,
                   uint16_t receiveSize) noexcept override {
    try {
      if (!m_initialized) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.I2C_Transaction(
          m_resourceHandle, deviceAddress, const_cast<uint8_t*>(dataToSend),
          sendSize, dataReceived, receiveSize, &error);
    } catch (...) {
      return false;
    }
  }

  bool Write(uint8_t deviceAddress, const uint8_t* dataToSend,
             int32_t sendSize) noexcept override {
    try {
      if (!m_initialized) {
        return false;
      }
      // WPILib HAL_Write receives the complete wire buffer.  VMX's
      // convenience I2C_Write takes the first byte as a register address and
      // the remaining bytes as payload, so retain that exact WPILib layout.
      if (sendSize <= 1) {
        return Transaction(deviceAddress, dataToSend,
                           static_cast<uint16_t>(sendSize), nullptr, 0);
      }
      VMXErrorCode error;
      return m_context->io.I2C_Write(
          m_resourceHandle, deviceAddress, dataToSend[0],
          const_cast<uint8_t*>(dataToSend + 1), sendSize - 1, &error);
    } catch (...) {
      return false;
    }
  }

  bool Read(uint8_t deviceAddress, uint8_t* dataReceived,
            int32_t receiveSize) noexcept override {
    // HAL_ReadI2C is WPILib's read-only operation: it must not invent a
    // register byte.  The VMX transaction primitive supports this directly.
    return Transaction(deviceAddress, nullptr, 0, dataReceived,
                       static_cast<uint16_t>(receiveSize));
  }

  bool GetPhysicalChannels(int32_t& sda,
                           int32_t& scl) const noexcept override {
    if (!m_initialized) {
      return false;
    }
    sda = m_sda;
    scl = m_scl;
    return true;
  }

 private:
  std::shared_ptr<VMXPi> m_context;
  VMXResourceHandle m_resourceHandle = 0;
  int32_t m_sda = -1;
  int32_t m_scl = -1;
  bool m_initialized = false;
};

std::unique_ptr<I2CBackend> CreateI2CBackend() {
  auto context = GetRuntimeContext();
  if (!context) {
    return nullptr;
  }
  auto backend = std::make_unique<DriverI2CBackend>(std::move(context));
  return backend->IsInitialized() ? std::move(backend) : nullptr;
}

I2CManager& GetI2CManager() {
  static I2CManager manager{CreateI2CBackend, GetDigitalChannelRegistry(),
                            GetVMXCommDIOChannelMap};
  return manager;
}

void SetI2CResult(I2CResult result, int32_t* status,
                  std::string_view message, int32_t port = -1) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  switch (result) {
    case I2CResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case I2CResult::kPortOutOfRange:
      *status = RESOURCE_OUT_OF_RANGE;
      hal::SetLastErrorIndexOutOfRange(status, message, 0, 1, port);
      return;
    case I2CResult::kUnsupportedPort:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
    case I2CResult::kInvalidAddress:
    case I2CResult::kInvalidSize:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status, message);
      return;
    case I2CResult::kNullPointer:
      *status = NULL_PARAMETER;
      hal::SetLastError(status, message);
      return;
    case I2CResult::kNotInitialized:
    case I2CResult::kHardwareFailure:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
    case I2CResult::kNoResources:
      *status = NO_AVAILABLE_RESOURCES;
      hal::SetLastError(status, message);
      return;
    case I2CResult::kResourceConflict:
      *status = RESOURCE_IS_ALLOCATED;
      hal::SetLastError(status, message);
      return;
  }
}

void ReportI2CResult(I2CResult result, std::string_view message,
                     int32_t port = -1) {
  if (result != I2CResult::kOk) {
    SetI2CResult(result, nullptr, message, port);
  }
}

int32_t ReturnI2COperation(I2CResult result, std::string_view message,
                           int32_t port) {
  SetI2CResult(result, nullptr, message, port);
  return result == I2CResult::kOk ? 0 : -1;
}

}  // namespace
}  // namespace hal::vmx

extern "C" {

void HAL_InitializeI2C(HAL_I2CPort port, int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetI2CResult(
      hal::vmx::GetI2CManager().Initialize(port), status,
      "VMX I2C onboard and MXP ports alias physical channels 26/27",
      static_cast<int32_t>(port));
}

int32_t HAL_TransactionI2C(HAL_I2CPort port, int32_t deviceAddress,
                           const uint8_t* dataToSend, int32_t sendSize,
                           uint8_t* dataReceived, int32_t receiveSize) {
  return hal::vmx::ReturnI2COperation(
      hal::vmx::GetI2CManager().Transaction(
          port, deviceAddress, dataToSend, sendSize, dataReceived,
          receiveSize),
      "VMX I2C transaction failed", static_cast<int32_t>(port));
}

int32_t HAL_WriteI2C(HAL_I2CPort port, int32_t deviceAddress,
                     const uint8_t* dataToSend, int32_t sendSize) {
  return hal::vmx::ReturnI2COperation(
      hal::vmx::GetI2CManager().Write(port, deviceAddress, dataToSend,
                                      sendSize),
      "VMX I2C write failed", static_cast<int32_t>(port));
}

int32_t HAL_ReadI2C(HAL_I2CPort port, int32_t deviceAddress,
                    uint8_t* buffer, int32_t count) {
  return hal::vmx::ReturnI2COperation(
      hal::vmx::GetI2CManager().Read(port, deviceAddress, buffer, count),
      "VMX I2C read failed", static_cast<int32_t>(port));
}

void HAL_CloseI2C(HAL_I2CPort port) {
  auto result = hal::vmx::GetI2CManager().Close(port);
  hal::vmx::ReportI2CResult(result, "Invalid VMX I2C port",
                           static_cast<int32_t>(port));
}

}  // extern "C"
