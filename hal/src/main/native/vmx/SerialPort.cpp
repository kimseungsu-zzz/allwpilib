// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/SerialPort.h"

#include <memory>
#include <string_view>

#include "HALInitializer.h"
#include "HALInternal.h"
#include "VMXPi.h"
#include "VMXRuntime.h"
#include "SerialInternal.h"
#include "hal/Errors.h"

namespace hal::vmx {
namespace {

class DriverSerialBackend final : public SerialBackend {
 public:
  DriverSerialBackend(std::shared_ptr<VMXPi> context,
                      const VMXCommDIOChannelMap& channels,
                      uint32_t baudRate)
      : m_context{std::move(context)}, m_channels{channels} {
    m_initialized = Activate(baudRate);
  }

  ~DriverSerialBackend() override {
    if (m_initialized && m_context) {
      VMXErrorCode error;
      m_context->io.DeallocateResource(m_resourceHandle, &error);
    }
  }

  bool Reconfigure(uint32_t baudRate) noexcept override {
    if (!m_initialized || !m_context) {
      return false;
    }
    const auto previous = m_baudRate;
    VMXErrorCode error;
    m_context->io.DeallocateResource(m_resourceHandle, &error);
    m_initialized = false;
    if (Activate(baudRate)) {
      return true;
    }
    // Keep the old active resource when a baud-rate update fails.
    m_initialized = Activate(previous);
    return false;
  }

  bool Write(const uint8_t* data, uint16_t size) noexcept override {
    if (!m_initialized || size == 0) {
      return m_initialized;
    }
    try {
      VMXErrorCode error;
      return m_context->io.UART_Write(m_resourceHandle,
                                      const_cast<uint8_t*>(data), size,
                                      &error);
    } catch (...) {
      return false;
    }
  }

  bool Read(uint8_t* data, uint16_t maxSize,
            uint16_t& actualSize) noexcept override {
    actualSize = 0;
    if (!m_initialized || maxSize == 0) {
      return m_initialized;
    }
    try {
      VMXErrorCode error;
      return m_context->io.UART_Read(m_resourceHandle, data, maxSize,
                                     actualSize, &error);
    } catch (...) {
      return false;
    }
  }

  bool GetBytesAvailable(uint16_t& size) noexcept override {
    size = 0;
    if (!m_initialized) {
      return false;
    }
    try {
      VMXErrorCode error;
      return m_context->io.UART_GetBytesAvailable(m_resourceHandle, size,
                                                  &error);
    } catch (...) {
      return false;
    }
  }

  bool IsInitialized() const noexcept { return m_initialized; }

 private:
  bool Activate(uint32_t baudRate) noexcept {
    if (!m_context || !m_context->IsOpen()) {
      return false;
    }
    try {
      ::VMXChannelInfo tx{
          static_cast<VMXChannelIndex>(m_channels.uartTX),
          VMXChannelCapability::UART_TX};
      ::VMXChannelInfo rx{
          static_cast<VMXChannelIndex>(m_channels.uartRX),
          VMXChannelCapability::UART_RX};
      if (!tx.IsValid() || !rx.IsValid()) {
        return false;
      }
      ::UARTConfig config{baudRate};
      VMXErrorCode error;
      if (!m_context->io.ActivateDualchannelResource(
              tx, rx, &config, m_resourceHandle, &error)) {
        return false;
      }
      m_baudRate = baudRate;
      return true;
    } catch (...) {
      return false;
    }
  }

  std::shared_ptr<VMXPi> m_context;
  VMXCommDIOChannelMap m_channels;
  VMXResourceHandle m_resourceHandle = 0;
  uint32_t m_baudRate = 57600;
  bool m_initialized = false;
};

std::unique_ptr<SerialBackend> CreateSerialBackend(
    const VMXCommDIOChannelMap& channels, uint32_t baudRate) {
  auto context = GetRuntimeContext();
  if (!context) {
    return nullptr;
  }
  auto backend =
      std::make_unique<DriverSerialBackend>(std::move(context), channels,
                                            baudRate);
  return backend->IsInitialized() ? std::move(backend) : nullptr;
}

SerialManager& GetSerialManager() {
  static SerialManager manager{CreateSerialBackend, GetDigitalChannelRegistry(),
                               GetVMXCommDIOChannelMap};
  return manager;
}

void SetSerialResult(SerialResult result, int32_t* status,
                     std::string_view message, int32_t port = -1) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  switch (result) {
    case SerialResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case SerialResult::kPortOutOfRange:
      *status = RESOURCE_OUT_OF_RANGE;
      hal::SetLastErrorIndexOutOfRange(status, message, 0, 4, port);
      return;
    case SerialResult::kUnsupportedPort:
    case SerialResult::kUnsupportedConfig:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
    case SerialResult::kInvalidHandle:
      *status = HAL_HANDLE_ERROR;
      hal::SetLastError(status, message);
      return;
    case SerialResult::kInvalidParameter:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status, message);
      return;
    case SerialResult::kResourceConflict:
      *status = RESOURCE_IS_ALLOCATED;
      hal::SetLastError(status, message);
      return;
    case SerialResult::kNotInitialized:
    case SerialResult::kHardwareFailure:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
  }
}

void SetOperationResult(SerialResult result, int32_t* status,
                        std::string_view message) {
  SetSerialResult(result, status, message);
}

}  // namespace
}  // namespace hal::vmx

namespace hal::init {
void InitializeSerialPort() {}
}  // namespace hal::init

extern "C" {

HAL_SerialPortHandle HAL_InitializeSerialPort(HAL_SerialPort port,
                                              int32_t* status) {
  hal::init::CheckInit();
  HAL_SerialPortHandle handle = HAL_kInvalidHandle;
  auto result = hal::vmx::GetSerialManager().Initialize(port, handle);
  hal::vmx::SetSerialResult(result, status,
                            "VMX supports only the CommDIO kMXP UART",
                            static_cast<int32_t>(port));
  return result == hal::vmx::SerialResult::kOk ? handle : HAL_kInvalidHandle;
}

HAL_SerialPortHandle HAL_InitializeSerialPortDirect(HAL_SerialPort port,
                                                    const char* portName,
                                                    int32_t* status) {
  hal::init::CheckInit();
  HAL_SerialPortHandle handle = HAL_kInvalidHandle;
  auto portValidation = hal::vmx::ValidateSerialPort(port);
  if (portValidation != hal::vmx::SerialResult::kOk) {
    hal::vmx::SetSerialResult(portValidation, status,
                              "VMX exposes only the CommDIO kMXP UART",
                              static_cast<int32_t>(port));
    return HAL_kInvalidHandle;
  }
  auto result = hal::vmx::GetSerialManager().InitializeDirect(port, portName,
                                                               handle);
  hal::vmx::SetSerialResult(result, status,
                            "VMX kMXP UART has no direct OS device name",
                            static_cast<int32_t>(port));
  return result == hal::vmx::SerialResult::kOk ? handle : HAL_kInvalidHandle;
}

int HAL_GetSerialFD(HAL_SerialPortHandle handle, int32_t* status) {
  // The SDK UART is not a Linux file descriptor.  Do not manufacture one.
  auto result = hal::vmx::GetSerialManager().GetRawFileDescriptor(handle);
  hal::vmx::SetSerialResult(result, status,
                            "VMX SDK UART does not expose a raw file descriptor");
  return -1;
}

void HAL_SetSerialBaudRate(HAL_SerialPortHandle handle, int32_t baud,
                           int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().SetBaudRate(handle, baud), status,
      "VMX UART baud rate must be in the SDK-supported range 0..230400");
}

void HAL_SetSerialDataBits(HAL_SerialPortHandle handle, int32_t bits,
                           int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().SetDataBits(handle, bits), status,
      "VMX UART supports only 8 data bits");
}

void HAL_SetSerialParity(HAL_SerialPortHandle handle, int32_t parity,
                         int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().SetParity(handle, parity), status,
      "VMX UART supports only no parity");
}

void HAL_SetSerialStopBits(HAL_SerialPortHandle handle, int32_t stopBits,
                           int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().SetStopBits(handle, stopBits), status,
      "VMX UART supports only one stop bit");
}

void HAL_SetSerialWriteMode(HAL_SerialPortHandle handle, int32_t mode,
                            int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().SetWriteMode(handle, mode), status,
      "VMX UART uses a blocking SDK writer");
}

void HAL_SetSerialFlowControl(HAL_SerialPortHandle handle, int32_t flow,
                              int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().SetFlowControl(handle, flow), status,
      "VMX UART supports only no flow control");
}

void HAL_SetSerialTimeout(HAL_SerialPortHandle handle, double timeout,
                          int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().SetTimeout(handle, timeout), status,
      "Invalid VMX UART timeout");
}

void HAL_EnableSerialTermination(HAL_SerialPortHandle handle, char terminator,
                                 int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().EnableTermination(handle, terminator),
      status, "Unable to enable VMX UART termination");
}

void HAL_DisableSerialTermination(HAL_SerialPortHandle handle, int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().DisableTermination(handle), status,
      "Unable to disable VMX UART termination");
}

void HAL_SetSerialReadBufferSize(HAL_SerialPortHandle handle, int32_t size,
                                 int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().SetReadBufferSize(handle, size), status,
      "VMX UART receive buffer size update failed");
}

void HAL_SetSerialWriteBufferSize(HAL_SerialPortHandle handle, int32_t size,
                                  int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().SetWriteBufferSize(handle, size), status,
      "VMX UART transmit buffer size update failed");
}

int32_t HAL_GetSerialBytesReceived(HAL_SerialPortHandle handle,
                                   int32_t* status) {
  int32_t size = 0;
  auto result = hal::vmx::GetSerialManager().GetBytesAvailable(handle, size);
  hal::vmx::SetOperationResult(result, status,
                               "Unable to query VMX UART receive bytes");
  return result == hal::vmx::SerialResult::kOk ? size : 0;
}

int32_t HAL_ReadSerial(HAL_SerialPortHandle handle, char* buffer, int32_t count,
                       int32_t* status) {
  int32_t actual = 0;
  auto result = hal::vmx::GetSerialManager().Read(
      handle, reinterpret_cast<uint8_t*>(buffer), count, actual);
  hal::vmx::SetOperationResult(result, status, "VMX UART read failed");
  return result == hal::vmx::SerialResult::kOk ? actual : 0;
}

int32_t HAL_WriteSerial(HAL_SerialPortHandle handle, const char* buffer,
                        int32_t count, int32_t* status) {
  int32_t actual = 0;
  auto result = hal::vmx::GetSerialManager().Write(
      handle, reinterpret_cast<const uint8_t*>(buffer), count, actual);
  hal::vmx::SetOperationResult(result, status, "VMX UART write failed");
  return result == hal::vmx::SerialResult::kOk ? actual : 0;
}

void HAL_FlushSerial(HAL_SerialPortHandle handle, int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().Flush(handle), status,
      "VMX UART flush failed");
}

void HAL_ClearSerial(HAL_SerialPortHandle handle, int32_t* status) {
  hal::vmx::SetOperationResult(
      hal::vmx::GetSerialManager().Clear(handle), status,
      "VMX UART receive-buffer clear failed");
}

void HAL_CloseSerial(HAL_SerialPortHandle handle) {
  // Close is intentionally idempotent for stale/double handles.
  hal::vmx::GetSerialManager().Close(handle);
}

}  // extern "C"
