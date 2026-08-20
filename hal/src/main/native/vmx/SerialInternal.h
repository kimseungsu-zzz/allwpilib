// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_set>

#include "DigitalChannelRegistry.h"
#include "VMXChannelCapabilities.h"
#include "hal/SerialPort.h"

namespace hal::vmx {

enum class SerialResult {
  kOk,
  kPortOutOfRange,
  kUnsupportedPort,
  kInvalidHandle,
  kInvalidParameter,
  kUnsupportedConfig,
  kNotInitialized,
  kResourceConflict,
  kHardwareFailure,
};

constexpr int32_t kVMXUARTMaxBaudRate = 230400;

class SerialBackend {
 public:
  virtual ~SerialBackend() = default;

  virtual bool Reconfigure(uint32_t baudRate) noexcept = 0;
  virtual bool Write(const uint8_t* data, uint16_t size) noexcept = 0;
  virtual bool Read(uint8_t* data, uint16_t maxSize,
                    uint16_t& actualSize) noexcept = 0;
  virtual bool GetBytesAvailable(uint16_t& size) noexcept = 0;
};

using SerialBackendFactory = std::function<std::unique_ptr<SerialBackend>(
    const VMXCommDIOChannelMap&, uint32_t baudRate)>;
using SerialCommDIOMapProvider = std::function<VMXCommDIOChannelMap()>;

struct SerialPortState {
  mutable std::mutex mutex;
  VMXCommDIOChannelMap channels;
  std::unique_ptr<SerialBackend> backend;
  std::unordered_set<HAL_SerialPortHandle> handles;
  HAL_SerialPortHandle nextHandle = 1;
  uint32_t baudRate = 57600;
  double timeoutMs = 0.0;
  bool terminationEnabled = false;
  char terminationCharacter = '\n';
  uint16_t readBufferSize = 0;
  uint16_t writeBufferSize = 0;
};

constexpr SerialResult ValidateSerialPort(HAL_SerialPort port) noexcept {
  if (port < HAL_SerialPort_Onboard || port > HAL_SerialPort_USB2) {
    return SerialResult::kPortOutOfRange;
  }
  // VMX exposes only the physical CommDIO TTL UART as WPILib kMXP.  Onboard
  // RS-232 and USB ports are separate Linux/board resources and are not
  // aliases for this SDK UART.
  return port == HAL_SerialPort_MXP ? SerialResult::kOk
                                    : SerialResult::kUnsupportedPort;
}

class SerialManager final {
 public:
  explicit SerialManager(
      SerialBackendFactory factory = {},
      DigitalChannelRegistry& registry = GetDigitalChannelRegistry(),
      SerialCommDIOMapProvider mapProvider =
          [] { return kDefaultVMXCommDIOChannelMap; })
      : m_factory{std::move(factory)},
        m_registry{registry},
        m_mapProvider{std::move(mapProvider)} {}

  ~SerialManager() {
    auto& state = m_state;
    std::scoped_lock lock{state.mutex};
    ReleaseChannels(state.channels);
    state.handles.clear();
    state.backend.reset();
  }

  SerialResult Initialize(HAL_SerialPort port,
                          HAL_SerialPortHandle& handle) noexcept {
    handle = HAL_kInvalidHandle;
    auto validation = ValidateSerialPort(port);
    if (validation != SerialResult::kOk) {
      return validation;
    }
    auto& state = m_state;
    std::scoped_lock lock{state.mutex};
    if (!state.backend) {
      VMXCommDIOChannelMap channels;
      try {
        channels = m_mapProvider ? m_mapProvider()
                                 : kDefaultVMXCommDIOChannelMap;
      } catch (...) {
        return SerialResult::kHardwareFailure;
      }
      if (!channels.valid || !channels.uartValid ||
          !IsPhysicalChannelValid(channels.uartTX) ||
          !IsPhysicalChannelValid(channels.uartRX) ||
          channels.uartTX == channels.uartRX) {
        return SerialResult::kUnsupportedConfig;
      }
      if (!ReserveChannels(channels)) {
        return SerialResult::kResourceConflict;
      }
      try {
        state.backend = m_factory ? m_factory(channels, state.baudRate)
                                   : nullptr;
      } catch (...) {
        state.backend.reset();
      }
      if (!state.backend) {
        ReleaseChannels(channels);
        return SerialResult::kHardwareFailure;
      }
      state.channels = channels;
    }
    // A unique logical handle lets two SerialPort objects share the one
    // physical UART without making double-close ambiguous.
    handle = state.nextHandle++;
    if (handle == HAL_kInvalidHandle) {
      handle = state.nextHandle++;
    }
    state.handles.insert(handle);
    return SerialResult::kOk;
  }

  SerialResult InitializeDirect(HAL_SerialPort port, const char* portName,
                                HAL_SerialPortHandle& handle) noexcept {
    if (port != HAL_SerialPort_MXP) {
      handle = HAL_kInvalidHandle;
      return ValidateSerialPort(port);
    }
    // Direct names identify Linux device nodes.  The VMX TTL UART is an SDK
    // resource and has no meaningful OS path, so accepting a name would be a
    // false success.  A null/empty name is treated as the normal kMXP path.
    if (portName && portName[0] != '\0') {
      handle = HAL_kInvalidHandle;
      return SerialResult::kUnsupportedConfig;
    }
    return Initialize(port, handle);
  }

  SerialResult Close(HAL_SerialPortHandle handle) noexcept {
    auto& state = m_state;
    std::scoped_lock lock{state.mutex};
    auto it = state.handles.find(handle);
    if (it == state.handles.end()) {
      return SerialResult::kOk;
    }
    state.handles.erase(it);
    if (state.handles.empty()) {
      state.backend.reset();
      ReleaseChannels(state.channels);
      state.channels = {};
      state.baudRate = 57600;
      state.timeoutMs = 0.0;
      state.terminationEnabled = false;
      state.readBufferSize = 0;
      state.writeBufferSize = 0;
    }
    return SerialResult::kOk;
  }

  SerialResult SetBaudRate(HAL_SerialPortHandle handle,
                           int32_t baudRate) noexcept {
    if (baudRate < 0 || baudRate > kVMXUARTMaxBaudRate) {
      return SerialResult::kInvalidParameter;
    }
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    auto& state = *lockAndState.state;
    if (!state.backend->Reconfigure(static_cast<uint32_t>(baudRate))) {
      return SerialResult::kHardwareFailure;
    }
    state.baudRate = static_cast<uint32_t>(baudRate);
    return SerialResult::kOk;
  }

  SerialResult SetDataBits(HAL_SerialPortHandle handle, int32_t bits) noexcept {
    if (bits < 5 || bits > 8) {
      return SerialResult::kInvalidParameter;
    }
    return SetFixedConfig(handle, bits == 8);
  }

  SerialResult SetParity(HAL_SerialPortHandle handle, int32_t parity) noexcept {
    if (parity < 0 || parity > 4) {
      return SerialResult::kInvalidParameter;
    }
    return SetFixedConfig(handle, parity == 0);
  }

  SerialResult SetStopBits(HAL_SerialPortHandle handle,
                           int32_t stopBits) noexcept {
    if (stopBits != 10 && stopBits != 15 && stopBits != 20) {
      return SerialResult::kInvalidParameter;
    }
    return SetFixedConfig(handle, stopBits == 10);
  }

  SerialResult SetWriteMode(HAL_SerialPortHandle handle, int32_t mode) noexcept {
    if (mode != 1 && mode != 2) {
      return SerialResult::kInvalidParameter;
    }
    // The SDK writer is blocking, so both WPILib modes are implemented as an
    // immediate write. There is no deferred SDK queue to flush.
    return SetFixedConfig(handle, true);
  }

  SerialResult SetFlowControl(HAL_SerialPortHandle handle,
                              int32_t flow) noexcept {
    if (flow < 0 || flow > 3) {
      return SerialResult::kInvalidParameter;
    }
    return SetFixedConfig(handle, flow == 0);
  }

  SerialResult SetTimeout(HAL_SerialPortHandle handle, double timeoutMs) noexcept {
    if (!std::isfinite(timeoutMs) || timeoutMs < 0.0) {
      return SerialResult::kInvalidParameter;
    }
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    lockAndState.state->timeoutMs = timeoutMs;
    return SerialResult::kOk;
  }

  SerialResult EnableTermination(HAL_SerialPortHandle handle,
                                 char character) noexcept {
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    lockAndState.state->terminationEnabled = true;
    lockAndState.state->terminationCharacter = character;
    return SerialResult::kOk;
  }

  SerialResult DisableTermination(HAL_SerialPortHandle handle) noexcept {
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    lockAndState.state->terminationEnabled = false;
    return SerialResult::kOk;
  }

  SerialResult SetReadBufferSize(HAL_SerialPortHandle handle,
                                 int32_t size) noexcept {
    if (size < 0 || size > UINT16_MAX) {
      return SerialResult::kInvalidParameter;
    }
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    // The SDK queue itself is fixed, but the adapter uses this limit when
    // chunking SDK reads so the WPILib software control has observable
    // semantics without inventing an OS file descriptor.
    lockAndState.state->readBufferSize = static_cast<uint16_t>(size);
    return SerialResult::kOk;
  }

  SerialResult SetWriteBufferSize(HAL_SerialPortHandle handle,
                                  int32_t size) noexcept {
    if (size < 0 || size > UINT16_MAX) {
      return SerialResult::kInvalidParameter;
    }
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    // The SDK queue itself is fixed, but the adapter uses this limit when
    // chunking SDK writes so the WPILib software control has observable
    // semantics without inventing an OS file descriptor.
    lockAndState.state->writeBufferSize = static_cast<uint16_t>(size);
    return SerialResult::kOk;
  }

  SerialResult GetRawFileDescriptor(HAL_SerialPortHandle handle) noexcept {
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    return SerialResult::kUnsupportedConfig;
  }

  SerialResult GetBytesAvailable(HAL_SerialPortHandle handle,
                                 int32_t& size) noexcept {
    size = 0;
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    uint16_t available = 0;
    if (!lockAndState.state->backend->GetBytesAvailable(available)) {
      return SerialResult::kHardwareFailure;
    }
    size = available;
    return SerialResult::kOk;
  }

  SerialResult Read(HAL_SerialPortHandle handle, uint8_t* buffer,
                    int32_t count, int32_t& actual) noexcept {
    actual = 0;
    if (count < 0 || count > UINT16_MAX || (count > 0 && !buffer)) {
      return SerialResult::kInvalidParameter;
    }
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    auto& state = *lockAndState.state;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double, std::milli>(
                              state.timeoutMs);
    while (actual < count) {
      uint16_t available = 0;
      if (!state.backend->GetBytesAvailable(available)) {
        return SerialResult::kHardwareFailure;
      }
      if (available > 0) {
        uint16_t read = 0;
        const auto configuredLimit = state.readBufferSize == 0
                                          ? static_cast<int32_t>(UINT16_MAX)
                                          : state.readBufferSize;
        const auto request = state.terminationEnabled
                                 ? uint16_t{1}
                                 : static_cast<uint16_t>(std::min<int32_t>(
                                       {available, count - actual,
                                        configuredLimit}));
        if (!state.backend->Read(buffer + actual, request, read)) {
          return SerialResult::kHardwareFailure;
        }
        if (read == 0) {
          return SerialResult::kHardwareFailure;
        }
        for (uint16_t i = 0; i < read; ++i) {
          ++actual;
          if (state.terminationEnabled &&
              buffer[actual - 1] ==
                  static_cast<uint8_t>(state.terminationCharacter)) {
            return SerialResult::kOk;
          }
          if (actual == count) {
            break;
          }
        }
        continue;
      }
      if (state.timeoutMs <= 0.0 ||
          std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    return SerialResult::kOk;
  }

  SerialResult Write(HAL_SerialPortHandle handle, const uint8_t* buffer,
                     int32_t count, int32_t& actual) noexcept {
    actual = 0;
    if (count < 0 || count > UINT16_MAX || (count > 0 && !buffer)) {
      return SerialResult::kInvalidParameter;
    }
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    if (count == 0) {
      return SerialResult::kOk;
    }
    const auto configuredLimit = lockAndState.state->writeBufferSize == 0
                                     ? static_cast<int32_t>(UINT16_MAX)
                                     : lockAndState.state->writeBufferSize;
    while (actual < count) {
      const auto request = static_cast<uint16_t>(std::min<int32_t>(
          {count - actual, configuredLimit, static_cast<int32_t>(UINT16_MAX)}));
      if (!lockAndState.state->backend->Write(buffer + actual, request)) {
        return SerialResult::kHardwareFailure;
      }
      actual += request;
    }
    return SerialResult::kOk;
  }

  SerialResult UnsupportedOperation(HAL_SerialPortHandle handle) noexcept {
    auto lockAndState = GetState(handle);
    return lockAndState.state ? SerialResult::kUnsupportedConfig
                              : SerialResult::kInvalidHandle;
  }

  SerialResult Flush(HAL_SerialPortHandle handle) noexcept {
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    // UART_Write is blocking and returns only after handing the bytes to the
    // SDK, so there is no deferred adapter queue left to flush.
    return SerialResult::kOk;
  }

  SerialResult Clear(HAL_SerialPortHandle handle) noexcept {
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    auto& backend = lockAndState.state->backend;
    std::array<uint8_t, UINT16_MAX> scratch{};
    while (true) {
      uint16_t available = 0;
      if (!backend->GetBytesAvailable(available)) {
        return SerialResult::kHardwareFailure;
      }
      if (available == 0) {
        return SerialResult::kOk;
      }
      uint16_t actual = 0;
      if (!backend->Read(scratch.data(),
                        std::min<uint16_t>(available, scratch.size()), actual)) {
        return SerialResult::kHardwareFailure;
      }
      if (actual == 0) {
        return SerialResult::kHardwareFailure;
      }
    }
  }

 private:
  struct LockedState {
    std::unique_lock<std::mutex> lock;
    SerialPortState* state = nullptr;
  };

  LockedState GetState(HAL_SerialPortHandle handle) noexcept {
    LockedState result{std::unique_lock<std::mutex>{m_state.mutex}, &m_state};
    if (m_state.handles.find(handle) == m_state.handles.end() ||
        !m_state.backend) {
      result.state = nullptr;
    }
    return result;
  }

  SerialResult SetFixedConfig(HAL_SerialPortHandle handle, bool supported) {
    auto lockAndState = GetState(handle);
    if (!lockAndState.state) {
      return SerialResult::kInvalidHandle;
    }
    return supported ? SerialResult::kOk : SerialResult::kUnsupportedConfig;
  }

  bool ReserveChannels(const VMXCommDIOChannelMap& map) {
    auto tx = m_registry.Reserve(map.uartTX, DigitalChannelOwner::kUART,
                                 "VMX UART TX");
    if (!tx.reserved) {
      return false;
    }
    auto rx = m_registry.Reserve(map.uartRX, DigitalChannelOwner::kUART,
                                 "VMX UART RX");
    if (!rx.reserved) {
      m_registry.Release(map.uartTX, DigitalChannelOwner::kUART);
      return false;
    }
    return true;
  }

  void ReleaseChannels(const VMXCommDIOChannelMap& map) noexcept {
    m_registry.Release(map.uartTX, DigitalChannelOwner::kUART);
    m_registry.Release(map.uartRX, DigitalChannelOwner::kUART);
  }

  SerialBackendFactory m_factory;
  DigitalChannelRegistry& m_registry;
  SerialCommDIOMapProvider m_mapProvider;
  SerialPortState m_state;
};

}  // namespace hal::vmx
