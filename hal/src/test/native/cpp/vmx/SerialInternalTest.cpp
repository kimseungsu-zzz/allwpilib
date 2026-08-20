// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "../../../../main/native/vmx/SerialInternal.h"
#include "../../../../main/native/vmx/SPIInternal.h"

namespace hal::vmx {
namespace {

struct FakeUARTState {
  int factories = 0;
  int reconfigures = 0;
  std::vector<uint8_t> received;
  std::vector<uint8_t> transmitted;
  uint32_t baudRate = 57600;
  bool failRead = false;
  bool failWrite = false;
};

class FakeUARTBackend final : public SerialBackend {
 public:
  explicit FakeUARTBackend(std::shared_ptr<FakeUARTState> state,
                           uint32_t baudRate)
      : m_state{std::move(state)} {
    m_state->baudRate = baudRate;
  }

  bool Reconfigure(uint32_t baudRate) noexcept override {
    ++m_state->reconfigures;
    if (baudRate == 12345) {
      return false;
    }
    m_state->baudRate = baudRate;
    return true;
  }

  bool Write(const uint8_t* data, uint16_t size) noexcept override {
    if (m_state->failWrite) {
      return false;
    }
    m_state->transmitted.insert(m_state->transmitted.end(), data, data + size);
    return true;
  }

  bool Read(uint8_t* data, uint16_t maxSize,
            uint16_t& actualSize) noexcept override {
    actualSize = 0;
    if (m_state->failRead) {
      return false;
    }
    actualSize = static_cast<uint16_t>(
        std::min<size_t>(maxSize, m_state->received.size()));
    std::copy_n(m_state->received.begin(), actualSize, data);
    m_state->received.erase(m_state->received.begin(),
                            m_state->received.begin() + actualSize);
    return true;
  }

  bool GetBytesAvailable(uint16_t& size) noexcept override {
    size = static_cast<uint16_t>(m_state->received.size());
    return true;
  }

 private:
  std::shared_ptr<FakeUARTState> m_state;
};

class NoopSPIBackend final : public SPIBackend {
 public:
  bool Transaction(uint8_t*, uint8_t*, uint16_t) noexcept override {
    return true;
  }
  bool Write(uint8_t*, uint16_t) noexcept override { return true; }
  bool Read(uint8_t*, uint16_t) noexcept override { return true; }
  bool Reconfigure(const SPIPortConfig&) noexcept override { return true; }
  int32_t GetHandle() const noexcept override { return 7; }
};

struct UARTFixture {
  UARTFixture()
      : state{std::make_shared<FakeUARTState>()},
        manager{[this](const VMXCommDIOChannelMap&,
                       uint32_t baudRate) {
          ++state->factories;
          return std::unique_ptr<SerialBackend>{
              std::make_unique<FakeUARTBackend>(state, baudRate)};
        },
                registry} {}

  std::shared_ptr<FakeUARTState> state;
  DigitalChannelRegistry registry;
  SerialManager manager;
};

TEST(VMXSerialInternalTest, OnlyMXPMapsToTheVMXTTLResource) {
  UARTFixture fixture;
  HAL_SerialPortHandle handle = HAL_kInvalidHandle;
  EXPECT_EQ(ValidateSerialPort(HAL_SerialPort_Onboard),
            SerialResult::kUnsupportedPort);
  EXPECT_EQ(ValidateSerialPort(HAL_SerialPort_USB1),
            SerialResult::kUnsupportedPort);
  EXPECT_EQ(ValidateSerialPort(static_cast<HAL_SerialPort>(-1)),
            SerialResult::kPortOutOfRange);
  EXPECT_EQ(fixture.manager.Initialize(HAL_SerialPort_MXP, handle),
            SerialResult::kOk);
  EXPECT_NE(handle, HAL_kInvalidHandle);
  EXPECT_EQ(fixture.state->factories, 1);
  EXPECT_EQ(fixture.manager.Close(handle), SerialResult::kOk);
}

TEST(VMXSerialInternalTest, RepeatedLifecycleSharesPhysicalUARTAndCloseIsSafe) {
  UARTFixture fixture;
  HAL_SerialPortHandle first = HAL_kInvalidHandle;
  HAL_SerialPortHandle second = HAL_kInvalidHandle;
  ASSERT_EQ(fixture.manager.Initialize(HAL_SerialPort_MXP, first),
            SerialResult::kOk);
  ASSERT_EQ(fixture.manager.Initialize(HAL_SerialPort_MXP, second),
            SerialResult::kOk);
  EXPECT_NE(first, second);
  EXPECT_EQ(fixture.state->factories, 1);
  EXPECT_EQ(fixture.manager.Close(first), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.SetBaudRate(second, 115200), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.Close(first), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.Close(second), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.SetBaudRate(second, 9600),
            SerialResult::kInvalidHandle);
}

TEST(VMXSerialInternalTest, BaudRangeAndExactUARTConfigSemantics) {
  UARTFixture fixture;
  HAL_SerialPortHandle handle = HAL_kInvalidHandle;
  ASSERT_EQ(fixture.manager.Initialize(HAL_SerialPort_MXP, handle),
            SerialResult::kOk);
  EXPECT_EQ(fixture.manager.SetBaudRate(handle, 0), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.SetBaudRate(handle, 230400), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.SetBaudRate(handle, 230401),
            SerialResult::kInvalidParameter);
  EXPECT_EQ(fixture.manager.SetBaudRate(handle, -1),
            SerialResult::kInvalidParameter);
  EXPECT_EQ(fixture.manager.SetDataBits(handle, 8), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.SetDataBits(handle, 7),
            SerialResult::kUnsupportedConfig);
  EXPECT_EQ(fixture.manager.SetDataBits(handle, 9),
            SerialResult::kInvalidParameter);
  EXPECT_EQ(fixture.manager.SetParity(handle, 0), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.SetParity(handle, 2),
            SerialResult::kUnsupportedConfig);
  EXPECT_EQ(fixture.manager.SetStopBits(handle, 10), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.SetStopBits(handle, 20),
            SerialResult::kUnsupportedConfig);
  EXPECT_EQ(fixture.manager.SetFlowControl(handle, 0), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.SetFlowControl(handle, 2),
            SerialResult::kUnsupportedConfig);
  EXPECT_EQ(fixture.manager.SetWriteMode(handle, 2), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.SetWriteMode(handle, 1), SerialResult::kOk);
}

TEST(VMXSerialInternalTest, ReadWriteTimeoutAndTerminationUseSDKByteSemantics) {
  UARTFixture fixture;
  HAL_SerialPortHandle handle = HAL_kInvalidHandle;
  ASSERT_EQ(fixture.manager.Initialize(HAL_SerialPort_MXP, handle),
            SerialResult::kOk);
  fixture.state->received = {'a', '\n', 'z'};
  ASSERT_EQ(fixture.manager.EnableTermination(handle, '\n'), SerialResult::kOk);
  uint8_t buffer[8]{};
  int32_t actual = -1;
  EXPECT_EQ(fixture.manager.Read(handle, buffer, 8, actual), SerialResult::kOk);
  EXPECT_EQ(actual, 2);
  EXPECT_EQ(std::string(reinterpret_cast<char*>(buffer), 2), "a\n");
  EXPECT_EQ(fixture.manager.GetBytesAvailable(handle, actual), SerialResult::kOk);
  EXPECT_EQ(actual, 1);

  const uint8_t write[] = {'o', 'k'};
  EXPECT_EQ(fixture.manager.Write(handle, write, 2, actual), SerialResult::kOk);
  EXPECT_EQ(actual, 2);
  EXPECT_EQ(fixture.state->transmitted, std::vector<uint8_t>({'o', 'k'}));
  EXPECT_EQ(fixture.manager.Read(handle, nullptr, 1, actual),
            SerialResult::kInvalidParameter);
  EXPECT_EQ(fixture.manager.Write(handle, nullptr, 1, actual),
            SerialResult::kInvalidParameter);
}

TEST(VMXSerialInternalTest, TimeoutAndUnsupportedOperationsAreExplicit) {
  UARTFixture fixture;
  HAL_SerialPortHandle handle = HAL_kInvalidHandle;
  ASSERT_EQ(fixture.manager.Initialize(HAL_SerialPort_MXP, handle),
            SerialResult::kOk);
  EXPECT_EQ(fixture.manager.SetTimeout(handle, 2.0), SerialResult::kOk);
  uint8_t buffer[1]{};
  int32_t actual = -1;
  EXPECT_EQ(fixture.manager.Read(handle, buffer, 1, actual), SerialResult::kOk);
  EXPECT_EQ(actual, 0);
  EXPECT_EQ(fixture.manager.SetReadBufferSize(handle, 1), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.SetWriteBufferSize(handle, 2), SerialResult::kOk);
  EXPECT_EQ(fixture.manager.GetRawFileDescriptor(handle),
            SerialResult::kUnsupportedConfig);
  EXPECT_EQ(fixture.manager.UnsupportedOperation(handle),
            SerialResult::kUnsupportedConfig);
}

TEST(VMXSerialInternalTest, RegistryConflictsAreAtomic) {
  UARTFixture fixture;
  const auto map = kDefaultVMXCommDIOChannelMap;
  ASSERT_TRUE(fixture.registry
                  .Reserve(map.uartRX, DigitalChannelOwner::kDIO,
                           "test DIO")
                  .reserved);
  HAL_SerialPortHandle handle = HAL_kInvalidHandle;
  EXPECT_EQ(fixture.manager.Initialize(HAL_SerialPort_MXP, handle),
            SerialResult::kResourceConflict);
  EXPECT_EQ(fixture.state->factories, 0);
}

TEST(VMXSerialInternalTest, CorrectedSPIAndUARTPinsCanShareTheRegistry) {
  DigitalChannelRegistry registry;
  SPIManager spi{
      [](HAL_SPIPort, const VMXCommDIOChannelMap&, const SPIPortConfig&) {
        return std::unique_ptr<SPIBackend>{std::make_unique<NoopSPIBackend>()};
      },
      registry};
  auto uartState = std::make_shared<FakeUARTState>();
  SerialManager uart{
      [uartState](const VMXCommDIOChannelMap&, uint32_t baudRate) {
        return std::unique_ptr<SerialBackend>{
            std::make_unique<FakeUARTBackend>(uartState, baudRate)};
      },
      registry};
  ASSERT_EQ(spi.Initialize(HAL_SPI_kOnboardCS0), SPIResult::kOk);
  HAL_SerialPortHandle handle = HAL_kInvalidHandle;
  EXPECT_EQ(uart.Initialize(HAL_SerialPort_MXP, handle), SerialResult::kOk);
  EXPECT_EQ(spi.Close(HAL_SPI_kMXP), SPIResult::kOk);
  EXPECT_EQ(uart.Close(handle), SerialResult::kOk);
}

TEST(VMXSerialInternalTest, DirectNamesCannotPretendToBeSDKUARTs) {
  UARTFixture fixture;
  HAL_SerialPortHandle handle = HAL_kInvalidHandle;
  EXPECT_EQ(fixture.manager.InitializeDirect(HAL_SerialPort_MXP, "/dev/ttyUSB0",
                                             handle),
            SerialResult::kUnsupportedConfig);
  EXPECT_EQ(fixture.manager.InitializeDirect(HAL_SerialPort_MXP, nullptr,
                                             handle),
            SerialResult::kOk);
}

}  // namespace
}  // namespace hal::vmx
