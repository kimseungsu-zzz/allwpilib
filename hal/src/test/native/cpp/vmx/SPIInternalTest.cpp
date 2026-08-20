// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "../../../../main/native/vmx/SPIInternal.h"

namespace hal::vmx {
namespace {

struct FakeSPIState {
  int factories = 0;
  int transactions = 0;
  int writes = 0;
  int reads = 0;
  int reconfigures = 0;
  bool failTransfers = false;
  bool failReconfigure = false;
  bool blockTransfer = false;
  std::atomic_bool transferEntered = false;
  SPIPortConfig config;
};

class FakeSPIBackend final : public SPIBackend {
 public:
  FakeSPIBackend(std::shared_ptr<FakeSPIState> state, SPIPortConfig config)
      : m_state{std::move(state)} {
    m_state->config = config;
  }

  bool Transaction(uint8_t*, uint8_t* receive, uint16_t size) noexcept override {
    ++m_state->transactions;
    m_state->transferEntered = true;
    if (m_state->blockTransfer) {
      std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
    if (receive) {
      for (uint16_t i = 0; i < size; ++i) {
        receive[i] = static_cast<uint8_t>(i + 1);
      }
    }
    return !m_state->failTransfers;
  }

  bool Write(uint8_t*, uint16_t) noexcept override {
    ++m_state->writes;
    return !m_state->failTransfers;
  }

  bool Read(uint8_t* receive, uint16_t size) noexcept override {
    ++m_state->reads;
    if (receive) {
      for (uint16_t i = 0; i < size; ++i) {
        receive[i] = 0xa5;
      }
    }
    return !m_state->failTransfers;
  }

  bool Reconfigure(const SPIPortConfig& config) noexcept override {
    ++m_state->reconfigures;
    if (m_state->failReconfigure) {
      return false;
    }
    m_state->config = config;
    return true;
  }

  int32_t GetHandle() const noexcept override { return 42; }

 private:
  std::shared_ptr<FakeSPIState> m_state;
};

struct SPIFixture {
  SPIFixture()
      : manager{
            [this](HAL_SPIPort, const VMXCommDIOChannelMap&,
                   const SPIPortConfig& config) {
              ++state->factories;
              return std::unique_ptr<SPIBackend>{
                  std::make_unique<FakeSPIBackend>(state, config)};
            },
            registry,
            [this] { return map; }} {}

  DigitalChannelRegistry registry;
  VMXCommDIOChannelMap map;
  std::shared_ptr<FakeSPIState> state = std::make_shared<FakeSPIState>();
  SPIManager manager;
};

TEST(VMXSPIInternalTest, ValidatesPortsAndRepeatedLifecycle) {
  SPIFixture fixture;
  EXPECT_EQ(ValidateSPIPort(static_cast<HAL_SPIPort>(-2)),
            SPIResult::kPortOutOfRange);
  EXPECT_EQ(ValidateSPIPort(HAL_SPI_kInvalid), SPIResult::kPortOutOfRange);
  EXPECT_EQ(fixture.manager.Initialize(HAL_SPI_kOnboardCS0),
            SPIResult::kUnsupportedPort);
  EXPECT_EQ(fixture.manager.Initialize(HAL_SPI_kMXP), SPIResult::kOk);
  EXPECT_EQ(fixture.manager.Initialize(HAL_SPI_kMXP), SPIResult::kOk);
  EXPECT_EQ(fixture.state->factories, 1);
  EXPECT_EQ(fixture.manager.GetHandle(HAL_SPI_kMXP), 42);
  EXPECT_EQ(fixture.manager.Close(HAL_SPI_kMXP), SPIResult::kOk);
  EXPECT_EQ(fixture.manager.Close(HAL_SPI_kMXP), SPIResult::kOk);
  EXPECT_EQ(fixture.manager.GetHandle(HAL_SPI_kMXP), 0);
}

TEST(VMXSPIInternalTest, ValidatesTransferLengthsPointersAndResults) {
  SPIFixture fixture;
  ASSERT_EQ(fixture.manager.Initialize(HAL_SPI_kMXP), SPIResult::kOk);
  int32_t transferred = -1;
  uint8_t send[3]{1, 2, 3};
  uint8_t receive[3]{};
  EXPECT_EQ(fixture.manager.Transaction(HAL_SPI_kMXP, send, receive, 3,
                                        transferred),
            SPIResult::kOk);
  EXPECT_EQ(transferred, 3);
  EXPECT_EQ(receive[2], 3);
  EXPECT_EQ(fixture.manager.Transaction(HAL_SPI_kMXP, nullptr, receive, 1,
                                        transferred),
            SPIResult::kNullPointer);
  EXPECT_EQ(fixture.manager.Write(HAL_SPI_kMXP, nullptr, 1, transferred),
            SPIResult::kNullPointer);
  EXPECT_EQ(fixture.manager.Read(HAL_SPI_kMXP, nullptr, 1, transferred),
            SPIResult::kNullPointer);
  EXPECT_EQ(fixture.manager.Write(HAL_SPI_kMXP, nullptr, 0, transferred),
            SPIResult::kOk);
  EXPECT_EQ(fixture.manager.Read(HAL_SPI_kMXP, nullptr, 0, transferred),
            SPIResult::kOk);
  EXPECT_EQ(fixture.manager.Write(HAL_SPI_kMXP, send, UINT16_MAX + 1,
                                  transferred),
            SPIResult::kInvalidSize);
  fixture.state->failTransfers = true;
  EXPECT_EQ(fixture.manager.Write(HAL_SPI_kMXP, send, 3, transferred),
            SPIResult::kHardwareFailure);
  EXPECT_EQ(transferred, -1);
}

TEST(VMXSPIInternalTest, MapsClockModesAndChipSelectPolarity) {
  SPIFixture fixture;
  ASSERT_EQ(fixture.manager.Initialize(HAL_SPI_kMXP), SPIResult::kOk);
  EXPECT_EQ(fixture.manager.GetMode(HAL_SPI_kMXP), HAL_SPI_kMode0);
  EXPECT_EQ(fixture.manager.SetClockRate(HAL_SPI_kMXP, 499999),
            SPIResult::kInvalidClockRate);
  EXPECT_EQ(fixture.manager.SetClockRate(HAL_SPI_kMXP, 10000000),
            SPIResult::kOk);
  for (auto mode : {HAL_SPI_kMode0, HAL_SPI_kMode1, HAL_SPI_kMode2,
                    HAL_SPI_kMode3}) {
    EXPECT_EQ(fixture.manager.SetMode(HAL_SPI_kMXP, mode), SPIResult::kOk);
    EXPECT_EQ(fixture.manager.GetMode(HAL_SPI_kMXP), mode);
  }
  EXPECT_EQ(fixture.manager.SetMode(HAL_SPI_kMXP,
                                    static_cast<HAL_SPIMode>(4)),
            SPIResult::kInvalidMode);
  EXPECT_EQ(fixture.manager.SetChipSelectActiveLow(HAL_SPI_kMXP, false),
            SPIResult::kOk);
  EXPECT_FALSE(fixture.state->config.chipSelectActiveLow);
  EXPECT_EQ(fixture.manager.SetChipSelectActiveLow(HAL_SPI_kMXP, true),
            SPIResult::kOk);
  EXPECT_TRUE(fixture.state->config.chipSelectActiveLow);
  EXPECT_EQ(fixture.state->reconfigures, 6);
}

TEST(VMXSPIInternalTest, AutoSPIIsExplicitlyUnsupported) {
  SPIFixture fixture;
  EXPECT_EQ(fixture.manager.AutoUnsupported(HAL_SPI_kMXP),
            SPIResult::kAutoUnsupported);
  EXPECT_EQ(fixture.manager.AutoUnsupported(HAL_SPI_kOnboardCS0),
            SPIResult::kUnsupportedPort);
}

TEST(VMXSPIInternalTest, CommDIOReservationsPreventPhysicalConflicts) {
  {
    SPIFixture fixture;
    ASSERT_TRUE(fixture.registry
                    .Reserve(fixture.map.spiCLK, DigitalChannelOwner::kDIO,
                             "test DIO")
                    .reserved);
    EXPECT_EQ(fixture.manager.Initialize(HAL_SPI_kMXP),
              SPIResult::kResourceConflict);
  }
  {
    SPIFixture fixture;
    ASSERT_TRUE(fixture.registry
                    .Reserve(fixture.map.spiMOSI, DigitalChannelOwner::kI2C,
                             "test I2C")
                    .reserved);
    EXPECT_EQ(fixture.manager.Initialize(HAL_SPI_kMXP),
              SPIResult::kResourceConflict);
  }
  {
    SPIFixture fixture;
    ASSERT_TRUE(fixture.registry
                    .Reserve(fixture.map.spiMISO, DigitalChannelOwner::kUART,
                             "test UART")
                    .reserved);
    EXPECT_EQ(fixture.manager.Initialize(HAL_SPI_kMXP),
              SPIResult::kResourceConflict);
  }
}

TEST(VMXSPIInternalTest, SerializesTransferAndCloseRace) {
  SPIFixture fixture;
  fixture.state->blockTransfer = true;
  ASSERT_EQ(fixture.manager.Initialize(HAL_SPI_kMXP), SPIResult::kOk);
  uint8_t send = 1;
  uint8_t receive = 0;
  int32_t transferred = -1;
  std::thread transfer{[&] {
    EXPECT_EQ(fixture.manager.Transaction(HAL_SPI_kMXP, &send, &receive, 1,
                                          transferred),
              SPIResult::kOk);
  }};
  while (!fixture.state->transferEntered.load()) {
    std::this_thread::yield();
  }
  EXPECT_EQ(fixture.manager.Close(HAL_SPI_kMXP), SPIResult::kOk);
  transfer.join();
  EXPECT_EQ(fixture.manager.GetHandle(HAL_SPI_kMXP), 0);
}

TEST(VMXSPIInternalTest, RejectsInvalidPhysicalMapping) {
  SPIFixture fixture;
  fixture.map.spiCS = fixture.map.spiCLK;
  EXPECT_EQ(fixture.manager.Initialize(HAL_SPI_kMXP),
            SPIResult::kUnsupportedConfig);
}

}  // namespace
}  // namespace hal::vmx
