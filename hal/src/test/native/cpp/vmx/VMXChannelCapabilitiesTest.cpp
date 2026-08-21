// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <algorithm>

#include "../../../../main/native/vmx/VMXChannelCapabilities.h"
#include "../../../../main/native/vmx/VMXConstants.h"
#include "../../../../main/native/vmx/DigitalChannelRegistry.h"

namespace hal::vmx {

TEST(VMXChannelMappingTest, LogicalChannelsTranslateToPhysicalChannels) {
  EXPECT_EQ(ToVMXDigitalChannel(0), 0);
  EXPECT_EQ(ToVMXDigitalChannel(11), 11);
  EXPECT_EQ(ToVMXDigitalChannel(12), 12);
  EXPECT_EQ(ToVMXDigitalChannel(21), 21);
  EXPECT_EQ(ToVMXDigitalChannel(22), 26);
  EXPECT_EQ(ToVMXDigitalChannel(29), 33);
  EXPECT_EQ(ToVMXAnalogChannel(0), 22);
  EXPECT_EQ(ToVMXAnalogChannel(3), 25);
}

TEST(VMXChannelMappingTest, PhysicalBanksAndOfficialPairsAreExplicit) {
  EXPECT_TRUE(IsFlexDIOChannel(0));
  EXPECT_TRUE(IsFlexDIOChannel(11));
  EXPECT_TRUE(IsHighCurrentDIOChannel(12));
  EXPECT_TRUE(IsHighCurrentDIOChannel(21));
  EXPECT_TRUE(IsCommDIOChannel(22));
  EXPECT_TRUE(IsCommDIOChannel(29));
  EXPECT_TRUE(IsVMXEncoderPair(0, 1));
  EXPECT_TRUE(IsVMXEncoderPair(8, 9));
  EXPECT_FALSE(IsVMXEncoderPair(1, 2));
  EXPECT_FALSE(IsVMXEncoderPair(10, 11));
  EXPECT_TRUE(IsVMXCounterPair(10, 11));
  EXPECT_FALSE(IsVMXCounterPair(1, 2));
}

TEST(VMXChannelMappingTest, CommunicationAliasesUseDocumentedPhysicalPins) {
  const auto map = kDefaultVMXCommDIOChannelMap;
  EXPECT_EQ(map.uartTX, 26);
  EXPECT_EQ(map.uartRX, 27);
  EXPECT_EQ(map.spiCLK, 28);
  EXPECT_EQ(map.spiMOSI, 29);
  EXPECT_EQ(map.spiMISO, 30);
  EXPECT_EQ(map.spiCS, 31);
  EXPECT_EQ(map.i2cSDA, 32);
  EXPECT_EQ(map.i2cSCL, 33);
  EXPECT_NE(map.uartRX, map.spiCLK);
  EXPECT_NE(map.spiCS, map.i2cSDA);
}

TEST(VMXChannelMappingTest, CommunicationBusesHaveDisjointPhysicalResources) {
  const auto map = kDefaultVMXCommDIOChannelMap;
  std::array<int32_t, 8> channels{{map.uartTX, map.uartRX, map.spiCLK,
                                   map.spiMOSI, map.spiMISO, map.spiCS,
                                   map.i2cSDA, map.i2cSCL}};
  std::sort(channels.begin(), channels.end());
  EXPECT_EQ(channels, (std::array<int32_t, 8>{{26, 27, 28, 29, 30, 31, 32,
                                               33}}));
}

TEST(VMXChannelMappingTest, UartSpiAndI2CCanCoexistOnDistinctCommDioPins) {
  DigitalChannelRegistry registry;
  const auto map = kDefaultVMXCommDIOChannelMap;
  EXPECT_TRUE(
      registry.Reserve(map.uartTX, DigitalChannelOwner::kUART, "UART TX")
          .reserved);
  EXPECT_TRUE(
      registry.Reserve(map.uartRX, DigitalChannelOwner::kUART, "UART RX")
          .reserved);
  for (int32_t channel : {map.spiCLK, map.spiMOSI, map.spiMISO, map.spiCS}) {
    EXPECT_TRUE(
        registry.Reserve(channel, DigitalChannelOwner::kSPI, "SPI")
            .reserved);
  }
  EXPECT_TRUE(
      registry.Reserve(map.i2cSDA, DigitalChannelOwner::kI2C, "I2C SDA")
          .reserved);
  EXPECT_TRUE(
      registry.Reserve(map.i2cSCL, DigitalChannelOwner::kI2C, "I2C SCL")
          .reserved);
  EXPECT_FALSE(
      registry.Reserve(map.spiCS, DigitalChannelOwner::kDIO, "overlap").reserved);
}

TEST(VMXChannelCapabilityTest, MockSdkCapabilitiesModelJumperAndCommDio) {
  VMXCapabilityProvider mock{[](int32_t physical, VMXCapability capability) {
    if (physical >= 12 && physical <= 21) {
      return capability == VMXCapability::kDigitalOutput ||
             capability == VMXCapability::kPWMGenerator;
    }
    if (physical >= 26 && physical <= 33) {
      return capability == VMXCapability::kDigitalInput ||
             capability == VMXCapability::kInterruptInput;
    }
    return capability == VMXCapability::kDigitalInput ||
           capability == VMXCapability::kDigitalOutput ||
           capability == VMXCapability::kInterruptInput ||
           capability == VMXCapability::kPWMGenerator;
  }};

  EXPECT_TRUE(mock.SupportsLogicalDIO(12, VMXCapability::kDigitalOutput));
  EXPECT_FALSE(mock.SupportsLogicalDIO(12, VMXCapability::kDigitalInput));
  EXPECT_TRUE(mock.SupportsLogicalDIO(22, VMXCapability::kDigitalInput));
  EXPECT_FALSE(mock.SupportsLogicalDIO(22, VMXCapability::kPWMGenerator));
  EXPECT_TRUE(mock.SupportsLogicalDIO(0, VMXCapability::kDigitalInput));
  EXPECT_TRUE(mock.SupportsLogicalDIO(0, VMXCapability::kDigitalOutput));
}

}  // namespace hal::vmx
