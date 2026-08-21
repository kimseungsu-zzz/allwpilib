// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <atomic>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "../../../../main/native/vmx/DriverStationInternal.h"

namespace hal::vmx {
namespace {

std::vector<uint8_t> MakeUdpPacket() {
  // KauaiLabs UDP v1: six-byte header followed by one joystick tag.
  std::vector<uint8_t> packet{0x12, 0x34, 1, 0x06, 0x00,
                              HAL_AllianceStationID_kBlue2};
  const std::vector<uint8_t> joystick{2, 127, 0, 3, 0x05, 1, 0x00, 90};
  packet.push_back(static_cast<uint8_t>(joystick.size() + 1));
  packet.push_back(12);
  packet.insert(packet.end(), joystick.begin(), joystick.end());
  return packet;
}

TEST(VMXDriverStationParserTest, DecodesKauaiLabsUdpV1State) {
  VMXDriverStationPacket packet;
  const auto data = MakeUdpPacket();
  ASSERT_TRUE(VMXDriverStationPacketParser::ParseUdp(data.data(), data.size(),
                                                    packet));
  EXPECT_EQ(packet.wireSequence, 0x1234);
  EXPECT_TRUE(packet.controlWord.enabled);
  EXPECT_TRUE(packet.controlWord.autonomous);
  EXPECT_TRUE(packet.controlWord.dsAttached);
  EXPECT_EQ(packet.alliance, HAL_AllianceStationID_kBlue2);
  EXPECT_EQ(packet.axes[0].count, 2);
  EXPECT_FLOAT_EQ(packet.axes[0].axes[0], 1.0F);
  EXPECT_FLOAT_EQ(packet.axes[0].axes[1], 0.0F);
  EXPECT_EQ(packet.buttons[0].count, 3);
  EXPECT_EQ(packet.buttons[0].buttons, 0x05U);
  EXPECT_EQ(packet.povs[0].count, 1);
  EXPECT_EQ(packet.povs[0].povs[0], 90);
}

TEST(VMXDriverStationParserTest, RejectsMalformedOrTruncatedPackets) {
  auto data = MakeUdpPacket();
  data[6] = 0xff;
  VMXDriverStationPacket packet;
  EXPECT_FALSE(VMXDriverStationPacketParser::ParseUdp(data.data(), data.size(),
                                                     packet));
  data = MakeUdpPacket();
  data.pop_back();
  EXPECT_FALSE(VMXDriverStationPacketParser::ParseUdp(data.data(), data.size(),
                                                     packet));
  EXPECT_FALSE(VMXDriverStationPacketParser::ParseUdp(nullptr, 0, packet));
}

TEST(VMXDriverStationParserTest, DecodesTcpDescriptorsMatchInfoAndGameData) {
  std::vector<uint8_t> descriptor{0, 1, 7, 4, 'X', 'B', 'o', 'x', 2, 3, 4, 3, 0};
  std::vector<uint8_t> match{3, 'E', 'v', 't', 2, 0, 42, 1};
  std::vector<uint8_t> game{'G', '1'};
  std::vector<uint8_t> data;
  auto append = [&data](uint8_t tag, const std::vector<uint8_t>& payload) {
    const uint16_t length = static_cast<uint16_t>(payload.size() + 1);
    data.push_back(static_cast<uint8_t>(length >> 8));
    data.push_back(static_cast<uint8_t>(length));
    data.push_back(tag);
    data.insert(data.end(), payload.begin(), payload.end());
  };
  append(2, descriptor);
  append(7, match);
  append(14, game);

  VMXDriverStationPacket packet;
  ASSERT_TRUE(VMXDriverStationPacketParser::ParseTcp(data.data(), data.size(),
                                                    packet));
  EXPECT_STREQ(packet.descriptors[0].name, "XBox");
  EXPECT_EQ(packet.descriptors[0].axisCount, 2);
  EXPECT_EQ(packet.descriptors[0].buttonCount, 3);
  EXPECT_EQ(packet.matchInfo.matchType, HAL_kMatchType_qualification);
  EXPECT_EQ(packet.matchInfo.matchNumber, 42);
  EXPECT_EQ(packet.matchInfo.gameSpecificMessageSize, 2);
}

TEST(VMXDriverStationStateTest, FailsSafeUntilFreshDataAndOnTimeout) {
  uint64_t now = 100;
  int wakeups = 0;
  VMXDriverStationState state{[&now] { return now; }, [&wakeups] { ++wakeups; }};
  HAL_ControlWord word{};
  EXPECT_EQ(state.GetControlWord(&word), HAL_SUCCESS);
  EXPECT_FALSE(word.enabled);
  EXPECT_FALSE(word.dsAttached);

  VMXDriverStationPacket packet;
  const auto data = MakeUdpPacket();
  ASSERT_TRUE(VMXDriverStationPacketParser::ParseUdp(data.data(), data.size(),
                                                    packet));
  state.CommitUdp(packet, now);
  EXPECT_TRUE(state.GetControlWord(&word) == HAL_SUCCESS && word.enabled);
  EXPECT_TRUE(state.Refresh());
  EXPECT_FALSE(state.Refresh());
  now += 2'000'001;
  EXPECT_TRUE(state.PollTimeout(now));
  EXPECT_FALSE(state.GetControlWord(&word) == HAL_SUCCESS && word.enabled);
  EXPECT_FALSE(word.dsAttached);
  EXPECT_GE(wakeups, 2);

  packet.wireSequence = 0;
  packet.controlWord.enabled = true;
  packet.controlWord.dsAttached = true;
  now += 1;
  state.CommitUdp(packet, now);
  EXPECT_EQ(state.GetControlWord(&word), HAL_SUCCESS);
  EXPECT_TRUE(word.enabled);
  EXPECT_TRUE(word.dsAttached);
  EXPECT_TRUE(state.Refresh());
}

TEST(VMXDriverStationStateTest, JoystickDescriptorMatchAndOutputSemantics) {
  VMXDriverStationState state;
  bool outputCalled = false;
  state.SetOutput([&](int32_t joystick, int64_t outputs, int32_t left,
                      int32_t right) {
    outputCalled = joystick == 0 && outputs == 3 && left == 4 && right == 5;
    return true;
  });
  EXPECT_EQ(state.SetJoystickOutputs(0, 3, 4, 5), HAL_SUCCESS);
  EXPECT_TRUE(outputCalled);
  EXPECT_EQ(state.SetJoystickOutputs(-1, 0, 0, 0), PARAMETER_OUT_OF_RANGE);

  VMXDriverStationPacket packet;
  packet.valid = true;
  packet.descriptors[0].axisCount = 1;
  packet.descriptors[0].axisTypes[0] = 9;
  std::strcpy(packet.descriptors[0].name, "test");
  packet.matchInfo.matchType = HAL_kMatchType_practice;
  packet.matchInfo.matchNumber = 7;
  state.CommitUdp(packet, 1);
  HAL_JoystickDescriptor descriptor{};
  ASSERT_EQ(state.GetDescriptor(0, &descriptor), HAL_SUCCESS);
  EXPECT_STREQ(descriptor.name, "test");
  EXPECT_EQ(state.GetAxisType(0, 0), 9);
  HAL_MatchInfo info{};
  ASSERT_EQ(state.GetMatchInfo(&info), HAL_SUCCESS);
  EXPECT_EQ(info.matchNumber, 7);
}

TEST(VMXDriverStationStateTest, ShutdownRejectsMutationsAndReadsSafeDefaults) {
  VMXDriverStationState state;
  state.Shutdown();
  HAL_ControlWord word{};
  EXPECT_EQ(state.GetControlWord(&word), INCOMPATIBLE_STATE);
  EXPECT_FALSE(word.enabled);
  EXPECT_EQ(state.SetJoystickOutputs(0, 0, 0, 0), INCOMPATIBLE_STATE);
  EXPECT_FALSE(state.Refresh());
}

}  // namespace
}  // namespace hal::vmx
