// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "../../../../main/native/vmx/VMXCANInternal.h"

namespace hal::vmx {
namespace {

VMXCANFrame MakeFrame(uint32_t id, uint8_t value, uint64_t timestampUs) {
  VMXCANFrame frame;
  frame.messageID = id;
  frame.dataSize = 2;
  frame.data[0] = value;
  frame.data[1] = static_cast<uint8_t>(value + 1);
  frame.sysTimeStampUS = timestampUs;
  frame.timeStampMS = static_cast<uint32_t>(timestampUs / 1000);
  return frame;
}

TEST(VMXCANTest, ValidatesStandardExtendedAndRemoteIds) {
  EXPECT_TRUE(VMXCANIsValidMessageID(HAL_CAN_IS_FRAME_11BIT | 0x7ff));
  EXPECT_FALSE(VMXCANIsValidMessageID(HAL_CAN_IS_FRAME_11BIT | 0x800));
  EXPECT_TRUE(VMXCANIsValidMessageID(0x1fffffff));
  EXPECT_TRUE(VMXCANIsValidMessageID(HAL_CAN_IS_FRAME_REMOTE | 0x123));
  EXPECT_FALSE(VMXCANIsValidMessageID(0x20000000));
}

TEST(VMXCANTest, SendPreservesOneShotPeriodicAndCancelSemantics) {
  std::vector<int32_t> periods;
  VMXCANBackend backend;
  backend.send = [&periods](const VMXCANMessage& message, int32_t period,
                            int32_t& status) {
    EXPECT_EQ(message.dataSize, 1);
    periods.push_back(period);
    status = HAL_SUCCESS;
    return true;
  };
  VMXCANReceiveManager manager{std::move(backend)};
  VMXCANMessage message;
  message.messageID = HAL_CAN_IS_FRAME_11BIT | 0x123;
  message.dataSize = 1;
  int32_t status = 0;
  ASSERT_TRUE(manager.Send(message, HAL_CAN_SEND_PERIOD_NO_REPEAT, status));
  ASSERT_TRUE(manager.Send(message, 20, status));
  ASSERT_TRUE(
      manager.Send(message, HAL_CAN_SEND_PERIOD_STOP_REPEATING, status));
  EXPECT_EQ(periods,
            (std::vector<int32_t>{HAL_CAN_SEND_PERIOD_NO_REPEAT, 20, -1}));
  message.dataSize = 9;
  EXPECT_FALSE(manager.Send(message, 0, status));
  EXPECT_EQ(status, PARAMETER_OUT_OF_RANGE);
}

TEST(VMXCANTest, ReceiveLatestUsesNewestMatchingFrameAndTimestamp) {
  uint64_t now = 2'000'000;
  VMXCANReceiveManager manager{{}, [&now] { return now; }};
  manager.InjectFrame(MakeFrame(0x100, 1, 1'000'000));
  manager.InjectFrame(MakeFrame(0x101, 2, 1'500'000));
  manager.InjectFrame(MakeFrame(0x100, 3, 2'000'000));

  VMXCANFrame frame;
  int32_t status = 0;
  ASSERT_TRUE(manager.ReceiveLatest(0x100, 0x7ff, frame, status));
  EXPECT_EQ(status, HAL_SUCCESS);
  EXPECT_EQ(frame.data[0], 3);
  EXPECT_EQ(frame.timeStampMS, 2000U);
  EXPECT_FALSE(manager.ReceiveLatest(0x200, 0x7ff, frame, status));
  EXPECT_EQ(status, HAL_ERR_CANSessionMux_MessageNotFound);
}

TEST(VMXCANTest, PublicStreamsAreSoftwareOwnedAndOverflowIsReported) {
  VMXCANReceiveManager manager;
  uint32_t stream = 0;
  int32_t status = 0;
  ASSERT_TRUE(manager.OpenStream(stream, 0x120, 0x7ff, 2, status));
  manager.InjectFrame(MakeFrame(0x120, 1, 1'000));
  manager.InjectFrame(MakeFrame(0x120, 2, 2'000));
  manager.InjectFrame(MakeFrame(0x120, 3, 3'000));

  std::array<VMXCANFrame, 4> frames{};
  uint32_t read = 0;
  EXPECT_FALSE(manager.ReadStream(stream, frames.data(), frames.size(), read,
                                  status));
  EXPECT_EQ(status, HAL_CAN_BUFFER_OVERRUN);
  EXPECT_EQ(read, 0U);
  ASSERT_TRUE(manager.ReadStream(stream, frames.data(), frames.size(), read,
                                 status));
  ASSERT_EQ(read, 2U);
  EXPECT_EQ(frames[0].data[0], 2);
  EXPECT_EQ(frames[1].data[0], 3);
  EXPECT_TRUE(manager.CloseStream(stream));
  EXPECT_FALSE(manager.CloseStream(stream));
}

TEST(VMXCANTest, StreamFiltersAndInvalidHandlesAreRejected) {
  VMXCANReceiveManager manager;
  uint32_t stream = 0;
  int32_t status = 0;
  EXPECT_FALSE(manager.OpenStream(stream, 0, 0, 0, status));
  EXPECT_EQ(status, PARAMETER_OUT_OF_RANGE);
  ASSERT_TRUE(manager.OpenStream(stream, 0x100, 0x700, 4, status));
  manager.InjectFrame(MakeFrame(0x155, 1, 1'000));
  manager.InjectFrame(MakeFrame(0x255, 2, 2'000));
  std::array<VMXCANFrame, 2> frames{};
  uint32_t read = 0;
  ASSERT_TRUE(manager.ReadStream(stream, frames.data(), frames.size(), read,
                                 status));
  ASSERT_EQ(read, 1U);
  EXPECT_EQ(frames[0].messageID, 0x155U);
  EXPECT_FALSE(manager.ReadStream(999, frames.data(), frames.size(), read,
                                  status));
  EXPECT_EQ(status, HAL_HANDLE_ERROR);
}

TEST(VMXCANTest, ApiIdPackingAndDeviceHandlesRemainLogical) {
  VMXCANApiState state;
  int32_t status = 0;
  const auto handle = state.Initialize(HAL_CAN_Man_kREV, 63,
                                       HAL_CAN_Dev_kMotorController, status);
  ASSERT_NE(handle, HAL_kInvalidHandle);
  EXPECT_EQ(status, HAL_SUCCESS);
  auto device = state.Get(handle);
  ASSERT_NE(device, nullptr);
  EXPECT_EQ(VMXCreateCANId(device->manufacturer, device->deviceId,
                           device->deviceType, 0x3ff),
            (2U << 24) | (5U << 16) | (0x3ffU << 6) | 63U);
  EXPECT_EQ(state.Initialize(HAL_CAN_Man_kREV, 64,
                             HAL_CAN_Dev_kMotorController, status),
            HAL_kInvalidHandle);
  EXPECT_EQ(status, PARAMETER_OUT_OF_RANGE);
  EXPECT_EQ(state.Initialize(static_cast<HAL_CANManufacturer>(99), 1,
                             HAL_CAN_Dev_kMotorController, status),
            HAL_kInvalidHandle);
  EXPECT_EQ(state.Clean(handle), device);
  EXPECT_EQ(state.Get(handle), nullptr);
}

TEST(VMXCANTest, ApiReadNewLatestAndTimeoutUseGenerationAndAge) {
  uint64_t now = 1'000'000;
  VMXCANReceiveManager manager{{}, [&now] { return now; }};
  VMXCANDevice device;
  device.manufacturer = HAL_CAN_Man_kTeamUse;
  device.deviceId = 4;
  device.deviceType = HAL_CAN_Dev_kMiscellaneous;
  const auto id = VMXCreateCANId(device.manufacturer, device.deviceId,
                                  device.deviceType, 17);
  manager.InjectFrame(MakeFrame(id, 7, 900'000));

  std::array<uint8_t, 8> data{};
  int32_t length = 8;
  uint64_t timestamp = 0;
  int32_t status = 0;
  ASSERT_TRUE(VMXCANReadApiPacket(manager, device, 17, data.data(), &length,
                                  &timestamp, 0, 0, status));
  EXPECT_EQ(length, 2);
  EXPECT_EQ(data[0], 7);
  EXPECT_EQ(timestamp, 900U);
  length = 8;
  EXPECT_FALSE(VMXCANReadApiPacket(manager, device, 17, data.data(), &length,
                                   &timestamp, 0, 0, status));
  EXPECT_EQ(status, HAL_ERR_CANSessionMux_MessageNotFound);

  now = 1'000'000;
  length = 8;
  ASSERT_TRUE(VMXCANReadApiPacket(manager, device, 17, data.data(), &length,
                                  &timestamp, 100, 2, status));
  now = 1'100'001;
  length = 8;
  EXPECT_FALSE(VMXCANReadApiPacket(manager, device, 17, data.data(), &length,
                                   &timestamp, 100, 2, status));
  EXPECT_EQ(status, HAL_CAN_TIMEOUT);
}

TEST(VMXCANTest, StatusFacadePropagatesHardwareFields) {
  VMXCANBusStatus expected;
  expected.percentBusUtilization = 42.5F;
  expected.busOffCount = 2;
  expected.txFullCount = 3;
  expected.receiveErrorCount = 4;
  expected.transmitErrorCount = 5;
  expected.busWarning = true;
  expected.hwRxOverflow = true;
  VMXCANBackend backend;
  backend.status = [expected](VMXCANBusStatus& value, int32_t& status) {
    value = expected;
    status = HAL_SUCCESS;
    return true;
  };
  VMXCANReceiveManager manager{std::move(backend)};
  VMXCANBusStatus actual;
  int32_t status = 0;
  ASSERT_TRUE(manager.GetStatus(actual, status));
  EXPECT_EQ(status, HAL_SUCCESS);
  EXPECT_FLOAT_EQ(actual.percentBusUtilization, 42.5F);
  EXPECT_EQ(actual.busOffCount, 2U);
  EXPECT_TRUE(actual.busWarning);
  EXPECT_TRUE(actual.hwRxOverflow);
}

}  // namespace
}  // namespace hal::vmx
