// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>

#include "../../../../main/native/vmx/VMXCANInternal.h"

namespace hal::vmx {
namespace {

VMXCANFrame MakeFrame(uint32_t id, const uint8_t* data, uint8_t size) {
  VMXCANFrame frame;
  frame.messageID = id;
  frame.dataSize = size;
  std::copy(data, data + size, frame.data.begin());
  frame.sysTimeStampUS = 1'000'000;
  frame.timeStampMS = 1'000;
  return frame;
}

std::array<uint8_t, 8> ReadFrame(VMXCANReceiveManager& manager,
                                 VMXCANDevice& device, int api) {
  const auto id = VMXCreateCANId(device.manufacturer, device.deviceId,
                                 device.deviceType, api);
  std::array<uint8_t, 8> data{};
  int32_t length = data.size();
  uint64_t timestamp = 0;
  int32_t status = HAL_SUCCESS;
  EXPECT_TRUE(VMXCANReadApiPacket(manager, device, api, data.data(), &length,
                                  &timestamp, 0, 1, status));
  EXPECT_EQ(status, HAL_SUCCESS);
  EXPECT_EQ(timestamp, 1'000U);
  EXPECT_NE(id, 0U);
  return data;
}

TEST(VMXPowerPneumaticsCANFrameTest,
     CTREAndREVDevicesRetainLogicalCanIdentityAndPayload) {
  VMXCANReceiveManager manager;

  VMXCANDevice pcm;
  pcm.manufacturer = HAL_CAN_Man_kCTRE;
  pcm.deviceType = HAL_CAN_Dev_kPneumatics;
  pcm.deviceId = 2;
  const uint8_t pcmStatus[] = {0x05, 0x93, 0xD0, 0xA0, 0x28, 0x21, 0x00, 0x01};
  manager.InjectFrame(MakeFrame(
      VMXCreateCANId(pcm.manufacturer, pcm.deviceId, pcm.deviceType, 0x50),
      pcmStatus, sizeof(pcmStatus)));
  const auto pcmData = ReadFrame(manager, pcm, 0x50);
  EXPECT_EQ(pcmData[0], 0x05);
  EXPECT_TRUE((pcmData[1] & 0x01U) != 0);  // compressor on
  EXPECT_TRUE((pcmData[1] & 0x80U) != 0);  // pressure switch

  VMXCANDevice ph;
  ph.manufacturer = HAL_CAN_Man_kREV;
  ph.deviceType = HAL_CAN_Dev_kPneumatics;
  ph.deviceId = 3;
  // PH status-0 fixture: channel bits, compressor, and digital pressure
  // switch are all set in the same raw payload the shared decoder consumes.
  const std::array<uint8_t, 8> phData{{0x84, 0x81, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00}};
  const auto phApi = 0x00;
  manager.InjectFrame(MakeFrame(
      VMXCreateCANId(ph.manufacturer, ph.deviceId, ph.deviceType, phApi),
      phData.data(), phData.size()));
  const auto receivedPh = ReadFrame(manager, ph, phApi);
  EXPECT_EQ(receivedPh, phData);
}

TEST(VMXPowerPneumaticsCANFrameTest, PDPAndPDHStatusStreamsAreFrameAddressable) {
  VMXCANReceiveManager manager;

  VMXCANDevice pdp;
  pdp.manufacturer = HAL_CAN_Man_kCTRE;
  pdp.deviceType = HAL_CAN_Dev_kPowerDistribution;
  pdp.deviceId = 4;
  const uint8_t pdpStatus[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
  manager.InjectFrame(MakeFrame(
      VMXCreateCANId(pdp.manufacturer, pdp.deviceId, pdp.deviceType, 0x20),
      pdpStatus, sizeof(pdpStatus)));
  const auto receivedPdp = ReadFrame(manager, pdp, 0x20);
  EXPECT_EQ(receivedPdp[0], 0x12);
  EXPECT_EQ(receivedPdp[7], 0xF0);

  VMXCANDevice pdh;
  pdh.manufacturer = HAL_CAN_Man_kREV;
  pdh.deviceType = HAL_CAN_Dev_kPowerDistribution;
  pdh.deviceId = 5;
  const std::array<uint8_t, 8> pdhData{{0x41, 0x01, 0x8E, 0x02,
                                        0x00, 0x00, 0x00, 0x00}};
  const auto pdhApi = 0x00;
  manager.InjectFrame(MakeFrame(
      VMXCreateCANId(pdh.manufacturer, pdh.deviceId, pdh.deviceType, pdhApi),
      pdhData.data(), pdhData.size()));
  const auto receivedPdh = ReadFrame(manager, pdh, pdhApi);
  EXPECT_EQ(receivedPdh, pdhData);
}

}  // namespace
}  // namespace hal::vmx
