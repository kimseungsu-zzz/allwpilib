// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "../../../../main/native/vmx/AddressableLEDInternal.h"
#include "../../../../main/native/vmx/VMXAccelerometerInternal.h"
#include "../../../../main/native/vmx/VMXRTCInternal.h"

namespace hal::vmx {
namespace {

class FakeAddressableLEDBackend final : public AddressableLEDBackend {
 public:
  bool Configure(int32_t physicalChannel,
                 const AddressableLEDConfiguration& configuration,
                 const std::vector<HAL_AddressableLEDData>& data,
                 bool running) noexcept override {
    ++configureCalls;
    channel = physicalChannel;
    lastConfiguration = configuration;
    lastData = data;
    configuredRunning = running;
    return !failConfigure;
  }

  bool Write(const std::vector<HAL_AddressableLEDData>& data) noexcept override {
    ++writeCalls;
    lastData = data;
    return !failWrite;
  }

  bool Render() noexcept override {
    ++renderCalls;
    return !failRender;
  }

  bool Start() noexcept override {
    ++startCalls;
    return !failStart;
  }

  bool Stop() noexcept override {
    ++stopCalls;
    return !failStop;
  }

  void Close() noexcept override { ++closeCalls; }

  int32_t channel = -1;
  int configureCalls = 0;
  int writeCalls = 0;
  int renderCalls = 0;
  int startCalls = 0;
  int stopCalls = 0;
  int closeCalls = 0;
  bool configuredRunning = false;
  bool failConfigure = false;
  bool failWrite = false;
  bool failRender = false;
  bool failStart = false;
  bool failStop = false;
  AddressableLEDConfiguration lastConfiguration;
  std::vector<HAL_AddressableLEDData> lastData;
};

}  // namespace

TEST(VMXAddressableLEDIntegrationTest, LifecycleAndDataValidation) {
  auto* backend = static_cast<FakeAddressableLEDBackend*>(nullptr);
  AddressableLEDPort port;
  EXPECT_EQ(port.Initialize(
                28, "test",
                [&backend] {
                  auto result = std::make_unique<FakeAddressableLEDBackend>();
                  backend = result.get();
                  return result;
                }),
            AddressableLEDResult::kOk);
  EXPECT_EQ(port.GetLength(), 1);

  HAL_AddressableLEDData pixel{3, 2, 1, 0};
  EXPECT_EQ(port.Write(nullptr, 1), AddressableLEDResult::kInvalidParameter);
  EXPECT_EQ(port.Write(&pixel, 2), AddressableLEDResult::kInvalidParameter);
  EXPECT_EQ(port.Write(&pixel, 1), AddressableLEDResult::kOk);
  EXPECT_EQ(backend->lastData[0].r, 1);
  EXPECT_EQ(backend->lastData[0].g, 2);
  EXPECT_EQ(backend->lastData[0].b, 3);

  EXPECT_EQ(port.Start(), AddressableLEDResult::kOk);
  EXPECT_TRUE(port.IsRunning());
  EXPECT_EQ(backend->renderCalls, 1);
  EXPECT_EQ(port.Start(), AddressableLEDResult::kOk);
  EXPECT_EQ(backend->startCalls, 1);
  EXPECT_EQ(port.Stop(), AddressableLEDResult::kOk);
  EXPECT_FALSE(port.IsRunning());
  port.Close();
}

TEST(VMXAddressableLEDIntegrationTest, TimingRejectsUnrepresentablePeriods) {
  int32_t frequency = 0;
  EXPECT_TRUE(ConvertAddressableLEDBitTiming(300, 950, 600, 650,
                                             frequency));
  EXPECT_EQ(frequency, 800000);
  // The stock WPILib tuple has distinct bit periods. VMX has one frequency,
  // so accepting it would silently discard part of the requested timing.
  EXPECT_FALSE(ConvertAddressableLEDBitTiming(400, 900, 900, 600,
                                              frequency));
  EXPECT_FALSE(ConvertAddressableLEDBitTiming(-1, 1, 1, 1, frequency));
}

TEST(VMXAddressableLEDIntegrationTest, PhysicalRegistryRejectsSensorOverlap) {
  DigitalChannelRegistry registry;
  EXPECT_TRUE(
      registry.Reserve(28, DigitalChannelOwner::kSPI, "SPI clock").reserved);
  EXPECT_FALSE(registry
                   .Reserve(28, DigitalChannelOwner::kAddressableLED,
                            "addressable LED")
                   .reserved);
  registry.Release(28, DigitalChannelOwner::kSPI);
  EXPECT_TRUE(registry
                  .Reserve(28, DigitalChannelOwner::kAddressableLED,
                           "addressable LED")
                  .reserved);
}

TEST(VMXBuiltInAccelerometerIntegrationTest, RawAxesAndLocalStandbyState) {
  BuiltInAccelerometerState state{[](int axis) {
    return axis == 0 ? 0.25 : axis == 1 ? -0.5 : 1.0;
  }};
  EXPECT_DOUBLE_EQ(state.GetAxis(0), 0.25);
  EXPECT_DOUBLE_EQ(state.GetAxis(1), -0.5);
  EXPECT_DOUBLE_EQ(state.GetAxis(2), 1.0);
  state.SetActive(false);
  EXPECT_DOUBLE_EQ(state.GetAxis(0), 0.0);
  state.SetActive(true);
  state.SetRange(HAL_AccelerometerRange_k8G);
  EXPECT_EQ(state.GetRequestedRange(), HAL_AccelerometerRange_k8G);
}

TEST(VMXBuiltInAccelerometerIntegrationTest, NonfiniteReaderValuesAreSafe) {
  BuiltInAccelerometerState state{[](int) {
    return std::numeric_limits<double>::quiet_NaN();
  }};
  EXPECT_DOUBLE_EQ(state.GetAxis(0), 0.0);
}

TEST(VMXRTCIntegrationTest, BootstrapValidatesAndSetsWallClockOnly) {
  VMXRTCDateTime rtc{2026, 8, 21, 5, 12, 34, 56, 789};
  VMXRTCDateTime received;
  VMXRTCSystemClockBootstrap bootstrap{
      [&rtc](VMXRTCDateTime& value) {
        value = rtc;
        return true;
      },
      [&received](const VMXRTCDateTime& value) {
        received = value;
        return true;
      }};
  EXPECT_TRUE(bootstrap.Bootstrap());
  EXPECT_EQ(received.year, 2026);
  EXPECT_EQ(received.milliseconds, 789);
  EXPECT_FALSE(IsValidVMXRTCDateTime(VMXRTCDateTime{2026, 2, 30}));
}

}  // namespace hal::vmx
