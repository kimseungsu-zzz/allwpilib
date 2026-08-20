// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <thread>

#include "../../../../main/native/vmx/DutyCycleInternal.h"

namespace hal::vmx {
namespace {

class FakeDutyCycleBackend final : public DutyCycleBackend {
 public:
  FakeDutyCycleBackend(uint32_t period, uint32_t high)
      : m_period{period}, m_high{high} {}

  bool GetTiming(uint32_t& period,
                 uint32_t& high) noexcept override {
    period = m_period;
    high = m_high;
    return true;
  }

 private:
  uint32_t m_period;
  uint32_t m_high;
};

class FakeDIOBackend final : public DIOBackend {
 public:
  bool Set(bool) noexcept override { return true; }
  bool Get(bool& value) noexcept override {
    value = false;
    return true;
  }
  bool Pulse(uint32_t) noexcept override { return true; }
  bool IsPulsing(bool& pulsing) noexcept override {
    pulsing = false;
    return true;
  }
};

TEST(VMXDutyCycleTest, ConvertsPeriodHighTimeFrequencyAndOutput) {
  int releases = 0;
  DutyCycleManager manager{
      [](int32_t) {
        return std::unique_ptr<DutyCycleBackend>{
            std::make_unique<FakeDutyCycleBackend>(1000, 250)};
      },
      [](HAL_Handle) { return DutyCycleSourceClaim{DutyCycleResult::kOk, 0}; },
      [&releases](HAL_Handle, int32_t) { ++releases; }};

  auto allocation = manager.Allocate(static_cast<HAL_Handle>(1));
  ASSERT_EQ(allocation.result, DutyCycleResult::kOk);
  EXPECT_EQ(manager.GetFrequency(allocation.handle).second, 1000);
  EXPECT_DOUBLE_EQ(manager.GetOutput(allocation.handle).second, 0.25);
  EXPECT_EQ(manager.GetHighTime(allocation.handle).second, 250000);
  EXPECT_EQ(manager.GetFPGAIndex(allocation.handle).second, 0);
  EXPECT_EQ(manager.GetFPGAIndex(HAL_kInvalidHandle).first,
            DutyCycleResult::kInvalidHandle);
  manager.Free(allocation.handle);
  EXPECT_EQ(releases, 1);
}

TEST(VMXDutyCycleTest, ZeroPeriodIsDisconnectedAndHighTimeOverflowFails) {
  DutyCycleManager zero{
      [](int32_t) {
        return std::unique_ptr<DutyCycleBackend>{
            std::make_unique<FakeDutyCycleBackend>(0, 0)};
      },
      [](HAL_Handle) { return DutyCycleSourceClaim{DutyCycleResult::kOk, 2}; },
      [](HAL_Handle, int32_t) {}};
  auto zeroAllocation = zero.Allocate(static_cast<HAL_Handle>(2));
  ASSERT_EQ(zeroAllocation.result, DutyCycleResult::kOk);
  EXPECT_EQ(zero.GetFrequency(zeroAllocation.handle).second, 0);
  EXPECT_DOUBLE_EQ(zero.GetOutput(zeroAllocation.handle).second, 0.0);

  DutyCycleManager overflow{
      [](int32_t) {
        return std::unique_ptr<DutyCycleBackend>{
            std::make_unique<FakeDutyCycleBackend>(UINT32_MAX,
                                                   UINT32_MAX)};
      },
      [](HAL_Handle) { return DutyCycleSourceClaim{DutyCycleResult::kOk, 4}; },
      [](HAL_Handle, int32_t) {}};
  auto overflowAllocation = overflow.Allocate(static_cast<HAL_Handle>(3));
  ASSERT_EQ(overflowAllocation.result, DutyCycleResult::kOk);
  EXPECT_EQ(overflow.GetHighTime(overflowAllocation.handle).first,
            DutyCycleResult::kOutOfRange);
}

TEST(VMXDutyCycleTest, FlexTimerGroupsAreSharedByPwmCaptureAndPwm) {
  DigitalChannelRegistry registry;
  EXPECT_TRUE(registry
                  .ReserveFlexTimerGroup(0, DigitalChannelOwner::kPWM,
                                         "PWM 0")
                  .reserved);
  EXPECT_FALSE(registry
                   .ReserveFlexTimerGroup(1, DigitalChannelOwner::kDutyCycle,
                                          "DutyCycle 1")
                   .reserved);
  EXPECT_TRUE(registry
                  .ReserveFlexTimerGroup(2, DigitalChannelOwner::kDutyCycle,
                                         "DutyCycle 2")
                  .reserved);
  EXPECT_TRUE(registry
                  .ReserveFlexTimerGroup(4, DigitalChannelOwner::kEncoder,
                                         "Encoder 4/5")
                  .reserved);
  EXPECT_FALSE(registry
                   .ReserveFlexTimerGroup(5, DigitalChannelOwner::kDutyCycle,
                                          "DutyCycle 5")
                   .reserved);
  registry.ReleaseFlexTimerGroup(4, DigitalChannelOwner::kEncoder);
  EXPECT_TRUE(registry
                  .ReserveFlexTimerGroup(5, DigitalChannelOwner::kPWM,
                                         "PWM 5")
                  .reserved);
  registry.ReleaseFlexTimerGroup(0, DigitalChannelOwner::kPWM);
  EXPECT_TRUE(registry
                  .ReserveFlexTimerGroup(1, DigitalChannelOwner::kDutyCycle,
                                         "DutyCycle 1")
                  .reserved);
}

TEST(VMXDutyCycleTest, OnlyFlexDioSourcesCanBeClaimed) {
  DigitalChannelRegistry registry;
  VMXCapabilityProvider capabilities{[](int32_t physical,
                                        VMXCapability capability) {
    if (capability == VMXCapability::kPWMCapture) {
      return physical >= 0 && physical < 12;
    }
    return capability == VMXCapability::kDigitalInput;
  }};
  DIOManager dio{
      [](int32_t, bool) {
        return std::unique_ptr<DIOBackend>{
            std::make_unique<FakeDIOBackend>()};
      },
      registry, &capabilities};

  auto conflicted = dio.Allocate(0, true, "timer conflict source");
  ASSERT_EQ(conflicted.result, DIOResult::kOk);
  ASSERT_TRUE(registry
                  .ReserveFlexTimerGroup(0, DigitalChannelOwner::kPWM,
                                         "PWM owner")
                  .reserved);
  auto blocked = dio.ClaimResourceSource(
      conflicted.handle, DigitalChannelOwner::kDutyCycle, "blocked capture");
  EXPECT_EQ(blocked.result, DIOResult::kAlreadyAllocated);
  EXPECT_EQ(dio.GetValue(conflicted.handle).first, DIOResult::kOk);
  registry.ReleaseFlexTimerGroup(0, DigitalChannelOwner::kPWM);
  auto claimed = dio.ClaimResourceSource(
      conflicted.handle, DigitalChannelOwner::kDutyCycle, "capture");
  ASSERT_EQ(claimed.result, DIOResult::kOk);
  dio.ReleaseResourceSource(conflicted.handle, claimed.channelA,
                            DigitalChannelOwner::kDutyCycle);
  dio.Free(conflicted.handle);

  for (int32_t channel = 0; channel < 12; ++channel) {
    auto source = dio.Allocate(channel, true, "DutyCycle source");
    ASSERT_EQ(source.result, DIOResult::kOk);
    auto claim = dio.ClaimResourceSource(
        source.handle, DigitalChannelOwner::kDutyCycle, "DutyCycle capture");
    EXPECT_EQ(claim.result, DIOResult::kOk);
    dio.ReleaseResourceSource(source.handle, claim.channelA,
                              DigitalChannelOwner::kDutyCycle);
    dio.Free(source.handle);
  }

  for (int32_t channel = 12; channel < kNumDIOChannels; ++channel) {
    auto source = dio.Allocate(channel, true, "unsupported DutyCycle source");
    ASSERT_EQ(source.result, DIOResult::kOk);
    auto claim = dio.ClaimResourceSource(
        source.handle, DigitalChannelOwner::kDutyCycle, "DutyCycle capture");
    EXPECT_EQ(claim.result, DIOResult::kUnsupportedCapability);
    dio.Free(source.handle);
  }
}

TEST(VMXDutyCycleTest, InvalidTimingIsReportedAsHardwareFailure) {
  DutyCycleManager manager{
      [](int32_t) {
        return std::unique_ptr<DutyCycleBackend>{
            std::make_unique<FakeDutyCycleBackend>(100, 101)};
      },
      [](HAL_Handle) { return DutyCycleSourceClaim{DutyCycleResult::kOk, 0}; },
      [](HAL_Handle, int32_t) {}};
  auto allocation = manager.Allocate(static_cast<HAL_Handle>(4));
  ASSERT_EQ(allocation.result, DutyCycleResult::kOk);
  EXPECT_EQ(manager.GetOutput(allocation.handle).first,
            DutyCycleResult::kHardwareFailure);
}

TEST(VMXDutyCycleTest, ActivationRollbackDoubleFreeAndConcurrentReadAreSafe) {
  int releases = 0;
  bool failBackend = true;
  DutyCycleManager manager{
      [&failBackend](int32_t) -> std::unique_ptr<DutyCycleBackend> {
        if (failBackend) {
          return nullptr;
        }
        return std::make_unique<FakeDutyCycleBackend>(4096, 2048);
      },
      [](HAL_Handle) { return DutyCycleSourceClaim{DutyCycleResult::kOk, 6}; },
      [&releases](HAL_Handle, int32_t) { ++releases; }};

  auto failed = manager.Allocate(static_cast<HAL_Handle>(5));
  EXPECT_EQ(failed.result, DutyCycleResult::kHardwareFailure);
  EXPECT_EQ(releases, 1);

  failBackend = false;
  auto allocation = manager.Allocate(static_cast<HAL_Handle>(6));
  ASSERT_EQ(allocation.result, DutyCycleResult::kOk);
  std::atomic_bool stop{false};
  std::thread reader{[&] {
    while (!stop.load()) {
      static_cast<void>(manager.GetFrequency(allocation.handle));
    }
  }};
  manager.Free(allocation.handle);
  stop = true;
  reader.join();
  manager.Free(allocation.handle);
  EXPECT_EQ(releases, 2);
  EXPECT_EQ(manager.GetFrequency(allocation.handle).first,
            DutyCycleResult::kInvalidHandle);
}

}  // namespace
}  // namespace hal::vmx
