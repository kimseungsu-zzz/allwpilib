// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "../../../../main/native/vmx/CounterInternal.h"

namespace hal::vmx {
namespace {

struct FakeCounterHardware {
  uint32_t up = 0;
  uint32_t down = 0;
  bool failReads = false;
  bool failReset = false;
  int resets = 0;
  int factories = 0;
  bool lastUpRising = true;
  bool lastUpFalling = false;
  bool lastDownRising = true;
  bool lastDownFalling = false;
};

class FakeCounterBackend final : public CounterBackend {
 public:
  explicit FakeCounterBackend(std::shared_ptr<FakeCounterHardware> hardware)
      : m_hardware{std::move(hardware)} {}

  bool GetChannelCounts(uint32_t& up,
                        uint32_t& down) noexcept override {
    up = m_hardware->up;
    down = m_hardware->down;
    return !m_hardware->failReads;
  }

  bool GetInputStatus(bool& forward, bool& active) noexcept override {
    forward = m_hardware->up >= m_hardware->down;
    active = m_hardware->up != 0 || m_hardware->down != 0;
    return !m_hardware->failReads;
  }

  bool Reset() noexcept override {
    if (m_hardware->failReset) {
      return false;
    }
    m_hardware->up = 0;
    m_hardware->down = 0;
    ++m_hardware->resets;
    return true;
  }

 private:
  std::shared_ptr<FakeCounterHardware> m_hardware;
};

struct CounterFixture {
  CounterFixture()
      : manager{
            [this](int32_t, int32_t, bool upRising, bool upFalling,
                   bool downRising, bool downFalling) {
              ++hardware->factories;
              hardware->lastUpRising = upRising;
              hardware->lastUpFalling = upFalling;
              hardware->lastDownRising = downRising;
              hardware->lastDownFalling = downFalling;
              if (failFactory) {
                return std::unique_ptr<CounterBackend>{};
              }
              return std::unique_ptr<CounterBackend>{
                  std::make_unique<FakeCounterBackend>(hardware)};
            },
            [this](HAL_Handle up, HAL_Handle down) {
              ++claims;
              return claimResult == CounterResult::kOk
                         ? CounterSourceClaim{CounterResult::kOk, up + 10,
                                              down + 10}
                         : CounterSourceClaim{claimResult, -1, -1};
            },
            [this](HAL_Handle, HAL_Handle, int32_t, int32_t) { ++releases; },
            [this] { return now; }} {}

  CounterAllocationResult Allocate() {
    return manager.Allocate(HAL_Counter_kTwoPulse, nullptr);
  }

  void ConfigureSources(HAL_CounterHandle handle) {
    ASSERT_EQ(manager.SetUpSource(handle, 1, 11), CounterResult::kOk);
    ASSERT_EQ(manager.SetDownSource(handle, 2, 12), CounterResult::kOk);
  }

  std::shared_ptr<FakeCounterHardware> hardware =
      std::make_shared<FakeCounterHardware>();
  std::chrono::steady_clock::time_point now{};
  CounterResult claimResult = CounterResult::kOk;
  bool failFactory = false;
  int claims = 0;
  int releases = 0;
  CounterManager manager;
};

TEST(VMXCounterTest, InitializesWithoutResourceUntilBothSourcesExist) {
  CounterFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, CounterResult::kOk);
  EXPECT_EQ(fixture.manager.SetUpSource(allocation.handle, 1, 11),
            CounterResult::kOk);
  EXPECT_EQ(fixture.hardware->factories, 0);
  EXPECT_EQ(fixture.manager.Get(allocation.handle).first,
            CounterResult::kUnconfigured);
  EXPECT_EQ(fixture.manager.SetDownSource(allocation.handle, 2, 12),
            CounterResult::kOk);
  EXPECT_EQ(fixture.hardware->factories, 1);
}

TEST(VMXCounterTest, TwoPulseUsesUpMinusDownAndPreservesDirection) {
  CounterFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, CounterResult::kOk);
  fixture.ConfigureSources(allocation.handle);

  EXPECT_EQ(fixture.manager.Get(allocation.handle).second, 0);
  fixture.hardware->up = 5;
  fixture.hardware->down = 2;
  EXPECT_EQ(fixture.manager.Get(allocation.handle).second, 3);
  EXPECT_TRUE(fixture.manager.GetDirection(allocation.handle).second);
  fixture.hardware->down = 6;
  EXPECT_EQ(fixture.manager.Get(allocation.handle).second, -1);
  EXPECT_FALSE(fixture.manager.GetDirection(allocation.handle).second);
}

TEST(VMXCounterTest, ReverseDirectionFlipsCountAndDirection) {
  CounterFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, CounterResult::kOk);
  fixture.ConfigureSources(allocation.handle);
  fixture.manager.Get(allocation.handle);
  fixture.hardware->up = 4;
  ASSERT_EQ(fixture.manager.SetReverseDirection(allocation.handle, true),
            CounterResult::kOk);
  EXPECT_EQ(fixture.manager.Get(allocation.handle).second, -4);
  EXPECT_FALSE(fixture.manager.GetDirection(allocation.handle).second);
}

TEST(VMXCounterTest, PeriodUsesObservedEventTimeNotHalCallInterval) {
  CounterFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, CounterResult::kOk);
  fixture.ConfigureSources(allocation.handle);
  EXPECT_TRUE(std::isinf(fixture.manager.GetPeriod(allocation.handle).second));

  fixture.hardware->up = 1;
  fixture.now += std::chrono::milliseconds{100};
  fixture.manager.Get(allocation.handle);
  fixture.hardware->up = 2;
  fixture.now += std::chrono::milliseconds{250};
  EXPECT_DOUBLE_EQ(fixture.manager.GetPeriod(allocation.handle).second, 0.25);
}

TEST(VMXCounterTest, StoppedUsesMaxPeriodAndInitialNoEventIsStopped) {
  CounterFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, CounterResult::kOk);
  fixture.ConfigureSources(allocation.handle);
  ASSERT_EQ(fixture.manager.SetMaxPeriod(allocation.handle, 0.1),
            CounterResult::kOk);
  EXPECT_TRUE(fixture.manager.GetStopped(allocation.handle).second);
  fixture.hardware->up = 1;
  fixture.manager.Get(allocation.handle);
  EXPECT_FALSE(fixture.manager.GetStopped(allocation.handle).second);
  fixture.now += std::chrono::milliseconds{101};
  EXPECT_TRUE(fixture.manager.GetStopped(allocation.handle).second);
}

TEST(VMXCounterTest, EdgeChangesRecreateResourceWithoutResettingCount) {
  CounterFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, CounterResult::kOk);
  fixture.ConfigureSources(allocation.handle);
  fixture.manager.Get(allocation.handle);
  fixture.hardware->up = 7;
  ASSERT_EQ(fixture.manager.SetUpSourceEdge(allocation.handle, false, true),
            CounterResult::kOk);
  EXPECT_EQ(fixture.hardware->factories, 2);
  EXPECT_FALSE(fixture.hardware->lastUpRising);
  EXPECT_TRUE(fixture.hardware->lastUpFalling);
  EXPECT_EQ(fixture.manager.Get(allocation.handle).second, 7);
}

TEST(VMXCounterTest, ResetClearsHardwareAndSoftwareStateOnlyAfterSuccess) {
  CounterFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, CounterResult::kOk);
  fixture.ConfigureSources(allocation.handle);
  fixture.manager.Get(allocation.handle);
  fixture.hardware->up = 3;
  ASSERT_EQ(fixture.manager.Get(allocation.handle).second, 3);
  fixture.hardware->failReset = true;
  EXPECT_EQ(fixture.manager.Reset(allocation.handle),
            CounterResult::kHardwareFailure);
  EXPECT_EQ(fixture.manager.Get(allocation.handle).second, 3);
  fixture.hardware->failReset = false;
  EXPECT_EQ(fixture.manager.Reset(allocation.handle), CounterResult::kOk);
  EXPECT_EQ(fixture.manager.Get(allocation.handle).second, 0);
}

TEST(VMXCounterTest, FailedAllocationRollsBackSources) {
  CounterFixture fixture;
  fixture.failFactory = true;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, CounterResult::kOk);
  EXPECT_EQ(fixture.manager.SetUpSource(allocation.handle, 1, 11),
            CounterResult::kOk);
  EXPECT_EQ(fixture.manager.SetDownSource(allocation.handle, 2, 12),
            CounterResult::kHardwareFailure);
  EXPECT_EQ(fixture.releases, 1);
  EXPECT_EQ(fixture.manager.Get(allocation.handle).first,
            CounterResult::kUnconfigured);
}

TEST(VMXCounterTest, ClearingSourceReleasesBothChannelsAndRetainsCount) {
  CounterFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, CounterResult::kOk);
  fixture.ConfigureSources(allocation.handle);
  fixture.manager.Get(allocation.handle);
  fixture.hardware->up = 4;
  ASSERT_EQ(fixture.manager.ClearDownSource(allocation.handle),
            CounterResult::kOk);
  EXPECT_EQ(fixture.releases, 1);
  EXPECT_EQ(fixture.manager.Get(allocation.handle).first,
            CounterResult::kUnconfigured);
  fixture.manager.SetDownSource(allocation.handle, 2, 12);
  EXPECT_EQ(fixture.manager.Get(allocation.handle).second, 4);
}

TEST(VMXCounterTest, UnsupportedModesAndAveragingAreExplicit) {
  CounterFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, CounterResult::kOk);
  EXPECT_EQ(fixture.manager.SetUnsupportedMode(allocation.handle),
            CounterResult::kUnsupportedMode);
  EXPECT_EQ(fixture.manager.SetSamplesToAverage(allocation.handle, 0),
            CounterResult::kOutOfRange);
  EXPECT_EQ(fixture.manager.SetSamplesToAverage(allocation.handle, 4),
            CounterResult::kUnsupported);
  EXPECT_EQ(fixture.manager.SetAverageSize(allocation.handle, 4),
            CounterResult::kUnsupported);
  EXPECT_EQ(fixture.manager.SetUpdateWhenEmpty(allocation.handle, true),
            CounterResult::kUnsupported);
}

TEST(VMXCounterTest, InvalidHandlesAreRejected) {
  CounterFixture fixture;
  EXPECT_EQ(fixture.manager.Get(HAL_kInvalidHandle).first,
            CounterResult::kInvalidHandle);
  EXPECT_EQ(fixture.manager.Reset(HAL_kInvalidHandle),
            CounterResult::kInvalidHandle);
}

}  // namespace
}  // namespace hal::vmx
