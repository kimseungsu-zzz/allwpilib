// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <thread>
#include <vector>

#include "../../../../main/native/vmx/InterruptInternal.h"

namespace hal::vmx {
namespace {

struct FakeInterruptHardware {
  int failFactoriesRemaining = 0;
  bool failTimestamp = false;
  bool enabled = false;
  int factories = 0;
  int destroys = 0;
  VMXInterruptEdge lastEdge = VMXInterruptEdge::kRising;
  InterruptCallbackState* state = nullptr;
  uint64_t risingTimestamp = 0;
  uint64_t fallingTimestamp = 0;
};

class FakeInterruptBackend final : public InterruptBackend {
 public:
  explicit FakeInterruptBackend(std::shared_ptr<FakeInterruptHardware> hardware)
      : m_hardware{std::move(hardware)} {}

  ~FakeInterruptBackend() override {
    ++m_hardware->destroys;
    // A callback racing with teardown must only see the still-live callback
    // state, which is closed before the backend is destroyed.
    if (m_hardware->state) {
      m_hardware->state->OnHardwareEvent(true, 999);
    }
  }

  bool SetEnabled(bool enabled) noexcept override {
    m_hardware->enabled = enabled;
    return true;
  }

  bool GetEnabled(bool& enabled) noexcept override {
    enabled = m_hardware->enabled;
    return true;
  }

  bool ReadTimestamp(bool rising, uint64_t& timestampUs) noexcept override {
    if (m_hardware->failTimestamp) {
      timestampUs = 0;
      return false;
    }
    timestampUs = rising ? m_hardware->risingTimestamp
                         : m_hardware->fallingTimestamp;
    return true;
  }

 private:
  std::shared_ptr<FakeInterruptHardware> m_hardware;
};

struct InterruptFixture {
  InterruptFixture()
      : port{
            0,
            [this](int32_t, VMXInterruptEdge edge,
                   InterruptCallbackState* state) {
              ++hardware->factories;
              hardware->lastEdge = edge;
              hardware->state = state;
              if (hardware->failFactoriesRemaining > 0) {
                --hardware->failFactoriesRemaining;
                return std::unique_ptr<InterruptBackend>{};
              }
              return std::unique_ptr<InterruptBackend>{
                  std::make_unique<FakeInterruptBackend>(hardware)};
            }} {}

  void ConfigureRising() {
    ASSERT_EQ(port.RequestSource(1, 4), InterruptResult::kOk);
    ASSERT_EQ(port.SetEdges(true, false), InterruptResult::kOk);
  }

  std::shared_ptr<FakeInterruptHardware> hardware =
      std::make_shared<FakeInterruptHardware>();
  InterruptPort port;
};

TEST(VMXInterruptTest, HandleStateWaitsForSourceAndDoubleCloseIsSafe) {
  InterruptFixture fixture;
  EXPECT_EQ(fixture.port.Wait(0.0, false, ~uint64_t{0}).result,
            InterruptResult::kUnconfigured);
  fixture.port.Close();
  fixture.port.Close();
  EXPECT_EQ(fixture.port.Wait(0.0, false, ~uint64_t{0}).result,
            InterruptResult::kInvalidHandle);
}

TEST(VMXInterruptTest, RisingFallingAndBothEdgesMapToSdkConfigAndMasks) {
  InterruptFixture fixture;
  ASSERT_EQ(fixture.port.RequestSource(1, 4), InterruptResult::kOk);
  EXPECT_EQ(fixture.hardware->factories, 1);
  EXPECT_EQ(fixture.port.SetEdges(true, false), InterruptResult::kOk);
  EXPECT_EQ(fixture.hardware->lastEdge, VMXInterruptEdge::kRising);
  fixture.hardware->state->OnHardwareEvent(true, 100);
  EXPECT_EQ(fixture.port.Wait(0.0, false, ~uint64_t{0}).mask, 1);

  EXPECT_EQ(fixture.port.SetEdges(false, true), InterruptResult::kOk);
  EXPECT_EQ(fixture.hardware->lastEdge, VMXInterruptEdge::kFalling);
  fixture.hardware->state->OnHardwareEvent(false, 200);
  EXPECT_EQ(fixture.port.Wait(0.0, false, ~uint64_t{0}).mask, 1u << 8);

  EXPECT_EQ(fixture.port.SetEdges(true, true), InterruptResult::kOk);
  EXPECT_EQ(fixture.hardware->lastEdge, VMXInterruptEdge::kBoth);
  fixture.hardware->state->OnHardwareEvent(true, 300);
  fixture.hardware->state->OnHardwareEvent(false, 400);
  EXPECT_EQ(fixture.port.Wait(0.0, false, ~uint64_t{0}).mask,
            (1u << 0) | (1u << 8));
}

TEST(VMXInterruptTest, BothEdgesFalseIsRejectedWithoutChangingConfiguration) {
  InterruptFixture fixture;
  fixture.ConfigureRising();
  EXPECT_EQ(fixture.port.SetEdges(false, false), InterruptResult::kOutOfRange);
  fixture.hardware->state->OnHardwareEvent(true, 500);
  EXPECT_EQ(fixture.port.Wait(0.0, false, ~uint64_t{0}).mask, 1);
}

TEST(VMXInterruptTest, CallbackWakesWaiterAndTimeoutIsSuccessfulZero) {
  InterruptFixture fixture;
  fixture.ConfigureRising();
  auto waiter = std::async(std::launch::async, [&] {
    return fixture.port.Wait(1.0, false, ~uint64_t{0});
  });
  std::this_thread::sleep_for(std::chrono::milliseconds{5});
  fixture.hardware->state->OnHardwareEvent(true, 600);
  EXPECT_EQ(waiter.get().mask, 1);
  EXPECT_EQ(fixture.port.Wait(0.01, false, ~uint64_t{0}).mask, 0);
}

TEST(VMXInterruptTest, IgnorePreviousUsesSequencesAndDoesNotLoseRapidEdges) {
  InterruptFixture fixture;
  fixture.ConfigureRising();
  fixture.hardware->state->OnHardwareEvent(true, 700);
  EXPECT_EQ(fixture.port.Wait(0.0, true, ~uint64_t{0}).mask, 0);

  fixture.hardware->state->OnHardwareEvent(true, 701);
  fixture.hardware->state->OnHardwareEvent(true, 702);
  EXPECT_EQ(fixture.port.Wait(0.0, false, ~uint64_t{0}).mask, 1);
  EXPECT_EQ(fixture.port.Wait(0.0, false, ~uint64_t{0}).mask, 0);
}

TEST(VMXInterruptTest, MultipleMaskOnlyReturnsThisHandleBits) {
  InterruptFixture fixture;
  fixture.ConfigureRising();
  fixture.hardware->state->OnHardwareEvent(true, 800);
  EXPECT_EQ(fixture.port.Wait(0.0, false, uint64_t{1} << 4).mask, 0);
  EXPECT_EQ(fixture.port.Wait(0.0, false, uint64_t{1}).mask, 1);
}

TEST(VMXInterruptTest, ReleaseWakesWaitAndNextWaitCanBeReused) {
  InterruptFixture fixture;
  fixture.ConfigureRising();
  auto waiter = std::async(std::launch::async, [&] {
    return fixture.port.Wait(1.0, false, ~uint64_t{0});
  });
  std::this_thread::sleep_for(std::chrono::milliseconds{5});
  EXPECT_EQ(fixture.port.ReleaseWaiting(), InterruptResult::kOk);
  EXPECT_EQ(waiter.get().mask, 0);

  auto nextWaiter = std::async(std::launch::async, [&] {
    return fixture.port.Wait(1.0, false, ~uint64_t{0});
  });
  std::this_thread::sleep_for(std::chrono::milliseconds{5});
  fixture.hardware->state->OnHardwareEvent(true, 900);
  EXPECT_EQ(nextWaiter.get().mask, 1);
}

TEST(VMXInterruptTest, CleanWakesWaitAndTeardownCallbackIsHarmless) {
  InterruptFixture fixture;
  fixture.ConfigureRising();
  auto waiter = std::async(std::launch::async, [&] {
    return fixture.port.Wait(1.0, false, ~uint64_t{0});
  });
  std::this_thread::sleep_for(std::chrono::milliseconds{5});
  fixture.port.Close();
  EXPECT_EQ(waiter.get().result, InterruptResult::kInvalidHandle);
  EXPECT_EQ(fixture.hardware->destroys, 1);
}

TEST(VMXInterruptTest, HardwareTimestampsAndFailuresAreReported) {
  InterruptFixture fixture;
  fixture.ConfigureRising();
  fixture.hardware->risingTimestamp = 123456;
  fixture.hardware->fallingTimestamp = 234567;
  EXPECT_EQ(fixture.port.ReadTimestamp(true).second, 123456u);
  EXPECT_EQ(fixture.port.ReadTimestamp(false).second, 234567u);
  fixture.hardware->failTimestamp = true;
  EXPECT_EQ(fixture.port.ReadTimestamp(true).first,
            InterruptResult::kHardwareFailure);
}

TEST(VMXInterruptTest, ActivationFailureRollsBackAndEdgeReconfigurationIsTransactional) {
  InterruptFixture fixture;
  fixture.ConfigureRising();
  fixture.hardware->failFactoriesRemaining = 1;
  EXPECT_EQ(fixture.port.SetEdges(false, true),
            InterruptResult::kHardwareFailure);
  fixture.hardware->state->OnHardwareEvent(true, 1000);
  EXPECT_EQ(fixture.port.Wait(0.0, false, ~uint64_t{0}).mask, 1);
}

TEST(VMXInterruptTest, IndexEightBitLayoutIsPreserved) {
  InterruptPort port{
      3,
      [](int32_t, VMXInterruptEdge, InterruptCallbackState*) {
        return std::unique_ptr<InterruptBackend>{
            std::make_unique<FakeInterruptBackend>(
                std::make_shared<FakeInterruptHardware>())};
      }};
  ASSERT_EQ(port.RequestSource(1, 4), InterruptResult::kOk);
  ASSERT_EQ(port.SetEdges(true, true), InterruptResult::kOk);
  auto* state = port.GetCallbackStateForTesting();
  state->OnHardwareEvent(true, 1);
  state->OnHardwareEvent(false, 2);
  EXPECT_EQ(static_cast<uint64_t>(port.Wait(0.0, false, ~uint64_t{0}).mask),
            (uint64_t{1} << 3) | (uint64_t{1} << 11));
}

}  // namespace
}  // namespace hal::vmx
