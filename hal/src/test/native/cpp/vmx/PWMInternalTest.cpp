// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "../../../../main/native/vmx/DIOInternal.h"
#include "../../../../main/native/vmx/PWMInternal.h"

namespace hal::vmx {
namespace {

struct FakePWMHardware {
  std::mutex mutex;
  std::atomic_int creates{0};
  std::atomic_int destroys{0};
  std::atomic_int disables{0};
  std::atomic_int failedCreatesRemaining{0};
  std::atomic_bool failWrite{false};
  std::atomic_bool failDisable{false};
  std::atomic_bool failRead{false};
  std::atomic_int pulseMicroseconds{0};
  std::atomic_bool quantizeToFourMicroseconds{false};
  std::shared_ptr<int> runtimeContext = std::make_shared<int>(42);
  std::vector<const int*> observedContexts;
};

class FakePWMBackend final : public PWMBackend {
 public:
  explicit FakePWMBackend(std::shared_ptr<FakePWMHardware> hardware)
      : m_hardware{std::move(hardware)} {
    std::scoped_lock lock{m_hardware->mutex};
    m_hardware->observedContexts.push_back(m_hardware->runtimeContext.get());
  }

  ~FakePWMBackend() override { ++m_hardware->destroys; }

  bool SetPulseTimeMicroseconds(int32_t requested,
                                int32_t& applied) noexcept override {
    if (m_hardware->failWrite) {
      return false;
    }
    applied = m_hardware->quantizeToFourMicroseconds ? ((requested + 2) / 4) * 4
                                                     : requested;
    m_hardware->pulseMicroseconds = applied;
    return true;
  }

  bool GetPulseTimeMicroseconds(int32_t& pulse) noexcept override {
    if (m_hardware->failRead) {
      pulse = 0;
      return false;
    }
    pulse = m_hardware->pulseMicroseconds;
    return true;
  }

  bool Disable() noexcept override {
    ++m_hardware->disables;
    if (m_hardware->failDisable) {
      return false;
    }
    m_hardware->pulseMicroseconds = 0;
    return true;
  }

 private:
  std::shared_ptr<FakePWMHardware> m_hardware;
};

struct PWMFixture {
  PWMFixture()
      : manager{
            [this](int32_t channel) {
              static_cast<void>(channel);
              ++hardware->creates;
              int remaining = hardware->failedCreatesRemaining.load();
              while (remaining > 0 &&
                     !hardware->failedCreatesRemaining.compare_exchange_weak(
                         remaining, remaining - 1)) {
              }
              if (remaining > 0) {
                return std::unique_ptr<PWMBackend>{};
              }
              return std::unique_ptr<PWMBackend>{
                  std::make_unique<FakePWMBackend>(hardware)};
            },
            registry} {}

  std::shared_ptr<FakePWMHardware> hardware =
      std::make_shared<FakePWMHardware>();
  DigitalChannelRegistry registry;
  PWMManager manager;
};

class MinimalDIOBackend final : public DIOBackend {
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

TEST(VMXPWMTest, ChannelRangeIsZeroThroughTwentyOne) {
  EXPECT_FALSE(IsPWMChannelValid(-1));
  EXPECT_TRUE(IsPWMChannelValid(0));
  EXPECT_TRUE(IsPWMChannelValid(21));
  EXPECT_TRUE(IsPWMChannelValid(22));
  EXPECT_TRUE(IsPWMChannelValid(27));
  EXPECT_FALSE(IsPWMChannelValid(28));
}

TEST(VMXPWMTest, RejectsDuplicateAllocationAndRollsBackDriverFailure) {
  PWMFixture fixture;
  auto first = fixture.manager.Allocate(1, "first PWM");
  ASSERT_EQ(first.result, PWMResult::kOk);
  auto duplicate = fixture.manager.Allocate(1, "duplicate PWM");
  EXPECT_EQ(duplicate.result, PWMResult::kAlreadyAllocated);
  EXPECT_EQ(duplicate.previousAllocation, "first PWM");

  fixture.hardware->failedCreatesRemaining = 1;
  EXPECT_EQ(fixture.manager.Allocate(2, "failed").result,
            PWMResult::kHardwareFailure);
  EXPECT_EQ(fixture.manager.Allocate(2, "retry").result, PWMResult::kOk);
}

TEST(VMXPWMTest, DIOAndPWMShareTheSameChannelReservation) {
  DigitalChannelRegistry registry;
  PWMManager pwm{
      [](int32_t) {
        return std::unique_ptr<PWMBackend>{std::make_unique<FakePWMBackend>(
            std::make_shared<FakePWMHardware>())};
      },
      registry};
  DIOManager dio{[](int32_t, bool) {
                   return std::unique_ptr<DIOBackend>{
                       std::make_unique<MinimalDIOBackend>()};
                 },
                 registry};

  auto dioHandle = dio.Allocate(3, false, "DIO owner");
  ASSERT_EQ(dioHandle.result, DIOResult::kOk);
  auto blockedPWM = pwm.Allocate(3, "PWM contender");
  EXPECT_EQ(blockedPWM.result, PWMResult::kAlreadyAllocated);
  EXPECT_EQ(blockedPWM.previousAllocation, "DIO owner");
  dio.Free(dioHandle.handle);

  auto pwmHandle = pwm.Allocate(3, "PWM owner");
  ASSERT_EQ(pwmHandle.result, PWMResult::kOk);
  auto blockedDIO = dio.Allocate(3, false, "DIO contender");
  EXPECT_EQ(blockedDIO.result, DIOResult::kAlreadyAllocated);
  EXPECT_EQ(blockedDIO.previousAllocation, "PWM owner");
}

TEST(VMXPWMTest, DefaultAndCustomConfigRoundTrip) {
  PWMFixture fixture;
  auto allocation = fixture.manager.Allocate(4, "config");
  ASSERT_EQ(allocation.result, PWMResult::kOk);

  auto [defaultResult, defaultConfig] =
      fixture.manager.GetConfig(allocation.handle);
  ASSERT_EQ(defaultResult, PWMResult::kOk);
  EXPECT_EQ(defaultConfig.maxPwm, 2000);
  EXPECT_EQ(defaultConfig.deadbandMaxPwm, 1501);
  EXPECT_EQ(defaultConfig.centerPwm, 1500);
  EXPECT_EQ(defaultConfig.deadbandMinPwm, 1499);
  EXPECT_EQ(defaultConfig.minPwm, 1000);

  PWMConfig custom{2100, 1600, 1500, 1400, 900};
  ASSERT_EQ(fixture.manager.SetConfig(allocation.handle, custom),
            PWMResult::kOk);
  auto [customResult, actual] = fixture.manager.GetConfig(allocation.handle);
  EXPECT_EQ(customResult, PWMResult::kOk);
  EXPECT_EQ(actual.maxPwm, custom.maxPwm);
  EXPECT_EQ(actual.deadbandMaxPwm, custom.deadbandMaxPwm);
  EXPECT_EQ(actual.centerPwm, custom.centerPwm);
  EXPECT_EQ(actual.deadbandMinPwm, custom.deadbandMinPwm);
  EXPECT_EQ(actual.minPwm, custom.minPwm);
}

TEST(VMXPWMTest, DefaultSpeedScalingMatchesWPILib) {
  PWMFixture fixture;
  auto allocation = fixture.manager.Allocate(5, "speed");
  ASSERT_EQ(allocation.result, PWMResult::kOk);

  const std::vector<std::pair<double, int32_t>> cases{
      {-1.0, 1000}, {-0.5, 1250}, {0.0, 1500}, {0.5, 1751}, {1.0, 2000}};
  for (auto [speed, expectedPulse] : cases) {
    ASSERT_EQ(fixture.manager.SetSpeed(allocation.handle, speed),
              PWMResult::kOk);
    EXPECT_EQ(
        fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
        expectedPulse);
  }
}

TEST(VMXPWMTest, EliminateDeadbandChangesPositiveAndNegativeScaling) {
  PWMFixture fixture;
  auto allocation = fixture.manager.Allocate(6, "deadband");
  ASSERT_EQ(allocation.result, PWMResult::kOk);
  ASSERT_EQ(fixture.manager.SetConfig(allocation.handle,
                                      {2100, 1600, 1500, 1400, 900}),
            PWMResult::kOk);

  ASSERT_EQ(fixture.manager.SetEliminateDeadband(allocation.handle, false),
            PWMResult::kOk);
  ASSERT_EQ(fixture.manager.SetSpeed(allocation.handle, 0.5), PWMResult::kOk);
  EXPECT_EQ(fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
            1801);
  ASSERT_EQ(fixture.manager.SetSpeed(allocation.handle, -0.5), PWMResult::kOk);
  EXPECT_EQ(fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
            1200);

  ASSERT_EQ(fixture.manager.SetEliminateDeadband(allocation.handle, true),
            PWMResult::kOk);
  EXPECT_TRUE(fixture.manager.GetEliminateDeadband(allocation.handle).second);
  ASSERT_EQ(fixture.manager.SetSpeed(allocation.handle, 0.5), PWMResult::kOk);
  EXPECT_EQ(fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
            1850);
  ASSERT_EQ(fixture.manager.SetSpeed(allocation.handle, -0.5), PWMResult::kOk);
  EXPECT_EQ(fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
            1150);
}

TEST(VMXPWMTest, NonFiniteSpeedBecomesCenterPulse) {
  PWMFixture fixture;
  auto allocation = fixture.manager.Allocate(7, "non-finite");
  ASSERT_EQ(allocation.result, PWMResult::kOk);

  for (double value : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity()}) {
    ASSERT_EQ(fixture.manager.SetSpeed(allocation.handle, value),
              PWMResult::kOk);
    EXPECT_EQ(
        fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
        1500);
  }
}

TEST(VMXPWMTest, PositionScalingAndClampMatchWPILib) {
  PWMFixture fixture;
  auto allocation = fixture.manager.Allocate(8, "position");
  ASSERT_EQ(allocation.result, PWMResult::kOk);

  const std::vector<std::pair<double, int32_t>> cases{
      {-1.0, 1000}, {0.0, 1000}, {0.25, 1250},
      {0.5, 1500},  {1.0, 2000}, {2.0, 2000}};
  for (auto [position, expectedPulse] : cases) {
    ASSERT_EQ(fixture.manager.SetPosition(allocation.handle, position),
              PWMResult::kOk);
    EXPECT_EQ(
        fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
        expectedPulse);
  }
}

TEST(VMXPWMTest, RawPulseValidationAndInvalidHandles) {
  PWMFixture fixture;
  auto allocation = fixture.manager.Allocate(9, "raw");
  ASSERT_EQ(allocation.result, PWMResult::kOk);
  EXPECT_EQ(fixture.manager.SetPulseTimeMicroseconds(allocation.handle, -1),
            PWMResult::kOutOfRange);
  EXPECT_EQ(fixture.manager.SetPulseTimeMicroseconds(allocation.handle, 4096),
            PWMResult::kOutOfRange);
  EXPECT_EQ(fixture.manager.SetSpeed(HAL_kInvalidHandle, 0.0),
            PWMResult::kInvalidHandle);
  EXPECT_EQ(fixture.manager.GetPosition(HAL_kInvalidHandle).first,
            PWMResult::kInvalidHandle);
}

TEST(VMXPWMTest, DisabledIsDistinctFromZeroSpeedAndSetReenables) {
  PWMFixture fixture;
  auto allocation = fixture.manager.Allocate(10, "disable");
  ASSERT_EQ(allocation.result, PWMResult::kOk);
  EXPECT_TRUE(fixture.manager.IsDisabled(allocation.handle).second);

  ASSERT_EQ(fixture.manager.SetSpeed(allocation.handle, 0.0), PWMResult::kOk);
  EXPECT_FALSE(fixture.manager.IsDisabled(allocation.handle).second);
  EXPECT_EQ(fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
            1500);

  ASSERT_EQ(fixture.manager.Disable(allocation.handle), PWMResult::kOk);
  EXPECT_TRUE(fixture.manager.IsDisabled(allocation.handle).second);
  EXPECT_EQ(fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
            0);

  ASSERT_EQ(fixture.manager.SetPulseTimeMicroseconds(allocation.handle, 1200),
            PWMResult::kOk);
  EXPECT_FALSE(fixture.manager.IsDisabled(allocation.handle).second);
}

TEST(VMXPWMTest, FailedWriteDoesNotChangeCachedAppliedState) {
  PWMFixture fixture;
  auto allocation = fixture.manager.Allocate(11, "write failure");
  ASSERT_EQ(allocation.result, PWMResult::kOk);
  ASSERT_EQ(fixture.manager.SetPulseTimeMicroseconds(allocation.handle, 1500),
            PWMResult::kOk);

  fixture.hardware->failWrite = true;
  EXPECT_EQ(fixture.manager.SetPulseTimeMicroseconds(allocation.handle, 1800),
            PWMResult::kHardwareFailure);
  EXPECT_EQ(fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
            1500);
}

TEST(VMXPWMTest, GetterStoresAppliedQuantizedPulseNotRequestedPulse) {
  PWMFixture fixture;
  fixture.hardware->quantizeToFourMicroseconds = true;
  auto allocation = fixture.manager.Allocate(15, "quantized state");
  ASSERT_EQ(allocation.result, PWMResult::kOk);

  ASSERT_EQ(fixture.manager.SetPulseTimeMicroseconds(allocation.handle, 1501),
            PWMResult::kOk);
  EXPECT_EQ(fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
            1500);
  ASSERT_EQ(fixture.manager.SetPulseTimeMicroseconds(allocation.handle, 1502),
            PWMResult::kOk);
  EXPECT_EQ(fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
            1504);
}

TEST(VMXPWMTest, FailedDisablePreservesLastEnabledState) {
  PWMFixture fixture;
  auto allocation = fixture.manager.Allocate(16, "disable failure");
  ASSERT_EQ(allocation.result, PWMResult::kOk);
  ASSERT_EQ(fixture.manager.SetSpeed(allocation.handle, 0.0), PWMResult::kOk);

  fixture.hardware->failDisable = true;
  EXPECT_EQ(fixture.manager.Disable(allocation.handle),
            PWMResult::kHardwareFailure);
  EXPECT_FALSE(fixture.manager.IsDisabled(allocation.handle).second);
  EXPECT_EQ(fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
            1500);
}

TEST(VMXPWMTest, FreeDoubleFreeAndConcurrentAccessAreSafe) {
  PWMFixture fixture;
  auto allocation = fixture.manager.Allocate(12, "concurrent");
  ASSERT_EQ(allocation.result, PWMResult::kOk);

  std::vector<std::thread> threads;
  for (int i = 0; i < 4; ++i) {
    threads.emplace_back([&, i] {
      for (int j = 0; j < 100; ++j) {
        fixture.manager.SetSpeed(allocation.handle, ((i + j) % 21 - 10) / 10.0);
        fixture.manager.GetSpeed(allocation.handle);
        fixture.manager.GetPosition(allocation.handle);
      }
    });
  }
  threads.emplace_back([&] { fixture.manager.Free(allocation.handle); });
  for (auto& thread : threads) {
    thread.join();
  }
  fixture.manager.Free(allocation.handle);
  EXPECT_EQ(fixture.manager.GetSpeed(allocation.handle).first,
            PWMResult::kInvalidHandle);
  EXPECT_EQ(fixture.manager.Allocate(12, "replacement").result, PWMResult::kOk);
}

TEST(VMXPWMTest, EveryPWMBackendObservesOneSharedRuntimeContext) {
  PWMFixture fixture;
  ASSERT_EQ(fixture.manager.Allocate(13, "context one").result, PWMResult::kOk);
  ASSERT_EQ(fixture.manager.Allocate(14, "context two").result, PWMResult::kOk);

  std::scoped_lock lock{fixture.hardware->mutex};
  ASSERT_EQ(fixture.hardware->observedContexts.size(), 2u);
  EXPECT_EQ(fixture.hardware->observedContexts[0],
            fixture.hardware->runtimeContext.get());
  EXPECT_EQ(fixture.hardware->observedContexts[1],
            fixture.hardware->runtimeContext.get());
}

TEST(VMXPWMTest, GettersReadHardwareAndReportReadFailure) {
  PWMFixture fixture;
  auto allocation = fixture.manager.Allocate(17, "readback");
  ASSERT_EQ(allocation.result, PWMResult::kOk);
  ASSERT_EQ(fixture.manager.SetPulseTimeMicroseconds(allocation.handle, 1500),
            PWMResult::kOk);
  fixture.hardware->pulseMicroseconds = 1600;

  EXPECT_EQ(fixture.manager.GetPulseTimeMicroseconds(allocation.handle).second,
            1600);
  EXPECT_GT(fixture.manager.GetSpeed(allocation.handle).second, 0.0);
  EXPECT_DOUBLE_EQ(fixture.manager.GetPosition(allocation.handle).second, 0.6);

  fixture.hardware->failRead = true;
  EXPECT_EQ(fixture.manager.GetPulseTimeMicroseconds(allocation.handle).first,
            PWMResult::kHardwareFailure);
}

}  // namespace
}  // namespace hal::vmx
