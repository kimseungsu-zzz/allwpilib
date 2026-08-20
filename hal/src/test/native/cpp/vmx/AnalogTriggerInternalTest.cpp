// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "../../../../main/native/vmx/AnalogTriggerInternal.h"

namespace hal::vmx {
namespace {

struct FakeAnalogState {
  std::mutex mutex;
  uint32_t raw = 0;
  uint32_t averageRaw = 0;
  double voltage = 0.0;
  double averageVoltage = 0.0;
  double fullScale = 5.0;
};

class FakeAnalogBackend final : public AnalogInputBackend {
 public:
  explicit FakeAnalogBackend(std::shared_ptr<FakeAnalogState> state)
      : m_state{std::move(state)} {}

  bool GetValue(uint32_t& value) noexcept override {
    std::scoped_lock lock{m_state->mutex};
    value = m_state->raw;
    return true;
  }

  bool GetAverageValue(uint32_t& value) noexcept override {
    std::scoped_lock lock{m_state->mutex};
    value = m_state->averageRaw;
    return true;
  }

  bool GetVoltage(double& voltage) noexcept override {
    std::scoped_lock lock{m_state->mutex};
    voltage = m_state->voltage;
    return true;
  }

  bool GetAverageVoltage(double& voltage) noexcept override {
    std::scoped_lock lock{m_state->mutex};
    voltage = m_state->averageVoltage;
    return true;
  }

  bool ResetAccumulator() noexcept override { return true; }

  bool GetAccumulatorOutput(int64_t& value,
                            uint32_t& count) noexcept override {
    value = 0;
    count = 0;
    return true;
  }

  double GetFullScaleVoltage() const noexcept override {
    return m_state->fullScale;
  }

 private:
  std::shared_ptr<FakeAnalogState> m_state;
};

struct AnalogTriggerFixture {
  static std::shared_ptr<AnalogInputPort> CreateInput(
      const std::shared_ptr<FakeAnalogState>& state) {
    auto input = std::make_shared<AnalogInputPort>();
    input->Initialize(
        0, ToVMXAnalogChannel(0), "analog trigger test",
        [state](int32_t, const AnalogInputConfig&) {
          return std::unique_ptr<AnalogInputBackend>{
              std::make_unique<FakeAnalogBackend>(state)};
        });
    return input;
  }

  std::shared_ptr<FakeAnalogState> hardware =
      std::make_shared<FakeAnalogState>();
  std::shared_ptr<AnalogInputPort> input = CreateInput(hardware);
  AnalogTriggerPort trigger;
  AnalogTriggerManager manager;

  AnalogTriggerFixture()
      : trigger{input},
        manager{[this](HAL_AnalogInputHandle handle) {
          return handle == 42 ? input : std::shared_ptr<AnalogInputPort>{};
        }} {
  }
};

TEST(VMXAnalogTriggerTest, RawHysteresisAndInclusiveWindowMatchWPILib) {
  AnalogTriggerFixture fixture;
  ASSERT_EQ(fixture.trigger.SetLimitsRaw(1000, 3000),
            AnalogTriggerResult::kOk);

  fixture.hardware->raw = 500;
  EXPECT_FALSE(fixture.trigger.GetTriggerState().second);
  fixture.hardware->raw = 2000;
  EXPECT_FALSE(fixture.trigger.GetTriggerState().second);
  fixture.hardware->raw = 3500;
  EXPECT_TRUE(fixture.trigger.GetTriggerState().second);
  fixture.hardware->raw = 2000;
  EXPECT_TRUE(fixture.trigger.GetTriggerState().second);
  fixture.hardware->raw = 500;
  EXPECT_FALSE(fixture.trigger.GetTriggerState().second);

  fixture.hardware->raw = 1000;
  EXPECT_TRUE(fixture.trigger.GetInWindow().second);
  fixture.hardware->raw = 3000;
  EXPECT_TRUE(fixture.trigger.GetInWindow().second);
}

TEST(VMXAnalogTriggerTest, RawLimitsRejectOrderAndOutOfRange) {
  AnalogTriggerFixture fixture;
  EXPECT_EQ(fixture.trigger.SetLimitsRaw(3000, 1000),
            AnalogTriggerResult::kLimitOrder);
  EXPECT_EQ(fixture.trigger.SetLimitsRaw(-1, 1000),
            AnalogTriggerResult::kOutOfRange);
  EXPECT_EQ(fixture.trigger.SetLimitsRaw(0, kVMXADCCounts),
            AnalogTriggerResult::kOutOfRange);
  EXPECT_EQ(fixture.trigger.SetLimitsRaw(0, 4095),
            AnalogTriggerResult::kOk);
}

TEST(VMXAnalogTriggerTest, AveragedModeSelectsAveragedRawAndVoltageValues) {
  AnalogTriggerFixture fixture;
  fixture.hardware->raw = 500;
  fixture.hardware->averageRaw = 3500;
  ASSERT_EQ(fixture.trigger.SetLimitsRaw(1000, 3000),
            AnalogTriggerResult::kOk);
  EXPECT_FALSE(fixture.trigger.GetInWindow().second);
  ASSERT_EQ(fixture.trigger.SetAveraged(true), AnalogTriggerResult::kOk);
  EXPECT_FALSE(fixture.trigger.GetInWindow().second);
  fixture.hardware->averageRaw = 2000;
  EXPECT_TRUE(fixture.trigger.GetInWindow().second);

  fixture.hardware->voltage = 0.5;
  fixture.hardware->averageVoltage = 2.5;
  ASSERT_EQ(fixture.trigger.SetLimitsVoltage(1.0, 4.0),
            AnalogTriggerResult::kOk);
  EXPECT_TRUE(fixture.trigger.GetInWindow().second);
  ASSERT_EQ(fixture.trigger.SetAveraged(false), AnalogTriggerResult::kOk);
  EXPECT_FALSE(fixture.trigger.GetInWindow().second);
}

TEST(VMXAnalogTriggerTest, VoltageLimitsUseFullScaleAndDoNotQuantize) {
  AnalogTriggerFixture fixture;
  EXPECT_EQ(fixture.trigger.SetLimitsVoltage(4.0, 1.0),
            AnalogTriggerResult::kLimitOrder);
  EXPECT_EQ(fixture.trigger.SetLimitsVoltage(-0.1, 1.0),
            AnalogTriggerResult::kOutOfRange);
  EXPECT_EQ(fixture.trigger.SetLimitsVoltage(0.0, 5.1),
            AnalogTriggerResult::kOutOfRange);
  EXPECT_EQ(fixture.trigger.SetLimitsVoltage(0.0,
                                             std::numeric_limits<double>::quiet_NaN()),
            AnalogTriggerResult::kOutOfRange);
  EXPECT_EQ(fixture.trigger.SetLimitsVoltage(1.234567, 3.765432),
            AnalogTriggerResult::kOk);

  fixture.hardware->voltage = 1.234567;
  EXPECT_TRUE(fixture.trigger.GetInWindow().second);
  fixture.hardware->voltage = 3.765433;
  EXPECT_FALSE(fixture.trigger.GetInWindow().second);
}

TEST(VMXAnalogTriggerTest, FilteredAndPulseModesAreExplicitlyUnsupported) {
  AnalogTriggerFixture fixture;
  EXPECT_EQ(fixture.trigger.SetFiltered(true), AnalogTriggerResult::kUnsupported);
  EXPECT_EQ(fixture.trigger.SetFiltered(false), AnalogTriggerResult::kOk);
  EXPECT_EQ(fixture.trigger.GetOutput(HAL_Trigger_kRisingPulse).first,
            AnalogTriggerResult::kPulseOutput);
  EXPECT_EQ(fixture.trigger.GetOutput(HAL_Trigger_kFallingPulse).first,
            AnalogTriggerResult::kPulseOutput);
}

TEST(VMXAnalogTriggerTest, InputReferenceAndCleanAreLifetimeSafe) {
  AnalogTriggerFixture fixture;
  fixture.input->Close();
  EXPECT_EQ(fixture.trigger.GetInWindow().first,
            AnalogTriggerResult::kHardwareFailure);
  fixture.trigger.Close();
  fixture.trigger.Close();
  EXPECT_EQ(fixture.trigger.SetAveraged(true),
            AnalogTriggerResult::kInvalidHandle);
}

TEST(VMXAnalogTriggerTest, ManagerAllocatesStableLogicalIndices) {
  AnalogTriggerFixture fixture;
  auto first = fixture.manager.Allocate(42);
  auto second = fixture.manager.Allocate(42);
  ASSERT_EQ(first.result, AnalogTriggerResult::kOk);
  ASSERT_EQ(second.result, AnalogTriggerResult::kOk);
  EXPECT_EQ(fixture.manager.GetIndex(first.handle), 0);
  EXPECT_EQ(fixture.manager.GetIndex(second.handle), 1);
  EXPECT_EQ(fixture.manager.GetIndex(HAL_kInvalidHandle), -1);

  fixture.manager.Free(first.handle);
  EXPECT_FALSE(fixture.manager.IsValid(first.handle));
  EXPECT_EQ(fixture.manager.GetIndex(second.handle), 1);
}

TEST(VMXAnalogTriggerTest, ConcurrentReadsConfigurationAndCleanAreSafe) {
  AnalogTriggerFixture fixture;
  ASSERT_EQ(fixture.trigger.SetLimitsRaw(1000, 3000),
            AnalogTriggerResult::kOk);
  std::vector<std::thread> threads;
  for (int i = 0; i < 4; ++i) {
    threads.emplace_back([&fixture] {
      for (int j = 0; j < 100; ++j) {
        fixture.trigger.GetInWindow();
        fixture.trigger.GetTriggerState();
      }
    });
  }
  threads.emplace_back([&fixture] {
    for (int i = 0; i < 100; ++i) {
      fixture.trigger.SetLimitsRaw(i % 1000, 3000);
      fixture.trigger.SetAveraged((i & 1) != 0);
    }
  });
  threads.emplace_back([&fixture] { fixture.trigger.Close(); });
  for (auto& thread : threads) {
    thread.join();
  }
  EXPECT_EQ(fixture.trigger.GetTriggerState().first,
            AnalogTriggerResult::kInvalidHandle);
}

}  // namespace
}  // namespace hal::vmx
