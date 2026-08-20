// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <tuple>
#include <vector>

#include "../../../../main/native/vmx/AnalogInputInternal.h"

namespace hal::vmx {
namespace {

struct FakeAnalogHardware {
  std::mutex mutex;
  std::array<uint32_t, kNumAnalogInputs> values{};
  std::array<uint32_t, kNumAnalogInputs> averageValues{};
  std::array<double, kNumAnalogInputs> voltages{};
  std::array<double, kNumAnalogInputs> averageVoltages{};
  std::array<int64_t, kNumAnalogInputs> accumulatorValues{};
  std::array<uint32_t, kNumAnalogInputs> accumulatorCounts{};
  std::atomic_bool failValueRead{false};
  std::atomic_bool failAverageValueRead{false};
  std::atomic_bool failVoltageRead{false};
  std::atomic_bool failAverageVoltageRead{false};
  std::atomic_bool failAccumulatorRead{false};
  std::atomic_bool failAccumulatorReset{false};
  std::atomic_int failedCreatesRemaining{0};
  std::atomic_int creates{0};
  std::atomic_int destroys{0};
  std::atomic_int accumulatorReads{0};
  std::atomic_int accumulatorResets{0};
  double fullScaleVoltage = 5.0;
  std::shared_ptr<int> runtimeContext = std::make_shared<int>(73);
  std::vector<std::tuple<int32_t, int32_t, int32_t>> configurations;
  std::vector<AnalogInputConfig> detailedConfigurations;
  std::vector<const int*> observedContexts;
};

class FakeAnalogBackend final : public AnalogInputBackend {
 public:
  FakeAnalogBackend(std::shared_ptr<FakeAnalogHardware> hardware,
                    int32_t physicalChannel, const AnalogInputConfig& config)
      : m_hardware{std::move(hardware)},
        m_index{physicalChannel - kFirstVMXAnalogChannel} {
    std::scoped_lock lock{m_hardware->mutex};
    m_hardware->configurations.emplace_back(
        physicalChannel, config.averageBits, config.oversampleBits);
    m_hardware->detailedConfigurations.push_back(config);
    m_hardware->observedContexts.push_back(m_hardware->runtimeContext.get());
    if (config.accumulatorEnabled) {
      m_hardware->accumulatorValues[m_index] = 0;
      m_hardware->accumulatorCounts[m_index] = 0;
    }
  }

  ~FakeAnalogBackend() override { ++m_hardware->destroys; }

  bool GetValue(uint32_t& value) noexcept override {
    if (m_hardware->failValueRead) {
      value = 0;
      return false;
    }
    std::scoped_lock lock{m_hardware->mutex};
    value = m_hardware->values[m_index];
    return true;
  }

  bool GetAverageValue(uint32_t& value) noexcept override {
    if (m_hardware->failAverageValueRead) {
      value = 0;
      return false;
    }
    std::scoped_lock lock{m_hardware->mutex};
    value = m_hardware->averageValues[m_index];
    return true;
  }

  bool GetVoltage(double& voltage) noexcept override {
    if (m_hardware->failVoltageRead) {
      voltage = 0.0;
      return false;
    }
    std::scoped_lock lock{m_hardware->mutex};
    voltage = m_hardware->voltages[m_index];
    return true;
  }

  bool GetAverageVoltage(double& voltage) noexcept override {
    if (m_hardware->failAverageVoltageRead) {
      voltage = 0.0;
      return false;
    }
    std::scoped_lock lock{m_hardware->mutex};
    voltage = m_hardware->averageVoltages[m_index];
    return true;
  }

  bool ResetAccumulator() noexcept override {
    ++m_hardware->accumulatorResets;
    if (m_hardware->failAccumulatorReset) {
      return false;
    }
    std::scoped_lock lock{m_hardware->mutex};
    m_hardware->accumulatorValues[m_index] = 0;
    m_hardware->accumulatorCounts[m_index] = 0;
    return true;
  }

  bool GetAccumulatorOutput(int64_t& value,
                            uint32_t& count) noexcept override {
    ++m_hardware->accumulatorReads;
    if (m_hardware->failAccumulatorRead) {
      value = 0;
      count = 0;
      return false;
    }
    std::scoped_lock lock{m_hardware->mutex};
    value = m_hardware->accumulatorValues[m_index];
    count = m_hardware->accumulatorCounts[m_index];
    return true;
  }

  double GetFullScaleVoltage() const noexcept override {
    return m_hardware->fullScaleVoltage;
  }

 private:
  std::shared_ptr<FakeAnalogHardware> m_hardware;
  int32_t m_index;
};

struct AnalogFixture {
  AnalogFixture()
      : manager{[this](int32_t physicalChannel,
                       const AnalogInputConfig& config) {
          ++hardware->creates;
          int remaining = hardware->failedCreatesRemaining.load();
          while (remaining > 0 &&
                 !hardware->failedCreatesRemaining.compare_exchange_weak(
                     remaining, remaining - 1)) {
          }
          if (remaining > 0) {
            return std::unique_ptr<AnalogInputBackend>{};
          }
          return std::unique_ptr<AnalogInputBackend>{
              std::make_unique<FakeAnalogBackend>(hardware, physicalChannel,
                                                  config)};
        }} {}

  std::shared_ptr<FakeAnalogHardware> hardware =
      std::make_shared<FakeAnalogHardware>();
  AnalogInputManager manager;
};

TEST(VMXAnalogInputTest, LogicalRangeAndPhysicalMapping) {
  EXPECT_FALSE(IsAnalogInputChannelValid(-1));
  EXPECT_TRUE(IsAnalogInputChannelValid(0));
  EXPECT_TRUE(IsAnalogInputChannelValid(3));
  EXPECT_FALSE(IsAnalogInputChannelValid(4));
  EXPECT_FALSE(IsAnalogInputChannelValid(22));
  EXPECT_EQ(ToVMXAnalogChannel(0), 22);
  EXPECT_EQ(ToVMXAnalogChannel(1), 23);
  EXPECT_EQ(ToVMXAnalogChannel(2), 24);
  EXPECT_EQ(ToVMXAnalogChannel(3), 25);
}

TEST(VMXAnalogInputTest, AllocationUsesPhysicalMappingAndRejectsDuplicate) {
  AnalogFixture fixture;
  auto first = fixture.manager.Allocate(1, "first analog owner");
  ASSERT_EQ(first.result, AnalogInputResult::kOk);
  auto duplicate = fixture.manager.Allocate(1, "duplicate owner");
  EXPECT_EQ(duplicate.result, AnalogInputResult::kAlreadyAllocated);
  EXPECT_EQ(duplicate.previousAllocation, "first analog owner");

  std::scoped_lock lock{fixture.hardware->mutex};
  ASSERT_EQ(fixture.hardware->configurations.size(), 1u);
  EXPECT_EQ(std::get<0>(fixture.hardware->configurations[0]), 23);
}

TEST(VMXAnalogInputTest, ActivationFailureRollsBackHandle) {
  AnalogFixture fixture;
  fixture.hardware->failedCreatesRemaining = 1;
  EXPECT_EQ(fixture.manager.Allocate(2, "failed").result,
            AnalogInputResult::kHardwareFailure);
  EXPECT_EQ(fixture.manager.Allocate(2, "retry").result,
            AnalogInputResult::kOk);
}

TEST(VMXAnalogInputTest, ZeroVoltsAndReadFailureRemainDistinct) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(0, "voltage");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  fixture.hardware->voltages[0] = 0.0;

  auto [success, zeroVolts] = fixture.manager.GetVoltage(allocation.handle);
  EXPECT_EQ(success, AnalogInputResult::kOk);
  EXPECT_DOUBLE_EQ(zeroVolts, 0.0);

  fixture.hardware->failVoltageRead = true;
  auto [failure, failureValue] = fixture.manager.GetVoltage(allocation.handle);
  EXPECT_EQ(failure, AnalogInputResult::kHardwareFailure);
  EXPECT_DOUBLE_EQ(failureValue, 0.0);
}

TEST(VMXAnalogInputTest, InstantaneousAndAverageReadsAreIndependent) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(1, "reads");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  fixture.hardware->values[1] = 1024;
  fixture.hardware->averageValues[1] = 1050;
  fixture.hardware->voltages[1] = 1.25;
  fixture.hardware->averageVoltages[1] = 1.28125;

  EXPECT_EQ(fixture.manager.GetValue(allocation.handle).second, 1024);
  EXPECT_EQ(fixture.manager.GetAverageValue(allocation.handle).second, 1050);
  EXPECT_DOUBLE_EQ(fixture.manager.GetVoltage(allocation.handle).second, 1.25);
  EXPECT_DOUBLE_EQ(fixture.manager.GetAverageVoltage(allocation.handle).second,
                   1.28125);

  fixture.hardware->failAverageValueRead = true;
  fixture.hardware->failAverageVoltageRead = true;
  EXPECT_EQ(fixture.manager.GetAverageValue(allocation.handle).first,
            AnalogInputResult::kHardwareFailure);
  EXPECT_EQ(fixture.manager.GetAverageVoltage(allocation.handle).first,
            AnalogInputResult::kHardwareFailure);
}

TEST(VMXAnalogInputTest, DefaultAndChangedAverageOversampleBits) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(2, "configuration");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  EXPECT_EQ(fixture.manager.GetAverageBits(allocation.handle).second, 7);
  EXPECT_EQ(fixture.manager.GetOversampleBits(allocation.handle).second, 0);

  ASSERT_EQ(fixture.manager.SetAverageBits(allocation.handle, 4),
            AnalogInputResult::kOk);
  ASSERT_EQ(fixture.manager.SetOversampleBits(allocation.handle, 2),
            AnalogInputResult::kOk);
  EXPECT_EQ(fixture.manager.GetAverageBits(allocation.handle).second, 4);
  EXPECT_EQ(fixture.manager.GetOversampleBits(allocation.handle).second, 2);

  std::scoped_lock lock{fixture.hardware->mutex};
  ASSERT_EQ(fixture.hardware->configurations.size(), 3u);
  EXPECT_EQ(fixture.hardware->configurations.back(), std::make_tuple(24, 4, 2));
}

TEST(VMXAnalogInputTest, ReconfigurationFailureRestoresPreviousBits) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(3, "rollback");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  fixture.hardware->failedCreatesRemaining = 1;

  EXPECT_EQ(fixture.manager.SetAverageBits(allocation.handle, 3),
            AnalogInputResult::kHardwareFailure);
  EXPECT_EQ(fixture.manager.GetAverageBits(allocation.handle).second, 7);
  EXPECT_EQ(fixture.manager.GetOversampleBits(allocation.handle).second, 0);
}

TEST(VMXAnalogInputTest, ReconfigurationRollbackFailureFaultsHandle) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(3, "fault");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  fixture.hardware->failedCreatesRemaining = 2;

  EXPECT_EQ(fixture.manager.SetOversampleBits(allocation.handle, 1),
            AnalogInputResult::kRollbackFailure);
  EXPECT_EQ(fixture.manager.GetVoltage(allocation.handle).first,
            AnalogInputResult::kHardwareFailure);
}

TEST(VMXAnalogInputTest, RawConversionsUseDiscoveredTwelveBitFullScale) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(0, "conversion");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);

  EXPECT_DOUBLE_EQ(fixture.manager.ValueToVolts(allocation.handle, 0).second,
                   0.0);
  EXPECT_DOUBLE_EQ(fixture.manager.ValueToVolts(allocation.handle, 2048).second,
                   2.5);
  EXPECT_NEAR(fixture.manager.ValueToVolts(allocation.handle, 4095).second,
              5.0 * 4095.0 / 4096.0, 1e-12);
  EXPECT_EQ(fixture.manager.GetLSBWeight(allocation.handle).second, 1220703);

  auto negative = fixture.manager.VoltsToValue(allocation.handle, -1.0);
  EXPECT_TRUE(negative.clamped);
  EXPECT_EQ(negative.value, 0);
  auto middle = fixture.manager.VoltsToValue(allocation.handle, 2.5);
  EXPECT_FALSE(middle.clamped);
  EXPECT_EQ(middle.value, 2048);
  auto overRange = fixture.manager.VoltsToValue(allocation.handle, 6.0);
  EXPECT_TRUE(overRange.clamped);
  EXPECT_EQ(overRange.value, 4096);
}

TEST(VMXAnalogInputTest, FreeReleasesResourceAndDoubleFreeIsSafe) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(0, "free");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  fixture.manager.Free(allocation.handle);
  fixture.manager.Free(allocation.handle);
  EXPECT_EQ(fixture.hardware->destroys, 1);
  EXPECT_EQ(fixture.manager.GetVoltage(allocation.handle).first,
            AnalogInputResult::kInvalidHandle);
  EXPECT_EQ(fixture.manager.Allocate(0, "replacement").result,
            AnalogInputResult::kOk);
}

TEST(VMXAnalogInputTest, InvalidHandlesAndBitRangesAreRejected) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(1, "invalid");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  EXPECT_EQ(fixture.manager.GetVoltage(HAL_kInvalidHandle).first,
            AnalogInputResult::kInvalidHandle);
  EXPECT_EQ(fixture.manager.SetAverageBits(allocation.handle, -1),
            AnalogInputResult::kOutOfRange);
  EXPECT_EQ(fixture.manager.SetOversampleBits(allocation.handle, 256),
            AnalogInputResult::kOutOfRange);
}

TEST(VMXAnalogInputTest, ConcurrentReadConfigAndFreeAreLifetimeSafe) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(2, "concurrent");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);

  std::vector<std::thread> threads;
  for (int i = 0; i < 4; ++i) {
    threads.emplace_back([&] {
      for (int j = 0; j < 100; ++j) {
        fixture.manager.GetVoltage(allocation.handle);
        fixture.manager.GetAverageVoltage(allocation.handle);
        fixture.manager.GetValue(allocation.handle);
      }
    });
  }
  threads.emplace_back([&] {
    for (int i = 0; i < 20; ++i) {
      fixture.manager.SetAverageBits(allocation.handle, i % 8);
    }
  });
  threads.emplace_back([&] { fixture.manager.Free(allocation.handle); });
  for (auto& thread : threads) {
    thread.join();
  }
  EXPECT_EQ(fixture.manager.GetVoltage(allocation.handle).first,
            AnalogInputResult::kInvalidHandle);
}

TEST(VMXAnalogInputTest, EveryBackendObservesOneSharedRuntimeContext) {
  AnalogFixture fixture;
  ASSERT_EQ(fixture.manager.Allocate(0, "context zero").result,
            AnalogInputResult::kOk);
  ASSERT_EQ(fixture.manager.Allocate(1, "context one").result,
            AnalogInputResult::kOk);

  std::scoped_lock lock{fixture.hardware->mutex};
  ASSERT_EQ(fixture.hardware->observedContexts.size(), 2u);
  EXPECT_EQ(fixture.hardware->observedContexts[0],
            fixture.hardware->runtimeContext.get());
  EXPECT_EQ(fixture.hardware->observedContexts[1],
            fixture.hardware->runtimeContext.get());
}

TEST(VMXAnalogAccumulatorTest, PublicCompatibilityIsLimitedToChannelsZeroAndOne) {
  AnalogFixture fixture;
  auto zero = fixture.manager.Allocate(0, "accumulator zero");
  auto one = fixture.manager.Allocate(1, "accumulator one");
  auto two = fixture.manager.Allocate(2, "analog only");
  ASSERT_EQ(zero.result, AnalogInputResult::kOk);
  ASSERT_EQ(one.result, AnalogInputResult::kOk);
  ASSERT_EQ(two.result, AnalogInputResult::kOk);

  EXPECT_TRUE(fixture.manager.IsAccumulatorChannel(zero.handle).second);
  EXPECT_TRUE(fixture.manager.IsAccumulatorChannel(one.handle).second);
  EXPECT_FALSE(fixture.manager.IsAccumulatorChannel(two.handle).second);
  EXPECT_EQ(fixture.manager.InitAccumulator(two.handle),
            AnalogInputResult::kInvalidAccumulatorChannel);
  EXPECT_EQ(fixture.manager.IsAccumulatorChannel(HAL_kInvalidHandle).first,
            AnalogInputResult::kInvalidHandle);
}

TEST(VMXAnalogAccumulatorTest, InitEnablesCounterOnTheExistingAnalogPort) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(0, "init accumulator");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  EXPECT_EQ(fixture.manager.GetAccumulatorOutput(allocation.handle).first,
            AnalogInputResult::kAccumulatorNotInitialized);

  ASSERT_EQ(fixture.manager.InitAccumulator(allocation.handle),
            AnalogInputResult::kOk);
  ASSERT_EQ(fixture.manager.InitAccumulator(allocation.handle),
            AnalogInputResult::kOk);
  std::scoped_lock lock{fixture.hardware->mutex};
  ASSERT_EQ(fixture.hardware->detailedConfigurations.size(), 2u);
  EXPECT_FALSE(fixture.hardware->detailedConfigurations[0].accumulatorEnabled);
  EXPECT_TRUE(fixture.hardware->detailedConfigurations[1].accumulatorEnabled);
}

TEST(VMXAnalogAccumulatorTest, OutputUsesOneAtomicHardwareRead) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(1, "atomic output");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  ASSERT_EQ(fixture.manager.InitAccumulator(allocation.handle),
            AnalogInputResult::kOk);
  fixture.hardware->accumulatorValues[1] = -12345;
  fixture.hardware->accumulatorCounts[1] = 678;

  int before = fixture.hardware->accumulatorReads;
  auto [result, output] =
      fixture.manager.GetAccumulatorOutput(allocation.handle);
  EXPECT_EQ(result, AnalogInputResult::kOk);
  EXPECT_EQ(output.value, -12345);
  EXPECT_EQ(output.count, 678);
  EXPECT_EQ(fixture.hardware->accumulatorReads, before + 1);
}

TEST(VMXAnalogAccumulatorTest,
     ReconfigurationPreservesValueAndCountWithSoftwareOffsets) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(0, "continuous accumulator");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  ASSERT_EQ(fixture.manager.InitAccumulator(allocation.handle),
            AnalogInputResult::kOk);
  fixture.hardware->accumulatorValues[0] = 1000;
  fixture.hardware->accumulatorCounts[0] = 10;

  ASSERT_EQ(fixture.manager.SetAverageBits(allocation.handle, 4),
            AnalogInputResult::kOk);
  fixture.hardware->accumulatorValues[0] = 25;
  fixture.hardware->accumulatorCounts[0] = 2;
  auto afterAverage = fixture.manager.GetAccumulatorOutput(allocation.handle);
  EXPECT_EQ(afterAverage.second.value, 1025);
  EXPECT_EQ(afterAverage.second.count, 12);

  ASSERT_EQ(fixture.manager.SetAccumulatorCenter(allocation.handle, 17),
            AnalogInputResult::kOk);
  fixture.hardware->accumulatorValues[0] = -5;
  fixture.hardware->accumulatorCounts[0] = 3;
  auto afterCenter = fixture.manager.GetAccumulatorOutput(allocation.handle);
  EXPECT_EQ(afterCenter.second.value, 1020);
  EXPECT_EQ(afterCenter.second.count, 15);

  ASSERT_EQ(fixture.manager.SetAccumulatorDeadband(allocation.handle, 4),
            AnalogInputResult::kOk);
  std::scoped_lock lock{fixture.hardware->mutex};
  EXPECT_EQ(fixture.hardware->detailedConfigurations.back().center, 17);
  EXPECT_EQ(fixture.hardware->detailedConfigurations.back().deadband, 4);
}

TEST(VMXAnalogAccumulatorTest, ResetClearsHardwareAndSoftwareContinuityState) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(0, "reset accumulator");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  ASSERT_EQ(fixture.manager.InitAccumulator(allocation.handle),
            AnalogInputResult::kOk);
  fixture.hardware->accumulatorValues[0] = 400;
  fixture.hardware->accumulatorCounts[0] = 8;
  ASSERT_EQ(fixture.manager.SetOversampleBits(allocation.handle, 2),
            AnalogInputResult::kOk);
  fixture.hardware->accumulatorValues[0] = 7;
  fixture.hardware->accumulatorCounts[0] = 1;

  ASSERT_EQ(fixture.manager.ResetAccumulator(allocation.handle),
            AnalogInputResult::kOk);
  auto output = fixture.manager.GetAccumulatorOutput(allocation.handle);
  EXPECT_EQ(output.second.value, 0);
  EXPECT_EQ(output.second.count, 0);
  EXPECT_EQ(fixture.hardware->accumulatorResets, 1);
}

TEST(VMXAnalogAccumulatorTest, FailedSnapshotDoesNotDestroyLiveResource) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(0, "snapshot failure");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  ASSERT_EQ(fixture.manager.InitAccumulator(allocation.handle),
            AnalogInputResult::kOk);
  int creates = fixture.hardware->creates;
  int destroys = fixture.hardware->destroys;
  fixture.hardware->failAccumulatorRead = true;

  EXPECT_EQ(fixture.manager.SetAverageBits(allocation.handle, 3),
            AnalogInputResult::kHardwareFailure);
  EXPECT_EQ(fixture.hardware->creates, creates);
  EXPECT_EQ(fixture.hardware->destroys, destroys);
}

TEST(VMXAnalogAccumulatorTest, ConfigurationRangesMatchVMXSDKFields) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(1, "ranges");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  EXPECT_EQ(fixture.manager.SetAccumulatorCenter(allocation.handle, 1),
            AnalogInputResult::kAccumulatorNotInitialized);
  ASSERT_EQ(fixture.manager.InitAccumulator(allocation.handle),
            AnalogInputResult::kOk);
  EXPECT_EQ(fixture.manager.SetAccumulatorCenter(allocation.handle, 32768),
            AnalogInputResult::kOutOfRange);
  EXPECT_EQ(fixture.manager.SetAccumulatorCenter(allocation.handle, -32769),
            AnalogInputResult::kOutOfRange);
  EXPECT_EQ(fixture.manager.SetAccumulatorDeadband(allocation.handle, -1),
            AnalogInputResult::kOutOfRange);
  EXPECT_EQ(fixture.manager.SetAccumulatorDeadband(allocation.handle, 32768),
            AnalogInputResult::kOutOfRange);
}

TEST(VMXAnalogAccumulatorTest, FailedReplacementRollbackKeepsContinuity) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(0, "rollback continuity");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  ASSERT_EQ(fixture.manager.InitAccumulator(allocation.handle),
            AnalogInputResult::kOk);
  fixture.hardware->accumulatorValues[0] = 900;
  fixture.hardware->accumulatorCounts[0] = 9;
  fixture.hardware->failedCreatesRemaining = 1;

  EXPECT_EQ(fixture.manager.SetAccumulatorCenter(allocation.handle, 12),
            AnalogInputResult::kHardwareFailure);
  fixture.hardware->accumulatorValues[0] = 6;
  fixture.hardware->accumulatorCounts[0] = 1;
  auto output = fixture.manager.GetAccumulatorOutput(allocation.handle);
  EXPECT_EQ(output.second.value, 906);
  EXPECT_EQ(output.second.count, 10);
}

TEST(VMXAnalogAccumulatorTest, FailedResetPreservesAccumulatedOutput) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(1, "failed reset");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  ASSERT_EQ(fixture.manager.InitAccumulator(allocation.handle),
            AnalogInputResult::kOk);
  fixture.hardware->accumulatorValues[1] = 44;
  fixture.hardware->accumulatorCounts[1] = 5;
  fixture.hardware->failAccumulatorReset = true;

  EXPECT_EQ(fixture.manager.ResetAccumulator(allocation.handle),
            AnalogInputResult::kHardwareFailure);
  auto output = fixture.manager.GetAccumulatorOutput(allocation.handle);
  EXPECT_EQ(output.second.value, 44);
  EXPECT_EQ(output.second.count, 5);
}

TEST(VMXAnalogAccumulatorTest, ExtendsHardwareCountAcrossUint32Rollover) {
  AnalogFixture fixture;
  auto allocation = fixture.manager.Allocate(0, "count rollover");
  ASSERT_EQ(allocation.result, AnalogInputResult::kOk);
  ASSERT_EQ(fixture.manager.InitAccumulator(allocation.handle),
            AnalogInputResult::kOk);
  fixture.hardware->accumulatorCounts[0] = UINT32_MAX - 1;
  EXPECT_EQ(fixture.manager.GetAccumulatorOutput(allocation.handle).second.count,
            static_cast<int64_t>(UINT32_MAX) - 1);

  fixture.hardware->accumulatorCounts[0] = 2;
  EXPECT_EQ(fixture.manager.GetAccumulatorOutput(allocation.handle).second.count,
            (int64_t{1} << 32) + 2);
}

}  // namespace
}  // namespace hal::vmx
