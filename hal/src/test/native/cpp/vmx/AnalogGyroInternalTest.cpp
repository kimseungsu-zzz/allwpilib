// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "../../../../main/native/vmx/AnalogGyroInternal.h"

namespace hal::vmx {
namespace {

struct FakeGyroHardware {
  std::mutex mutex;
  int64_t accumulatorValue = 0;
  uint32_t accumulatorCount = 0;
  uint32_t averageValue = 0;
  bool failAccumulatorRead = false;
  bool failAccumulatorReset = false;
  std::vector<AnalogInputConfig> configurations;
};

class FakeGyroBackend final : public AnalogInputBackend {
 public:
  FakeGyroBackend(std::shared_ptr<FakeGyroHardware> hardware,
                  const AnalogInputConfig& config)
      : m_hardware{std::move(hardware)} {
    std::scoped_lock lock{m_hardware->mutex};
    m_hardware->configurations.push_back(config);
    if (config.accumulatorEnabled) {
      m_hardware->accumulatorValue = 0;
      m_hardware->accumulatorCount = 0;
    }
  }

  bool GetValue(uint32_t& value) noexcept override {
    value = 0;
    return true;
  }
  bool GetAverageValue(uint32_t& value) noexcept override {
    std::scoped_lock lock{m_hardware->mutex};
    value = m_hardware->averageValue;
    return true;
  }
  bool GetVoltage(double& voltage) noexcept override {
    voltage = 0.0;
    return true;
  }
  bool GetAverageVoltage(double& voltage) noexcept override {
    voltage = 0.0;
    return true;
  }
  bool ResetAccumulator() noexcept override {
    std::scoped_lock lock{m_hardware->mutex};
    if (m_hardware->failAccumulatorReset) {
      return false;
    }
    m_hardware->accumulatorValue = 0;
    m_hardware->accumulatorCount = 0;
    return true;
  }
  bool GetAccumulatorOutput(int64_t& value,
                            uint32_t& count) noexcept override {
    std::scoped_lock lock{m_hardware->mutex};
    if (m_hardware->failAccumulatorRead) {
      value = 0;
      count = 0;
      return false;
    }
    value = m_hardware->accumulatorValue;
    count = m_hardware->accumulatorCount;
    return true;
  }
  double GetFullScaleVoltage() const noexcept override { return 5.0; }

 private:
  std::shared_ptr<FakeGyroHardware> m_hardware;
};

struct GyroFixture {
  GyroFixture()
      : input{std::make_shared<AnalogInputPort>()},
        gyro{[this](double seconds) { waits.push_back(seconds); }} {
    const bool initialized = input->Initialize(
        0, 22, "gyro input", [this](int32_t, const AnalogInputConfig& config) {
          return std::unique_ptr<AnalogInputBackend>{
              std::make_unique<FakeGyroBackend>(hardware, config)};
        });
    EXPECT_TRUE(initialized);
    if (!initialized) {
      return;
    }
    EXPECT_EQ(gyro.Attach(input, "gyro"), AnalogGyroResult::kOk);
  }

  std::shared_ptr<FakeGyroHardware> hardware =
      std::make_shared<FakeGyroHardware>();
  std::shared_ptr<AnalogInputPort> input;
  std::vector<double> waits;
  AnalogGyroPort gyro;
};

TEST(VMXAnalogGyroTest, AcceptsOnlyLogicalAccumulatorChannels) {
  auto hardware = std::make_shared<FakeGyroHardware>();
  for (int32_t channel : {0, 1}) {
    auto input = std::make_shared<AnalogInputPort>();
    ASSERT_TRUE(input->Initialize(
        channel, kFirstVMXAnalogChannel + channel, "valid", [hardware](
                                                        int32_t,
                                                        const AnalogInputConfig& config) {
          return std::unique_ptr<AnalogInputBackend>{
              std::make_unique<FakeGyroBackend>(hardware, config)};
        }));
    AnalogGyroPort gyro;
    EXPECT_EQ(gyro.Attach(input, "valid"), AnalogGyroResult::kOk);
  }
  for (int32_t channel : {2, 3}) {
    auto input = std::make_shared<AnalogInputPort>();
    ASSERT_TRUE(input->Initialize(
        channel, kFirstVMXAnalogChannel + channel, "invalid", [hardware](
                                                          int32_t,
                                                          const AnalogInputConfig& config) {
          return std::unique_ptr<AnalogInputBackend>{
              std::make_unique<FakeGyroBackend>(hardware, config)};
        }));
    AnalogGyroPort gyro;
    EXPECT_EQ(gyro.Attach(input, "invalid"),
              AnalogGyroResult::kInvalidChannel);
  }
}

TEST(VMXAnalogGyroTest, ManagerRejectsDuplicateGyroAllocation) {
  AnalogGyroHandleResource resource;
  HAL_GyroHandle first = HAL_kInvalidHandle;
  int32_t status = HAL_SUCCESS;
  ASSERT_TRUE(resource.Allocate(0, &first, &status));
  EXPECT_EQ(status, HAL_SUCCESS);
  HAL_GyroHandle duplicate = HAL_kInvalidHandle;
  auto duplicateResource = resource.Allocate(0, &duplicate, &status);
  EXPECT_FALSE(duplicateResource);
  EXPECT_EQ(status, RESOURCE_IS_ALLOCATED);
  EXPECT_EQ(duplicate, HAL_kInvalidHandle);
  resource.Free(first);
}

TEST(VMXAnalogGyroTest, SetupUsesFixedRateConfigurationAndInjectableWait) {
  GyroFixture fixture;
  ASSERT_EQ(fixture.gyro.Setup(), AnalogGyroResult::kOk);
  ASSERT_EQ(fixture.hardware->configurations.size(), 2u);
  EXPECT_EQ(fixture.hardware->configurations.back().averageBits, 0);
  EXPECT_EQ(fixture.hardware->configurations.back().oversampleBits, 10);
  EXPECT_TRUE(fixture.hardware->configurations.back().accumulatorEnabled);
  ASSERT_EQ(fixture.waits.size(), 1u);
  EXPECT_DOUBLE_EQ(fixture.waits[0], kAnalogGyroSetupSettleSeconds);
  EXPECT_DOUBLE_EQ(kAnalogGyroSampleRate, 46500.0);
}

TEST(VMXAnalogGyroTest, CalibrationUsesFiveSecondWaitAndRejectsZeroCount) {
  GyroFixture fixture;
  ASSERT_EQ(fixture.gyro.Setup(), AnalogGyroResult::kOk);
  fixture.hardware->accumulatorValue = 101;
  fixture.hardware->accumulatorCount = 4;
  ASSERT_EQ(fixture.gyro.Calibrate(), AnalogGyroResult::kOk);
  ASSERT_EQ(fixture.waits.size(), 2u);
  EXPECT_DOUBLE_EQ(fixture.waits.back(), kAnalogGyroCalibrationSeconds);
  EXPECT_EQ(fixture.gyro.GetCenter().second, 25);
  EXPECT_DOUBLE_EQ(fixture.gyro.GetOffset().second, 0.25);

  fixture.hardware->accumulatorCount = 0;
  EXPECT_EQ(fixture.gyro.Calibrate(), AnalogGyroResult::kZeroCount);
}

TEST(VMXAnalogGyroTest, FixedRateAngleAndRateScalingAndDeadband) {
  GyroFixture fixture;
  ASSERT_EQ(fixture.gyro.Setup(), AnalogGyroResult::kOk);
  fixture.hardware->accumulatorValue = 1000;
  fixture.hardware->accumulatorCount = 10;
  fixture.hardware->averageValue = 2000;

  const double lsb = 5.0 / kVMXADCCounts;
  EXPECT_NEAR(fixture.gyro.GetAngle().second,
              1000.0 * lsb / (kAnalogGyroSampleRate * 0.007), 1e-12);
  EXPECT_NEAR(fixture.gyro.GetRate().second,
              2000.0 * lsb / (1024.0 * 0.007), 1e-12);

  ASSERT_EQ(fixture.gyro.SetDeadband(0.001), AnalogGyroResult::kOk);
  EXPECT_GT(fixture.hardware->configurations.back().deadband, 0);
  EXPECT_EQ(fixture.gyro.SetDeadband(-1.0),
            AnalogGyroResult::kInvalidParameter);
}

TEST(VMXAnalogGyroTest, ResetClearsAccumulatorAndInputCloseIsSafe) {
  GyroFixture fixture;
  ASSERT_EQ(fixture.gyro.Setup(), AnalogGyroResult::kOk);
  fixture.hardware->accumulatorValue = 99;
  fixture.hardware->accumulatorCount = 3;
  ASSERT_EQ(fixture.gyro.Reset(), AnalogGyroResult::kOk);
  EXPECT_EQ(fixture.gyro.GetAngle().second, 0.0);
  EXPECT_EQ(fixture.waits.back(),
            (1.0 / kAnalogGyroSampleRate) * 1024.0);

  fixture.input->Close();
  EXPECT_EQ(fixture.gyro.GetRate().first,
            AnalogGyroResult::kHardwareFailure);
  EXPECT_EQ(fixture.gyro.Reset(), AnalogGyroResult::kHardwareFailure);
}

TEST(VMXAnalogGyroTest, ConcurrentReadAndCloseRemainSafe) {
  GyroFixture fixture;
  ASSERT_EQ(fixture.gyro.Setup(), AnalogGyroResult::kOk);
  std::atomic_bool start{false};
  std::thread reader{[&] {
    while (!start.load(std::memory_order_acquire)) {
    }
    for (int i = 0; i < 100; ++i) {
      fixture.gyro.GetAngle();
      fixture.gyro.GetRate();
    }
  }};
  std::thread closer{[&] {
    start.store(true, std::memory_order_release);
    fixture.input->Close();
  }};
  reader.join();
  closer.join();
  EXPECT_EQ(fixture.gyro.GetAngle().first,
            AnalogGyroResult::kHardwareFailure);
}

}  // namespace
}  // namespace hal::vmx
