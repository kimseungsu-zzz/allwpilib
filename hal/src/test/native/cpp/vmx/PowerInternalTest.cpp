// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "../../../../main/native/vmx/PowerInternal.h"

namespace hal::vmx {
namespace {

TEST(VMXPowerTest, FacadePropagatesReadersAndUnsupportedBrownout) {
  PowerFacade facade{[](double& voltage) {
                       voltage = 12.5;
                       return true;
                     },
                     [](double& temperature) {
                       temperature = 41.25;
                       return true;
                     },
                     [] { return true; }, [] { return false; }};

  double voltage = 0.0;
  EXPECT_EQ(facade.GetVinVoltage(voltage), PowerResult::kOk);
  EXPECT_DOUBLE_EQ(voltage, 12.5);
  double temperature = 0.0;
  EXPECT_EQ(facade.GetCPUTemp(temperature), PowerResult::kOk);
  EXPECT_DOUBLE_EQ(temperature, 41.25);
  bool active = false;
  EXPECT_EQ(facade.GetSystemActive(active), PowerResult::kOk);
  EXPECT_TRUE(active);
  bool valid = true;
  EXPECT_EQ(facade.GetSystemTimeValid(valid), PowerResult::kOk);
  EXPECT_FALSE(valid);
  bool brownedOut = true;
  EXPECT_EQ(facade.GetBrownedOut(brownedOut), PowerResult::kUnsupported);
  EXPECT_FALSE(brownedOut);
}

TEST(VMXPowerTest, FacadeReportsUnavailableReaders) {
  PowerFacade facade{VoltageReader{}, TemperatureReader{}, RuntimeReader{},
                     TimeValidReader{}};
  double value = 1.0;
  EXPECT_EQ(facade.GetVinVoltage(value), PowerResult::kUnavailable);
  EXPECT_DOUBLE_EQ(value, 0.0);
  EXPECT_EQ(facade.GetCPUTemp(value), PowerResult::kUnavailable);
  bool state = true;
  EXPECT_EQ(facade.GetSystemActive(state), PowerResult::kUnavailable);
  EXPECT_FALSE(state);
}

TEST(VMXPowerTest, ParsesThermalMilliCelsiusAndRejectsInvalidInput) {
  double temperature = 0.0;
  EXPECT_TRUE(ParseThermalTemperature("42500\n", temperature));
  EXPECT_DOUBLE_EQ(temperature, 42.5);
  EXPECT_TRUE(ParseThermalTemperature("-1000", temperature));
  EXPECT_DOUBLE_EQ(temperature, -1.0);
  EXPECT_FALSE(ParseThermalTemperature("not-a-temperature", temperature));
  EXPECT_FALSE(ParseThermalTemperature("1000001", temperature));
}

TEST(VMXPowerTest, FindsCpuThermalZoneWithoutAssumingThermalZoneZero) {
  const auto root = std::filesystem::temp_directory_path() /
                    "wpilib-vmx-power-thermal-test";
  std::error_code error;
  std::filesystem::remove_all(root, error);
  std::filesystem::create_directories(root / "thermal_zone7", error);
  ASSERT_FALSE(error);
  {
    std::ofstream{root / "thermal_zone7" / "type"} << "x86_pkg_temp\n";
    std::ofstream{root / "thermal_zone7" / "temp"} << "51000\n";
  }
  double temperature = 0.0;
  EXPECT_TRUE(ReadCpuTemperatureFromSysfs(root.string(), temperature));
  EXPECT_DOUBLE_EQ(temperature, 51.0);
  std::filesystem::remove_all(root, error);
}

TEST(VMXPowerTest, SystemTimeValidityIsIndependentOfMonotonicClock) {
  EXPECT_FALSE(IsSystemTimeValidUnixSeconds(1577836799));
  EXPECT_TRUE(IsSystemTimeValidUnixSeconds(1577836800));
  int64_t seconds = 0;
  EXPECT_TRUE(ReadSystemUnixSeconds(seconds));
  EXPECT_EQ(ReadSystemTimeValidNow(), IsSystemTimeValidUnixSeconds(seconds));
}

}  // namespace
}  // namespace hal::vmx
