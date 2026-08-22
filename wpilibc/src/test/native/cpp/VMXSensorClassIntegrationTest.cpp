// Copyright (c) 2026 WPILib contributors.

// This suite intentionally instantiates the real WPILib classes.  The values
// are supplied through the public simulation/mock HAL data handles, so the
// test validates class -> HAL C ABI conversions without duplicating any class
// implementation.  VMX adapter tests in hal/src/test/native/cpp/vmx validate
// the corresponding backend -> SDK seam.

#include <gtest/gtest.h>

#include <hal/HAL.h>

#include "frc/ADXL345_I2C.h"
#include "frc/ADXL345_SPI.h"
#include "frc/ADXL362.h"
#include "frc/ADXRS450_Gyro.h"
#include "frc/AnalogAccelerometer.h"
#include "frc/AnalogEncoder.h"
#include "frc/AnalogGyro.h"
#include "frc/AnalogInput.h"
#include "frc/AnalogPotentiometer.h"
#include "frc/DigitalInput.h"
#include "frc/DutyCycle.h"
#include "frc/DutyCycleEncoder.h"
#include "frc/Encoder.h"
#include "frc/SharpIR.h"
#include "frc/Ultrasonic.h"
#include "frc/counter/Tachometer.h"
#include "frc/simulation/ADXL345Sim.h"
#include "frc/simulation/ADXL362Sim.h"
#include "frc/simulation/ADXRS450_GyroSim.h"
#include "frc/simulation/AnalogEncoderSim.h"
#include "frc/simulation/AnalogInputSim.h"
#include "frc/simulation/AnalogGyroSim.h"
#include "frc/simulation/DutyCycleSim.h"
#include "frc/simulation/DutyCycleEncoderSim.h"
#include "frc/simulation/EncoderSim.h"
#include "frc/simulation/SharpIRSim.h"
#include "frc/simulation/UltrasonicSim.h"

namespace frc {

// Simulation data is process-global and outlives the Sim wrapper objects, so a
// test that leaves values behind corrupts whichever suite gtest happens to run
// next. That ordering follows link order and differs per platform: leaving
// AnalogGyro channel 0 at rate -3.0 here broke AnalogGyroSimTest.SetRate on
// macOS only, where this file registers first. Reset what these tests dirty,
// for every Sim class that exposes ResetData.

TEST(VMXSensorClassIntegrationTest, AnalogClassesUsePublicHalValues) {
  HAL_Initialize(500, 0);

  AnalogInput input{0};
  sim::AnalogInputSim inputSim{input};
  inputSim.SetVoltage(2.5);

  AnalogPotentiometer potentiometer{&input, 360.0};
  EXPECT_DOUBLE_EQ(potentiometer.Get(), 180.0);

  AnalogAccelerometer accelerometer{&input};
  accelerometer.SetZero(2.5);
  accelerometer.SetSensitivity(0.5);
  EXPECT_DOUBLE_EQ(accelerometer.GetAcceleration(), 0.0);

  AnalogEncoder encoder{&input};
  sim::AnalogEncoderSim encoderSim{encoder};
  encoderSim.Set(0.5);
  EXPECT_NEAR(encoder.Get(), 0.5, 1e-9);

  auto sharp = SharpIR::GP2Y0A02YK0F(1);
  SharpIRSim sharpSim{sharp};
  sharpSim.SetRange(units::centimeter_t{30});
  EXPECT_EQ(sharp.GetRange().value(), 30);

  inputSim.ResetData();
}

TEST(VMXSensorClassIntegrationTest, EncoderDutyCycleAndTachometerReadbacks) {
  HAL_Initialize(500, 0);

  Encoder encoder{2, 3};
  sim::EncoderSim encoderSim{encoder};
  encoderSim.SetCount(100);
  encoder.SetDistancePerPulse(0.25);
  EXPECT_DOUBLE_EQ(encoder.GetDistance(), 25.0);
  encoder.SetReverseDirection(true);
  EXPECT_DOUBLE_EQ(encoder.GetDistance(), 25.0);

  DutyCycleEncoder dutyCycleEncoder{4};
  sim::DutyCycleEncoderSim dutyCycleSim{dutyCycleEncoder};
  dutyCycleSim.SetConnected(true);
  dutyCycleSim.Set(0.375);
  EXPECT_TRUE(dutyCycleEncoder.IsConnected());
  EXPECT_NEAR(dutyCycleEncoder.Get(), 0.375, 1e-9);

  DigitalInput tachInput{5};
  Tachometer tachometer{tachInput};
  tachometer.SetEdgesPerRevolution(2);
  EXPECT_EQ(tachometer.GetEdgesPerRevolution(), 2);

  DigitalInput dutyInput{8};
  DutyCycle dutyCycle{dutyInput};
  sim::DutyCycleSim inputDutyCycleSim{dutyCycle};
  inputDutyCycleSim.SetFrequency(1000);
  inputDutyCycleSim.SetOutput(0.375);
  EXPECT_EQ(dutyCycle.GetFrequency(), 1000);
  EXPECT_DOUBLE_EQ(dutyCycle.GetOutput(), 0.375);

  encoderSim.ResetData();
  inputDutyCycleSim.ResetData();
}

TEST(VMXSensorClassIntegrationTest, UltrasonicUsesPulseRangeAndValidity) {
  HAL_Initialize(500, 0);
  Ultrasonic ultrasonic{6, 7};
  sim::UltrasonicSim ultrasonicSim{ultrasonic};
  ultrasonicSim.SetRange(units::meter_t{1.25});
  EXPECT_DOUBLE_EQ(ultrasonic.GetRange().value(), 1.25);
  ultrasonicSim.SetRangeValid(false);
  EXPECT_FALSE(ultrasonic.IsRangeValid());
}

TEST(VMXSensorClassIntegrationTest, I2CAndSpiAccelerometersExposeAxes) {
  HAL_Initialize(500, 0);
  {
    ADXL345_I2C accel{I2C::kMXP};
    sim::ADXL345Sim sim{accel};
    sim.SetX(1.0);
    sim.SetY(-2.0);
    sim.SetZ(3.0);
    EXPECT_DOUBLE_EQ(accel.GetX(), 1.0);
    EXPECT_DOUBLE_EQ(accel.GetY(), -2.0);
    EXPECT_DOUBLE_EQ(accel.GetZ(), 3.0);
  }
  {
    ADXL345_SPI accel{SPI::kMXP};
    sim::ADXL345Sim sim{accel};
    sim.SetX(0.5);
    EXPECT_DOUBLE_EQ(accel.GetX(), 0.5);
  }
  {
    ADXL362 accel;
    sim::ADXL362Sim sim{accel};
    sim.SetX(0.25);
    sim.SetY(-0.5);
    sim.SetZ(0.75);
    EXPECT_DOUBLE_EQ(accel.GetX(), 0.25);
    EXPECT_DOUBLE_EQ(accel.GetY(), -0.5);
    EXPECT_DOUBLE_EQ(accel.GetZ(), 0.75);
  }
}

TEST(VMXSensorClassIntegrationTest, GyroClassesExposeAngleAndRate) {
  HAL_Initialize(500, 0);
  {
    AnalogGyro gyro{0, 2048, 0.0};
    sim::AnalogGyroSim sim{gyro};
    sim.SetAngle(12.5);
    sim.SetRate(-3.0);
    EXPECT_DOUBLE_EQ(gyro.GetAngle(), 12.5);
    EXPECT_DOUBLE_EQ(gyro.GetRate(), -3.0);
    sim.ResetData();
  }
  {
    ADXRS450_Gyro gyro;
    sim::ADXRS450_GyroSim sim{gyro};
    sim.SetAngle(units::degree_t{22.0});
    sim.SetRate(units::degrees_per_second_t{4.0});
    EXPECT_DOUBLE_EQ(gyro.GetAngle(), 22.0);
    EXPECT_DOUBLE_EQ(gyro.GetRate(), 4.0);
  }
}

}  // namespace frc
