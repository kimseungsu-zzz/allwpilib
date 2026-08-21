// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include "studica/Titan.h"

TEST(StudicaTitanVendorTest, AbiIsVersionedAndUsesFixedWidthSnapshot) {
  EXPECT_EQ(offsetof(StudicaTitanSnapshot, structSize), 0U);
  EXPECT_EQ(offsetof(StudicaTitanSnapshot, abiVersion), 4U);
  EXPECT_EQ(offsetof(StudicaTitanSnapshot, canId), 8U);
  EXPECT_EQ(sizeof(StudicaTitanHandle), sizeof(uint32_t));
  EXPECT_EQ(STUDICA_TITAN_MOTOR_COUNT, 4U);
  EXPECT_EQ(STUDICA_TITAN_DEFAULT_CAN_ID, 42U);
}

TEST(StudicaTitanVendorTest, ValidatesCanIdAndMotorPortBeforeAllocation) {
  StudicaTitanHandle handle = 123;
  EXPECT_EQ(StudicaTitan_Create(0, 0, 15600, 0.0, &handle),
            STUDICA_TITAN_INVALID_ARGUMENT);
  EXPECT_EQ(StudicaTitan_Create(63, 0, 15600, 0.0, &handle),
            STUDICA_TITAN_INVALID_ARGUMENT);
  EXPECT_EQ(StudicaTitan_Create(42, 4, 15600, 0.0, &handle),
            STUDICA_TITAN_INVALID_ARGUMENT);
  EXPECT_EQ(StudicaTitan_Create(42, 0, 20001, 0.0, &handle),
            STUDICA_TITAN_INVALID_ARGUMENT);
  EXPECT_EQ(handle, 123U);
}

TEST(StudicaTitanVendorTest, HostBindingDoesNotPretendToHaveTitanHardware) {
  StudicaTitanHandle handle = 0;
  ASSERT_EQ(StudicaTitan_Create(42, 0, 15600, 0.001, &handle),
            STUDICA_TITAN_OK);
  ASSERT_NE(handle, 0U);

  uint8_t available = 1;
  EXPECT_EQ(StudicaTitan_IsAvailable(handle, &available), STUDICA_TITAN_OK);
  EXPECT_EQ(available, 0U);

  EXPECT_EQ(StudicaTitan_Set(handle, 0.25), STUDICA_TITAN_UNAVAILABLE);
  double speed = 0.0;
  EXPECT_EQ(StudicaTitan_Get(handle, &speed), STUDICA_TITAN_UNAVAILABLE);
  EXPECT_TRUE(std::isnan(speed));
  EXPECT_EQ(StudicaTitan_Enable(handle), STUDICA_TITAN_UNAVAILABLE);
  EXPECT_EQ(StudicaTitan_Disable(handle), STUDICA_TITAN_UNAVAILABLE);
  StudicaTitan_Destroy(handle);
  EXPECT_EQ(StudicaTitan_Get(handle, &speed), STUDICA_TITAN_INVALID_ARGUMENT);
}

TEST(StudicaTitanVendorTest, CppViewsUseStableCAbiAndShareConfigurationContract) {
  studica::TitanQuad motor{42, 2};
  studica::TitanQuadEncoder encoder{42, 2};
  EXPECT_FALSE(motor.IsAvailable());
  EXPECT_FALSE(encoder.IsAvailable());
  motor.Set(2.0);
  EXPECT_EQ(motor.GetLastStatus(), STUDICA_TITAN_UNAVAILABLE);
  EXPECT_TRUE(std::isnan(motor.Get()));
  EXPECT_EQ(encoder.GetRaw(), 0);
  EXPECT_TRUE(std::isnan(encoder.GetDistance()));
}
