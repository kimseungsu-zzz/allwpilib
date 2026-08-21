// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify and/or share it under the WPILib
// BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include "studica/VMXIMU.h"

TEST(VMXIMUVendorTest, SnapshotAbiIsVersionedAndFixedWidth) {
  EXPECT_EQ(offsetof(StudicaVMXIMUSnapshot, structSize), 0U);
  EXPECT_EQ(offsetof(StudicaVMXIMUSnapshot, abiVersion), 4U);
  EXPECT_EQ(offsetof(StudicaVMXIMUSnapshot, yaw), 8U);
  EXPECT_EQ(offsetof(StudicaVMXIMUSnapshot, sensorTimestamp), 216U);
  EXPECT_EQ(offsetof(StudicaVMXIMUSnapshot, firmwareVersion), 232U);
  EXPECT_EQ(sizeof(StudicaVMXIMUSnapshot), 328U);
}

TEST(VMXIMUVendorTest, HostAbiDoesNotPretendToHaveHardware) {
  StudicaVMXIMUHandle handle = 0;
  ASSERT_EQ(StudicaVMXIMU_Create(&handle), STUDICA_VMX_IMU_OK);
  ASSERT_NE(handle, 0U);

  StudicaVMXIMUSnapshot snapshot;
  EXPECT_EQ(StudicaVMXIMU_ReadSnapshot(handle, &snapshot),
            STUDICA_VMX_IMU_UNAVAILABLE);
  EXPECT_EQ(snapshot.structSize, sizeof(snapshot));
  EXPECT_EQ(snapshot.abiVersion, STUDICA_VMX_IMU_ABI_VERSION);
  EXPECT_TRUE(std::isnan(snapshot.yaw));
  EXPECT_EQ(StudicaVMXIMU_ZeroYaw(handle), STUDICA_VMX_IMU_UNAVAILABLE);
  EXPECT_EQ(StudicaVMXIMU_Reset(handle), STUDICA_VMX_IMU_UNAVAILABLE);
  StudicaVMXIMU_Destroy(handle);
}

TEST(VMXIMUVendorTest, CppWrapperUsesSameStableHandle) {
  studica::VMXIMU imu;
  EXPECT_FALSE(imu.IsAvailable());
  EXPECT_TRUE(std::isnan(imu.GetYaw()));
  EXPECT_EQ(imu.GetLastSensorTimestamp(), 0);
  EXPECT_FALSE(imu.ZeroYaw());
  EXPECT_FALSE(imu.Reset());
}
