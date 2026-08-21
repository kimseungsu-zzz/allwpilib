// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify and/or share it under the WPILib
// BSD license file in the root directory of this project.

package studica.vmx;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class VMXIMUTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void hostBindingDoesNotPretendToHaveAnImu() {
    try (VMXIMU imu = new VMXIMU()) {
      assertFalse(imu.isAvailable());
      assertFalse(imu.isConnected());
      assertTrue(Double.isNaN(imu.getYaw()));
      assertFalse(imu.zeroYaw());
      assertFalse(imu.reset());
    }
  }
}
