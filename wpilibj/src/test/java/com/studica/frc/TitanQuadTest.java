// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.studica.frc;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class TitanQuadTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void hostBindingDoesNotPretendToHaveTitanHardware() {
    try (TitanQuad motor = new TitanQuad(42, 0);
        TitanQuadEncoder encoder = new TitanQuadEncoder(42, 0)) {
      assertFalse(motor.isAvailable());
      assertFalse(encoder.isAvailable());
      motor.set(0.5);
      assertTrue(motor.getLastStatus() != 0);
      assertFalse(encoder.reset());
    }
  }
}
