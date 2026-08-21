// Copyright (c) 2026 WPILib contributors.

package com.studica.frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

class CobraLightTowerTest {
  @Test
  void cobraUsesStableVendorMetadataOnHost() {
    try (Cobra cobra = new Cobra()) {
      assertEquals(Cobra.CHANNEL_COUNT, cobra.getChannelCount());
      assertEquals(5.0, cobra.getReferenceVoltage());
      assertEquals(-1, cobra.getRaw(0));
      assertFalse(cobra.isAvailable());
    }
  }

  @Test
  void lightTowerKeepsHardwareUnavailableExplicit() {
    try (LightTower tower = new LightTower(1, 2, 3, 4, 5)) {
      assertFalse(tower.isAvailable());
      assertFalse(tower.setSolid());
      assertEquals(-19, tower.getLastStatus());
    }
  }
}
