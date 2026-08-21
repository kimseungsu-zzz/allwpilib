// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.studica.frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ParsecColoreTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void parsecExposesFixedSnapshotMetadataWithoutFakingHardware() {
    try (Parsec parsec = new Parsec(3)) {
      assertEquals(Parsec.RESOLUTION_4, parsec.getResolution());
      assertEquals(16, parsec.getZoneCount());
      assertEquals(-1, parsec.getDistance(0));
      assertFalse(parsec.hasValidMinDistance());
      assertFalse(parsec.isConnected());
      assertFalse(parsec.read());
    }
    assertThrows(IllegalArgumentException.class, () -> new Parsec(""));
  }

  @Test
  void coloreKeepsBrightnessStrictAndConnectionExplicit() {
    try (Colore colore = new Colore(3)) {
      assertFalse(colore.isConnected());
      assertFalse(colore.setBrightness(-1));
      assertEquals(-22, colore.getLastStatus());
      assertFalse(colore.setBrightness(101));
      assertEquals(-22, colore.getLastStatus());
      assertEquals(0, colore.getBrightness());
      assertFalse(colore.read());
    }
  }
}
