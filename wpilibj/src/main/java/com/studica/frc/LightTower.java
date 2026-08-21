// Copyright (c) 2026 WPILib contributors.

package com.studica.frc;

import edu.wpi.first.hal.JNIWrapper;

/** Studica Light Tower vendor API backed by five VMX physical DIO outputs. */
public final class LightTower extends JNIWrapper implements AutoCloseable {
  private long handle;
  private int lastStatus;

  /** Creates a tower using physical VMX channels for continuous, R, G, Y, buzzer. */
  public LightTower(int continuous, int red, int green, int yellow, int buzzer) {
    validatePins(continuous, red, green, yellow, buzzer);
    long[] result = create(continuous, red, green, yellow, buzzer);
    if (result == null || result.length < 2) {
      handle = 0;
      lastStatus = -5;
    } else {
      handle = result[0];
      lastStatus = (int) result[1];
    }
  }

  private static void validatePins(int... pins) {
    for (int i = 0; i < pins.length; ++i) {
      if (pins[i] < 0 || pins[i] >= 34) {
        throw new IllegalArgumentException("VMX physical DIO channel out of range");
      }
      for (int j = 0; j < i; ++j) {
        if (pins[i] == pins[j]) {
          throw new IllegalArgumentException("Light Tower outputs must be distinct");
        }
      }
    }
  }

  public synchronized int getLastStatus() {
    return lastStatus;
  }

  public synchronized boolean setRed(boolean enabled) {
    lastStatus = handle == 0 ? -107 : setRed(handle, enabled);
    return lastStatus == 0;
  }

  public synchronized boolean setYellow(boolean enabled) {
    lastStatus = handle == 0 ? -107 : setYellow(handle, enabled);
    return lastStatus == 0;
  }

  public synchronized boolean setGreen(boolean enabled) {
    lastStatus = handle == 0 ? -107 : setGreen(handle, enabled);
    return lastStatus == 0;
  }

  public synchronized boolean setBuzzer(boolean enabled) {
    lastStatus = handle == 0 ? -107 : setBuzzer(handle, enabled);
    return lastStatus == 0;
  }

  public synchronized boolean setSolid() {
    lastStatus = handle == 0 ? -107 : setSolid(handle);
    return lastStatus == 0;
  }

  public synchronized boolean setBlink() {
    lastStatus = handle == 0 ? -107 : setBlink(handle);
    return lastStatus == 0;
  }

  public synchronized boolean off() {
    lastStatus = handle == 0 ? -107 : off(handle);
    return lastStatus == 0;
  }

  public synchronized boolean isAvailable() {
    return handle != 0 && isAvailable(handle);
  }

  @Override
  public synchronized void close() {
    if (handle != 0) {
      destroy(handle);
      handle = 0;
    }
  }

  private static native long[] create(int continuous, int red, int green, int yellow, int buzzer);

  private static native void destroy(long handle);

  private static native int setRed(long handle, boolean enabled);

  private static native int setYellow(long handle, boolean enabled);

  private static native int setGreen(long handle, boolean enabled);

  private static native int setBuzzer(long handle, boolean enabled);

  private static native int setSolid(long handle);

  private static native int setBlink(long handle);

  private static native int off(long handle);

  private static native boolean isAvailable(long handle);
}
