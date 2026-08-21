// Copyright (c) 2026 WPILib contributors.

package com.studica.frc;

import edu.wpi.first.hal.JNIWrapper;

/** Studica Parsec 4x4/8x8 distance sensor vendor API. */
public final class Parsec extends JNIWrapper implements AutoCloseable {
  public static final int MAX_ZONES = 64;
  public static final int RESOLUTION_4 = 4;
  public static final int RESOLUTION_8 = 8;

  private long handle;
  private int lastStatus;

  /** Creates a Parsec attached to a VMX CAN device ID (0..63). */
  public Parsec(int canId) {
    long[] result = createCAN(canId);
    initialize(result);
  }

  /** Creates a Parsec attached to an explicit Linux USB CDC path. */
  public Parsec(String usbPath) {
    if (usbPath == null || usbPath.isEmpty()) {
      throw new IllegalArgumentException("USB path must not be empty");
    }
    long[] result = createUSB(usbPath);
    initialize(result);
  }

  private void initialize(long[] result) {
    if (result == null || result.length < 2) {
      handle = 0;
      lastStatus = -5;
    } else {
      handle = result[0];
      lastStatus = (int) result[1];
    }
  }

  /** Reads one current snapshot from the selected transport. */
  public synchronized boolean read() {
    lastStatus = handle == 0 ? -107 : read(handle);
    return lastStatus == 0;
  }

  public synchronized int getResolution() {
    return handle == 0 ? 0 : getResolution(handle);
  }

  public synchronized int getZoneCount() {
    return handle == 0 ? 0 : getZoneCount(handle);
  }

  /** Returns raw millimeter values, preserving -1 and -2 sentinel values. */
  public synchronized short[] getDistances() {
    if (handle == 0) return new short[0];
    return getDistances(handle);
  }

  public synchronized short getDistance(int zone) {
    if (zone < 0 || zone >= getZoneCount()) {
      lastStatus = -22;
      return -2;
    }
    short[] values = getDistances();
    return zone < values.length ? values[zone] : -2;
  }

  /** Returns the nearest valid distance, or -1 when no zone is valid. */
  public synchronized int getMinDistance() {
    int[] result = handle == 0 ? new int[] {0, 0, -107} : getMinDistance(handle);
    if (result.length < 3) {
      lastStatus = -5;
      return -1;
    }
    lastStatus = result[2];
    return result[1] != 0 ? result[0] : -1;
  }

  public synchronized boolean hasValidMinDistance() {
    int[] result = handle == 0 ? new int[] {0, 0, -107} : getMinDistance(handle);
    if (result.length < 3) {
      lastStatus = -5;
      return false;
    }
    lastStatus = result[2];
    return result[1] != 0;
  }

  /** Returns the raw GETCONFIG response; CAN and USB formats may differ. */
  public synchronized byte[] getConfig() {
    return handle == 0 ? new byte[0] : getConfig(handle);
  }

  public synchronized boolean isConnected() {
    return handle != 0 && isConnected(handle);
  }

  public synchronized int getLastStatus() {
    if (handle != 0) lastStatus = getLastStatus(handle);
    return lastStatus;
  }

  @Override
  public synchronized void close() {
    if (handle != 0) {
      destroy(handle);
      handle = 0;
    }
  }

  private static native long[] createCAN(int canId);

  private static native long[] createUSB(String path);

  private static native void destroy(long handle);

  private static native int read(long handle);

  private static native int getResolution(long handle);

  private static native int getZoneCount(long handle);

  private static native short[] getDistances(long handle);

  private static native int[] getMinDistance(long handle);

  private static native byte[] getConfig(long handle);

  private static native boolean isConnected(long handle);

  private static native int getLastStatus(long handle);
}
