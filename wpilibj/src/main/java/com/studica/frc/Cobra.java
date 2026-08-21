// Copyright (c) 2026 WPILib contributors.

package com.studica.frc;

import edu.wpi.first.hal.JNIWrapper;

/** Studica Cobra four-channel reflectance array vendor API. */
public final class Cobra extends JNIWrapper implements AutoCloseable {
  public static final int CHANNEL_COUNT = 4;
  public static final double DEFAULT_REFERENCE_VOLTAGE = 5.0;

  private long handle;
  private int lastStatus;

  /** Creates a Cobra using the driver's reference-voltage semantics. */
  public Cobra() {
    this(DEFAULT_REFERENCE_VOLTAGE);
  }

  public Cobra(double referenceVoltage) {
    long[] result = create(referenceVoltage);
    if (result == null || result.length < 2) {
      handle = 0;
      lastStatus = -5;
    } else {
      handle = result[0];
      lastStatus = (int) result[1];
    }
  }

  public synchronized int getRaw(int channel) {
    if (channel < 0 || channel >= CHANNEL_COUNT) {
      lastStatus = -22;
      return -1;
    }
    return handle == 0 ? -1 : getRaw(handle, channel);
  }

  public synchronized double getVoltage(int channel) {
    if (channel < 0 || channel >= CHANNEL_COUNT) {
      lastStatus = -22;
      return 0.0;
    }
    return handle == 0 ? 0.0 : getVoltage(handle, channel);
  }

  public synchronized int getChannelCount() {
    return handle == 0 ? 0 : getChannelCount(handle);
  }

  public synchronized double getReferenceVoltage() {
    return handle == 0 ? 0.0 : getReferenceVoltage(handle);
  }

  public synchronized boolean isAvailable() {
    return handle != 0 && isAvailable(handle);
  }

  public synchronized int getLastStatus() {
    return lastStatus;
  }

  @Override
  public synchronized void close() {
    if (handle != 0) {
      destroy(handle);
      handle = 0;
    }
  }

  private static native long[] create(double referenceVoltage);

  private static native void destroy(long handle);

  private static native int getRaw(long handle, int channel);

  private static native double getVoltage(long handle, int channel);

  private static native int getChannelCount(long handle);

  private static native double getReferenceVoltage(long handle);

  private static native boolean isAvailable(long handle);
}
