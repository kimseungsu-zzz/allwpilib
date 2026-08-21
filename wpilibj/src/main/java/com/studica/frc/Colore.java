// Copyright (c) 2026 WPILib contributors.

package com.studica.frc;

import edu.wpi.first.hal.JNIWrapper;

/** Studica Colore CIE-XYZ/sRGB color sensor vendor API. */
public final class Colore extends JNIWrapper implements AutoCloseable {
  /** Fixed-layout Java view of a matching result. */
  public static final class MatchResult {
    public final String label;
    public final float confidence;

    private MatchResult(String label, float confidence) {
      this.label = label;
      this.confidence = confidence;
    }
  }

  private long handle;
  private int lastStatus;

  /** Creates a Colore attached to a VMX CAN device ID (0..63). */
  public Colore(int canId) {
    long[] result = createCAN(canId);
    initialize(result);
  }

  /** Creates a Colore attached to an explicit Linux USB CDC path. */
  public Colore(String usbPath) {
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

  public synchronized boolean read() {
    lastStatus = handle == 0 ? -107 : read(handle);
    return lastStatus == 0;
  }

  /** sRGB channels are normalized to 0.0..1.0. */
  public synchronized float getRed() {
    return handle == 0 ? 0.0f : getRed(handle);
  }

  public synchronized float getGreen() {
    return handle == 0 ? 0.0f : getGreen(handle);
  }

  public synchronized float getBlue() {
    return handle == 0 ? 0.0f : getBlue(handle);
  }

  /** CIE XYZ values use the device's normalized floating-point units. */
  public synchronized float getX() {
    return handle == 0 ? 0.0f : getX(handle);
  }

  public synchronized float getY() {
    return handle == 0 ? 0.0f : getY(handle);
  }

  public synchronized float getZ() {
    return handle == 0 ? 0.0f : getZ(handle);
  }

  public synchronized boolean setBrightness(int percent) {
    lastStatus = handle == 0 ? -107 : setBrightness(handle, percent);
    return lastStatus == 0;
  }

  public synchronized int getBrightness() {
    return handle == 0 ? 0 : getBrightness(handle);
  }

  /** Returns transport-specific fixed raw configuration bytes. */
  public synchronized byte[] getConfig() {
    return handle == 0 ? new byte[0] : getConfig(handle);
  }

  /** Learned references are in-memory only and are not persistent calibration. */
  public synchronized boolean learnColor(String name) {
    return learnColor(name, 0.05f);
  }

  public synchronized boolean learnColor(String name, float threshold) {
    if (name == null || name.isEmpty()) {
      lastStatus = -22;
      return false;
    }
    lastStatus = handle == 0 ? -107 : learnColor(handle, name, threshold);
    return lastStatus == 0;
  }

  public synchronized boolean setReference(String name, float x, float y) {
    return setReference(name, x, y, 0.05f);
  }

  public synchronized boolean setReference(String name, float x, float y, float threshold) {
    if (name == null || name.isEmpty()) {
      lastStatus = -22;
      return false;
    }
    lastStatus = handle == 0 ? -107 : setReference(handle, name, x, y, threshold);
    return lastStatus == 0;
  }

  public synchronized MatchResult match() {
    if (handle == 0) return new MatchResult("", 0.0f);
    String label = matchLabel(handle);
    float confidence = matchConfidence(handle);
    return new MatchResult(label, confidence);
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

  private static native float getRed(long handle);

  private static native float getGreen(long handle);

  private static native float getBlue(long handle);

  private static native float getX(long handle);

  private static native float getY(long handle);

  private static native float getZ(long handle);

  private static native int setBrightness(long handle, int percent);

  private static native int getBrightness(long handle);

  private static native byte[] getConfig(long handle);

  private static native int learnColor(long handle, String name, float threshold);

  private static native int setReference(
      long handle, String name, float x, float y, float threshold);

  private static native String matchLabel(long handle);

  private static native float matchConfidence(long handle);

  private static native boolean isConnected(long handle);

  private static native int getLastStatus(long handle);
}
