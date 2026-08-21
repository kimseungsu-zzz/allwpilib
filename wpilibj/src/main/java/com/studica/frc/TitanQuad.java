// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.studica.frc;

import edu.wpi.first.hal.JNIWrapper;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** Compatibility wrapper for one channel of a Studica Titan Quad controller. */
public final class TitanQuad extends JNIWrapper implements MotorController, AutoCloseable {
  public static final int DEFAULT_CAN_ID = 42;
  public static final int MOTOR_COUNT = 4;
  private static final int DEFAULT_MOTOR_FREQUENCY_HZ = 15600;

  private long handle;
  private int lastStatus;

  /** Creates motor port 0 on the default Titan CAN ID (42). */
  public TitanQuad() {
    this(DEFAULT_CAN_ID, 0);
  }

  /** Creates a Titan motor view using the default motor frequency. */
  public TitanQuad(int canId, int motorPort) {
    this(canId, motorPort, DEFAULT_MOTOR_FREQUENCY_HZ, 0.0);
  }

  /** Creates a Titan motor view. All views for one CAN ID share one controller worker. */
  public TitanQuad(int canId, int motorPort, int motorFrequencyHz, double distancePerTick) {
    long[] result = create(canId, motorPort, motorFrequencyHz, distancePerTick);
    if (result == null || result.length < 2) {
      handle = 0;
      lastStatus = -5;
    } else {
      handle = result[0];
      lastStatus = (int) result[1];
    }
  }

  /** Returns the last native status observed by a mutating operation. */
  public synchronized int getLastStatus() {
    return lastStatus;
  }

  synchronized long getNativeHandle() {
    return handle;
  }

  public synchronized boolean isAvailable() {
    return handle != 0 && isAvailable(handle) != 0;
  }

  @Override
  public synchronized void set(double speed) {
    lastStatus = handle == 0 ? -107 : set(handle, speed);
  }

  @Override
  public synchronized double get() {
    if (handle == 0) return Double.NaN;
    double[] result = get(handle);
    lastStatus = (int) result[1];
    return result[0];
  }

  @Override
  public synchronized void setInverted(boolean inverted) {
    lastStatus = handle == 0 ? -107 : setInverted(handle, inverted);
  }

  @Override
  public synchronized boolean getInverted() {
    if (handle == 0) return false;
    long[] result = getInverted(handle);
    lastStatus = (int) result[1];
    return result[0] != 0;
  }

  /** Enables this Titan controller. The controller is shared by all four channel views. */
  public synchronized void enable() {
    lastStatus = handle == 0 ? -107 : enable(handle);
  }

  @Override
  public synchronized void disable() {
    lastStatus = handle == 0 ? -107 : disable(handle);
  }

  @Override
  public synchronized void stopMotor() {
    lastStatus = handle == 0 ? -107 : stopMotor(handle);
  }

  public synchronized void setTargetVelocity(float rpm) {
    lastStatus = handle == 0 ? -107 : setTargetVelocity(handle, rpm);
  }

  public synchronized void setTargetDistance(int encoderCounts) {
    lastStatus = handle == 0 ? -107 : setTargetDistance(handle, encoderCounts);
  }

  public synchronized void setTargetAngle(double angleDegrees) {
    lastStatus = handle == 0 ? -107 : setTargetAngle(handle, angleDegrees);
  }

  public synchronized void setPositionHold(boolean hold) {
    lastStatus = handle == 0 ? -107 : setPositionHold(handle, hold);
  }

  public synchronized void setCurrentLimit(float amps) {
    lastStatus = handle == 0 ? -107 : setCurrentLimit(handle, amps);
  }

  public synchronized void setCurrentLimitMode(int mode) {
    lastStatus = handle == 0 ? -107 : setCurrentLimitMode(handle, mode);
  }

  public synchronized void setMotorStopMode(int mode) {
    lastStatus = handle == 0 ? -107 : setMotorStopMode(handle, mode);
  }

  public synchronized void setPIDType(int type) {
    lastStatus = handle == 0 ? -107 : setPIDType(handle, type);
  }

  public synchronized String getFirmwareVersion() {
    return handle == 0 ? "" : getFirmwareVersion(handle);
  }

  public synchronized String getHardwareVersion() {
    return handle == 0 ? "" : getHardwareVersion(handle);
  }

  public synchronized float getControllerTemperature() {
    return handle == 0 ? Float.NaN : getControllerTemperature(handle);
  }

  @Override
  public synchronized void close() {
    if (handle != 0) {
      destroy(handle);
      handle = 0;
    }
  }

  private static native long[] create(
      int canId, int motorPort, int motorFrequencyHz, double distancePerTick);

  private static native void destroy(long handle);

  private static native int set(long handle, double speed);

  private static native double[] get(long handle);

  private static native int setInverted(long handle, boolean inverted);

  private static native long[] getInverted(long handle);

  private static native int enable(long handle);

  private static native int disable(long handle);

  private static native int stopMotor(long handle);

  private static native int isAvailable(long handle);

  private static native int setTargetVelocity(long handle, float rpm);

  private static native int setTargetDistance(long handle, int encoderCounts);

  private static native int setTargetAngle(long handle, double angleDegrees);

  private static native int setPositionHold(long handle, boolean hold);

  private static native int setCurrentLimit(long handle, float amps);

  private static native int setCurrentLimitMode(long handle, int mode);

  private static native int setMotorStopMode(long handle, int mode);

  private static native int setPIDType(long handle, int type);

  private static native String getFirmwareVersion(long handle);

  private static native String getHardwareVersion(long handle);

  private static native float getControllerTemperature(long handle);
}
