// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.studica.frc;

import edu.wpi.first.hal.JNIWrapper;

/** Titan-specific quadrature/Cypher encoder and limit-switch view. */
public final class TitanQuadEncoder extends JNIWrapper implements AutoCloseable {
  private final TitanQuad motor;

  /** Creates an encoder view on motor port 0 of the default Titan. */
  public TitanQuadEncoder() {
    this(TitanQuad.DEFAULT_CAN_ID, 0);
  }

  /** Creates an encoder view sharing the Titan controller for the given CAN ID. */
  public TitanQuadEncoder(int canId, int motorPort) {
    motor = new TitanQuad(canId, motorPort);
  }

  public boolean isAvailable() {
    return motor.isAvailable();
  }

  public int getLastStatus() {
    return motor.getLastStatus();
  }

  public int getRaw() {
    return getRaw(motorHandle());
  }

  public double getDistance() {
    return getDistance(motorHandle());
  }

  public double getRPM() {
    return getRPM(motorHandle());
  }

  /** Returns the Cypher absolute encoder angle in degrees, not a WPILib duty-cycle value. */
  public double getAbsoluteAngle() {
    return getAbsoluteAngle(motorHandle());
  }

  /** Returns true when the forward limit is triggered (adapter-normalized semantics). */
  public boolean getForwardLimit() {
    return getForwardLimit(motorHandle()) != 0;
  }

  /** Returns true when the reverse limit is triggered (adapter-normalized semantics). */
  public boolean getReverseLimit() {
    return getReverseLimit(motorHandle()) != 0;
  }

  public void setDistancePerTick(double distancePerTick) {
    setDistancePerTick(motorHandle(), distancePerTick);
  }

  public void setReverseDirection(boolean reverse) {
    setReverseDirection(motorHandle(), reverse);
  }

  public boolean reset() {
    return reset(motorHandle()) == 0;
  }

  @Override
  public void close() {
    motor.close();
  }

  private long motorHandle() {
    return motor.getNativeHandle();
  }

  private static native int getRaw(long handle);

  private static native double getDistance(long handle);

  private static native double getRPM(long handle);

  private static native double getAbsoluteAngle(long handle);

  private static native int getForwardLimit(long handle);

  private static native int getReverseLimit(long handle);

  private static native int setDistancePerTick(long handle, double value);

  private static native int setReverseDirection(long handle, boolean reverse);

  private static native int reset(long handle);
}
