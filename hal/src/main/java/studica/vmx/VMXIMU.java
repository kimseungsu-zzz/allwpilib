// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify and/or share it under the WPILib
// BSD license file in the root directory of this project.

package studica.vmx;

import edu.wpi.first.hal.JNIWrapper;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;

/**
 * Vendor API for the VMX onboard AHRS/IMU.
 *
 * <p>This is deliberately separate from WPILib's core accelerometer and gyro APIs. It uses the
 * VMXPi/AHRS instance already owned by the VMX runtime and does not allocate or stop a second
 * hardware object. Raw acceleration is in sensor-frame G with gravity included; world-linear
 * acceleration is the gravity-corrected, world-frame value. Sensor timestamps are opaque SDK
 * timestamps and are not FPGA time.
 */
public final class VMXIMU extends JNIWrapper implements AutoCloseable {
  private long handle;

  /** Creates a wrapper over the shared VMX runtime AHRS. */
  public VMXIMU() {
    handle = create();
  }

  /** Returns whether the shared VMX AHRS is available and connected. */
  public synchronized boolean isAvailable() {
    return handle != 0 && getSnapshot().isConnected();
  }

  /** Reads one coherent vendor snapshot. */
  public synchronized Snapshot getSnapshot() {
    if (handle == 0) {
      return Snapshot.unavailable();
    }
    byte[] data = readSnapshot(handle);
    return data == null ? Snapshot.unavailable() : Snapshot.fromBytes(data);
  }

  public double getYaw() {
    return getSnapshot().getYaw();
  }

  public double getPitch() {
    return getSnapshot().getPitch();
  }

  public double getRoll() {
    return getSnapshot().getRoll();
  }

  public double getAccumulatedAngle() {
    return getSnapshot().getAccumulatedAngle();
  }

  public double getYawRate() {
    return getSnapshot().getYawRate();
  }

  public double getQuaternionW() {
    return getSnapshot().getQuaternionW();
  }

  public double getQuaternionX() {
    return getSnapshot().getQuaternionX();
  }

  public double getQuaternionY() {
    return getSnapshot().getQuaternionY();
  }

  public double getQuaternionZ() {
    return getSnapshot().getQuaternionZ();
  }

  public double getRawGyroX() {
    return getSnapshot().getRawGyroX();
  }

  public double getRawGyroY() {
    return getSnapshot().getRawGyroY();
  }

  public double getRawGyroZ() {
    return getSnapshot().getRawGyroZ();
  }

  public double getRawAccelX() {
    return getSnapshot().getRawAccelX();
  }

  public double getRawAccelY() {
    return getSnapshot().getRawAccelY();
  }

  public double getRawAccelZ() {
    return getSnapshot().getRawAccelZ();
  }

  public double getRawMagX() {
    return getSnapshot().getRawMagX();
  }

  public double getRawMagY() {
    return getSnapshot().getRawMagY();
  }

  public double getRawMagZ() {
    return getSnapshot().getRawMagZ();
  }

  public double getWorldLinearAccelX() {
    return getSnapshot().getWorldLinearAccelX();
  }

  public double getWorldLinearAccelY() {
    return getSnapshot().getWorldLinearAccelY();
  }

  public double getWorldLinearAccelZ() {
    return getSnapshot().getWorldLinearAccelZ();
  }

  public double getCompassHeading() {
    return getSnapshot().getCompassHeading();
  }

  public double getFusedHeading() {
    return getSnapshot().getFusedHeading();
  }

  public boolean isMoving() {
    return getSnapshot().isMoving();
  }

  public boolean isRotating() {
    return getSnapshot().isRotating();
  }

  public boolean isCalibrating() {
    return getSnapshot().isCalibrating();
  }

  public boolean isConnected() {
    return getSnapshot().isConnected();
  }

  public long getLastSensorTimestamp() {
    return getSnapshot().getSensorTimestamp();
  }

  public double getTemperatureC() {
    return getSnapshot().getTemperatureC();
  }

  public double getPressure() {
    return getSnapshot().getPressure();
  }

  public double getAltitude() {
    return getSnapshot().getAltitude();
  }

  public boolean isAltitudeValid() {
    return getSnapshot().isAltitudeValid();
  }

  public String getFirmwareVersion() {
    return getSnapshot().getFirmwareVersion();
  }

  /** Zeros the current yaw reference. */
  public synchronized boolean zeroYaw() {
    return handle != 0 && zeroYaw(handle) == 0;
  }

  /** Resets the AHRS continuous angle/displacement state. */
  public synchronized boolean reset() {
    return handle != 0 && reset(handle) == 0;
  }

  @Override
  public synchronized void close() {
    if (handle != 0) {
      destroy(handle);
      handle = 0;
    }
  }

  private static native long create();

  private static native void destroy(long handle);

  private static native byte[] readSnapshot(long handle);

  private static native int zeroYaw(long handle);

  private static native int reset(long handle);

  /** Immutable values read from one VMX AHRS sample. */
  public static final class Snapshot {
    private static final int STRUCT_SIZE_OFFSET = 0;
    private static final int ABI_VERSION_OFFSET = 4;
    private static final int DOUBLE_OFFSET = 8;
    private static final int TIMESTAMP_OFFSET = 216;
    private static final int FLAGS_OFFSET = 224;
    private static final int FIRMWARE_OFFSET = 232;
    private static final int FIRMWARE_LENGTH = 96;
    private static final int ABI_VERSION = 1;

    private final double yaw;
    private final double pitch;
    private final double roll;
    private final double accumulatedAngle;
    private final double yawRate;
    private final double quaternionW;
    private final double quaternionX;
    private final double quaternionY;
    private final double quaternionZ;
    private final double rawGyroX;
    private final double rawGyroY;
    private final double rawGyroZ;
    private final double rawAccelX;
    private final double rawAccelY;
    private final double rawAccelZ;
    private final double rawMagX;
    private final double rawMagY;
    private final double rawMagZ;
    private final double worldLinearAccelX;
    private final double worldLinearAccelY;
    private final double worldLinearAccelZ;
    private final double compassHeading;
    private final double fusedHeading;
    private final double temperatureC;
    private final double pressure;
    private final double altitude;
    private final long sensorTimestamp;
    private final boolean moving;
    private final boolean rotating;
    private final boolean calibrating;
    private final boolean connected;
    private final boolean altitudeValid;
    private final String firmwareVersion;

    private Snapshot(
        double yaw,
        double pitch,
        double roll,
        double accumulatedAngle,
        double yawRate,
        double quaternionW,
        double quaternionX,
        double quaternionY,
        double quaternionZ,
        double rawGyroX,
        double rawGyroY,
        double rawGyroZ,
        double rawAccelX,
        double rawAccelY,
        double rawAccelZ,
        double rawMagX,
        double rawMagY,
        double rawMagZ,
        double worldLinearAccelX,
        double worldLinearAccelY,
        double worldLinearAccelZ,
        double compassHeading,
        double fusedHeading,
        double temperatureC,
        double pressure,
        double altitude,
        long sensorTimestamp,
        boolean moving,
        boolean rotating,
        boolean calibrating,
        boolean connected,
        boolean altitudeValid,
        String firmwareVersion) {
      this.yaw = yaw;
      this.pitch = pitch;
      this.roll = roll;
      this.accumulatedAngle = accumulatedAngle;
      this.yawRate = yawRate;
      this.quaternionW = quaternionW;
      this.quaternionX = quaternionX;
      this.quaternionY = quaternionY;
      this.quaternionZ = quaternionZ;
      this.rawGyroX = rawGyroX;
      this.rawGyroY = rawGyroY;
      this.rawGyroZ = rawGyroZ;
      this.rawAccelX = rawAccelX;
      this.rawAccelY = rawAccelY;
      this.rawAccelZ = rawAccelZ;
      this.rawMagX = rawMagX;
      this.rawMagY = rawMagY;
      this.rawMagZ = rawMagZ;
      this.worldLinearAccelX = worldLinearAccelX;
      this.worldLinearAccelY = worldLinearAccelY;
      this.worldLinearAccelZ = worldLinearAccelZ;
      this.compassHeading = compassHeading;
      this.fusedHeading = fusedHeading;
      this.temperatureC = temperatureC;
      this.pressure = pressure;
      this.altitude = altitude;
      this.sensorTimestamp = sensorTimestamp;
      this.moving = moving;
      this.rotating = rotating;
      this.calibrating = calibrating;
      this.connected = connected;
      this.altitudeValid = altitudeValid;
      this.firmwareVersion = firmwareVersion;
    }

    private static Snapshot unavailable() {
      double nan = Double.NaN;
      return new Snapshot(
          nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan,
          nan, nan, nan, nan, nan, nan, nan, nan, 0L, false, false, false, false, false, "");
    }

    private static Snapshot fromBytes(byte[] data) {
      if (data.length < FIRMWARE_OFFSET + FIRMWARE_LENGTH) {
        return unavailable();
      }
      ByteBuffer buffer = ByteBuffer.wrap(data).order(ByteOrder.nativeOrder());
      if (buffer.getInt(STRUCT_SIZE_OFFSET) != data.length
          || buffer.getInt(ABI_VERSION_OFFSET) != ABI_VERSION) {
        return unavailable();
      }
      double[] values = new double[26];
      for (int i = 0; i < values.length; i++) {
        values[i] = buffer.getDouble(DOUBLE_OFFSET + i * Double.BYTES);
      }
      String firmware = new String(data, FIRMWARE_OFFSET, FIRMWARE_LENGTH, StandardCharsets.UTF_8);
      int terminator = firmware.indexOf('\0');
      if (terminator >= 0) {
        firmware = firmware.substring(0, terminator);
      }
      return new Snapshot(
          values[0],
          values[1],
          values[2],
          values[3],
          values[4],
          values[5],
          values[6],
          values[7],
          values[8],
          values[9],
          values[10],
          values[11],
          values[12],
          values[13],
          values[14],
          values[15],
          values[16],
          values[17],
          values[18],
          values[19],
          values[20],
          values[21],
          values[22],
          values[23],
          values[24],
          values[25],
          buffer.getLong(TIMESTAMP_OFFSET),
          buffer.get(FLAGS_OFFSET) != 0,
          buffer.get(FLAGS_OFFSET + 1) != 0,
          buffer.get(FLAGS_OFFSET + 2) != 0,
          buffer.get(FLAGS_OFFSET + 3) != 0,
          buffer.get(FLAGS_OFFSET + 4) != 0,
          firmware);
    }

    public double getYaw() {
      return yaw;
    }

    public double getPitch() {
      return pitch;
    }

    public double getRoll() {
      return roll;
    }

    public double getAccumulatedAngle() {
      return accumulatedAngle;
    }

    public double getYawRate() {
      return yawRate;
    }

    public double getQuaternionW() {
      return quaternionW;
    }

    public double getQuaternionX() {
      return quaternionX;
    }

    public double getQuaternionY() {
      return quaternionY;
    }

    public double getQuaternionZ() {
      return quaternionZ;
    }

    public double getRawGyroX() {
      return rawGyroX;
    }

    public double getRawGyroY() {
      return rawGyroY;
    }

    public double getRawGyroZ() {
      return rawGyroZ;
    }

    public double getRawAccelX() {
      return rawAccelX;
    }

    public double getRawAccelY() {
      return rawAccelY;
    }

    public double getRawAccelZ() {
      return rawAccelZ;
    }

    public double getRawMagX() {
      return rawMagX;
    }

    public double getRawMagY() {
      return rawMagY;
    }

    public double getRawMagZ() {
      return rawMagZ;
    }

    public double getWorldLinearAccelX() {
      return worldLinearAccelX;
    }

    public double getWorldLinearAccelY() {
      return worldLinearAccelY;
    }

    public double getWorldLinearAccelZ() {
      return worldLinearAccelZ;
    }

    public double getCompassHeading() {
      return compassHeading;
    }

    public double getFusedHeading() {
      return fusedHeading;
    }

    public boolean isMoving() {
      return moving;
    }

    public boolean isRotating() {
      return rotating;
    }

    public boolean isCalibrating() {
      return calibrating;
    }

    public boolean isConnected() {
      return connected;
    }

    public long getSensorTimestamp() {
      return sensorTimestamp;
    }

    public double getTemperatureC() {
      return temperatureC;
    }

    public double getPressure() {
      return pressure;
    }

    public double getAltitude() {
      return altitude;
    }

    public boolean isAltitudeValid() {
      return altitudeValid;
    }

    public String getFirmwareVersion() {
      return firmwareVersion;
    }
  }
}
