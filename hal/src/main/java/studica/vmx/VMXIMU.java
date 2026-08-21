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
  private long m_handle;

  /** Creates a wrapper over the shared VMX runtime AHRS. */
  public VMXIMU() {
    m_handle = create();
  }

  /** Returns whether the shared VMX AHRS is available and connected. */
  public synchronized boolean isAvailable() {
    return m_handle != 0 && getSnapshot().isConnected();
  }

  /** Reads one coherent vendor snapshot. */
  public synchronized Snapshot getSnapshot() {
    if (m_handle == 0) {
      return Snapshot.unavailable();
    }
    byte[] data = readSnapshot(m_handle);
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
    return m_handle != 0 && zeroYaw(m_handle) == 0;
  }

  private static native int zeroYaw(long handle);

  /** Resets the AHRS continuous angle/displacement state. */
  public synchronized boolean reset() {
    return m_handle != 0 && reset(m_handle) == 0;
  }

  private static native int reset(long handle);

  @Override
  public synchronized void close() {
    if (m_handle != 0) {
      destroy(m_handle);
      m_handle = 0;
    }
  }

  private static native long create();

  private static native void destroy(long handle);

  private static native byte[] readSnapshot(long handle);

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

    private final double m_yaw;
    private final double m_pitch;
    private final double m_roll;
    private final double m_accumulatedAngle;
    private final double m_yawRate;
    private final double m_quaternionW;
    private final double m_quaternionX;
    private final double m_quaternionY;
    private final double m_quaternionZ;
    private final double m_rawGyroX;
    private final double m_rawGyroY;
    private final double m_rawGyroZ;
    private final double m_rawAccelX;
    private final double m_rawAccelY;
    private final double m_rawAccelZ;
    private final double m_rawMagX;
    private final double m_rawMagY;
    private final double m_rawMagZ;
    private final double m_worldLinearAccelX;
    private final double m_worldLinearAccelY;
    private final double m_worldLinearAccelZ;
    private final double m_compassHeading;
    private final double m_fusedHeading;
    private final double m_temperatureC;
    private final double m_pressure;
    private final double m_altitude;
    private final long m_sensorTimestamp;
    private final boolean m_moving;
    private final boolean m_rotating;
    private final boolean m_calibrating;
    private final boolean m_connected;
    private final boolean m_altitudeValid;
    private final String m_firmwareVersion;

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
      m_yaw = yaw;
      m_pitch = pitch;
      m_roll = roll;
      m_accumulatedAngle = accumulatedAngle;
      m_yawRate = yawRate;
      m_quaternionW = quaternionW;
      m_quaternionX = quaternionX;
      m_quaternionY = quaternionY;
      m_quaternionZ = quaternionZ;
      m_rawGyroX = rawGyroX;
      m_rawGyroY = rawGyroY;
      m_rawGyroZ = rawGyroZ;
      m_rawAccelX = rawAccelX;
      m_rawAccelY = rawAccelY;
      m_rawAccelZ = rawAccelZ;
      m_rawMagX = rawMagX;
      m_rawMagY = rawMagY;
      m_rawMagZ = rawMagZ;
      m_worldLinearAccelX = worldLinearAccelX;
      m_worldLinearAccelY = worldLinearAccelY;
      m_worldLinearAccelZ = worldLinearAccelZ;
      m_compassHeading = compassHeading;
      m_fusedHeading = fusedHeading;
      m_temperatureC = temperatureC;
      m_pressure = pressure;
      m_altitude = altitude;
      m_sensorTimestamp = sensorTimestamp;
      m_moving = moving;
      m_rotating = rotating;
      m_calibrating = calibrating;
      m_connected = connected;
      m_altitudeValid = altitudeValid;
      m_firmwareVersion = firmwareVersion;
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
      return m_yaw;
    }

    public double getPitch() {
      return m_pitch;
    }

    public double getRoll() {
      return m_roll;
    }

    public double getAccumulatedAngle() {
      return m_accumulatedAngle;
    }

    public double getYawRate() {
      return m_yawRate;
    }

    public double getQuaternionW() {
      return m_quaternionW;
    }

    public double getQuaternionX() {
      return m_quaternionX;
    }

    public double getQuaternionY() {
      return m_quaternionY;
    }

    public double getQuaternionZ() {
      return m_quaternionZ;
    }

    public double getRawGyroX() {
      return m_rawGyroX;
    }

    public double getRawGyroY() {
      return m_rawGyroY;
    }

    public double getRawGyroZ() {
      return m_rawGyroZ;
    }

    public double getRawAccelX() {
      return m_rawAccelX;
    }

    public double getRawAccelY() {
      return m_rawAccelY;
    }

    public double getRawAccelZ() {
      return m_rawAccelZ;
    }

    public double getRawMagX() {
      return m_rawMagX;
    }

    public double getRawMagY() {
      return m_rawMagY;
    }

    public double getRawMagZ() {
      return m_rawMagZ;
    }

    public double getWorldLinearAccelX() {
      return m_worldLinearAccelX;
    }

    public double getWorldLinearAccelY() {
      return m_worldLinearAccelY;
    }

    public double getWorldLinearAccelZ() {
      return m_worldLinearAccelZ;
    }

    public double getCompassHeading() {
      return m_compassHeading;
    }

    public double getFusedHeading() {
      return m_fusedHeading;
    }

    public boolean isMoving() {
      return m_moving;
    }

    public boolean isRotating() {
      return m_rotating;
    }

    public boolean isCalibrating() {
      return m_calibrating;
    }

    public boolean isConnected() {
      return m_connected;
    }

    public long getSensorTimestamp() {
      return m_sensorTimestamp;
    }

    public double getTemperatureC() {
      return m_temperatureC;
    }

    public double getPressure() {
      return m_pressure;
    }

    public double getAltitude() {
      return m_altitude;
    }

    public boolean isAltitudeValid() {
      return m_altitudeValid;
    }

    public String getFirmwareVersion() {
      return m_firmwareVersion;
    }
  }
}
