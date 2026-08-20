# VMX sensor compatibility matrix

This matrix records the current compatibility boundary for the existing
WPILib sensor classes when the Linux aarch64 VMX HAL backend is selected with
`-PvmxBuild`. It is a HAL/API assessment, not a claim that every row has
already been verified on physical VMX hardware.

The same status applies to Java, C++, and Python users of the existing WPILib
APIs. They all reach the same VMX HAL adapter; no VMX-specific sensor API is
required for the rows in the WPILib section.

## Status definitions

| Status | Meaning |
| --- | --- |
| `SUPPORTED` | The required HAL path exists and no known API blocker remains. A physical hardware smoke test may still be pending. |
| `PARTIAL` | The HAL path exists, but one or more WPILib modes, sources, or options are intentionally incomplete. |
| `BLOCKED_BY_HAL` | The sensor's required HAL primitive is missing or explicitly reports an incompatible state in the VMX backend. |
| `UNSUPPORTED` | Compatibility is deliberately excluded; this is different from a feature that is merely not implemented yet. |
| `VENDOR_API` | Use a separate Studica/VMX vendor wrapper rather than a WPILib core sensor class. |
| `NOT_TESTED` | The HAL path is plausible, but a sensor-level compatibility or hardware test has not been completed. |

`UNSUPPORTED` is intentionally not used as a synonym for “not implemented” in
this snapshot. Missing WPILib primitives are `BLOCKED_BY_HAL`, and paths that
are available but still awaiting sensor verification are `NOT_TESTED`.

## WPILib sensor classes

| WPILib sensor/class | Status | Current HAL path | Evidence and next validation |
| --- | --- | --- | --- |
| DigitalInput / limit switch | `NOT_TESTED` | Input-capable VMX DIO | Logical DIO 0–29 maps through the capability layer; selected FlexDIO/HighCurrent/CommDIO input capability and hardware smoke test remain. |
| Beam Break / digital sensor | `NOT_TESTED` | Input-capable VMX DIO | Same input-capability and interrupt/resource constraints as a limit switch. |
| AnalogInput | `SUPPORTED` | VMX AnalogInput / AccumulatorInput | Logical analog channels 0–3, voltage conversion, averaging, and resource lifecycle are implemented and covered by native tests. |
| AnalogPotentiometer | `NOT_TESTED` | Existing WPILib class → AnalogInput | No additional HAL is required. Add a compatibility test for voltage-to-position scaling and channel lifetime. |
| AnalogAccelerometer | `NOT_TESTED` | Existing WPILib class → AnalogInput average voltage | No additional HAL is required. Add a compatibility test for zero, sensitivity, and units. |
| AnalogEncoder | `NOT_TESTED` | Existing WPILib class → AnalogInput voltage | No additional HAL is required. Add a compatibility test for 5 V normalization, range mapping, and inversion. |
| SharpIR (WPILib) | `NOT_TESTED` | Existing WPILib class → AnalogInput voltage | No additional HAL is required. Add a compatibility test for the published voltage-to-distance curves and clamping. |
| Encoder | `NOT_TESTED` | VMX Encoder + valid DIO pair | Only FRC pairs 0+1, 2+3, 4+5, 6+7, and 8+9 are accepted; invalid pairs are rejected before activation. |
| DutyCycle | `SUPPORTED` | FlexDIO 0-11 + VMX PWMCapture | Existing `hal/DutyCycle.h` ABI is implemented with period/high-time conversion, DIO ownership handoff, shared FlexTimer groups, and stable logical FPGA indices. AnalogTrigger, HighCurrent, and CommDIO sources are explicitly rejected. Hardware smoke test remains. |
| DutyCycleEncoder | `NOT_TESTED` | Existing WPILib class + DutyCycle HAL | The standard class now has its required HAL path. Validate frequency threshold, connected/disconnected behavior, offset/scaling, and selected FlexDIO hardware on VMX. |
| DutyCycleInput | `NOT_TESTED` | Existing WPILib class + DutyCycle HAL | The shared HAL path is present; add a sensor-level test for the WPILib wrapper and source lifetime. |
| Tachometer | `NOT_TESTED` | Counter single-source InputCapture | Single-source hardware-backed activation is available; sensor-level period validation remains. |
| Counter | `PARTIAL` | VMX InputCapture Counter + DIO sources | Six official pairs are recognized. TwoPulse accepts one source configured as both Up and Down and rejects independent Up/Down handles; SemiPeriod is hardware-backed; ExternalDirection is limited to pairs 0+1 through 8+9; PulseLength remains unsupported. |
| Ultrasonic | `NOT_TESTED` | Output-capable DIO ping + Counter SemiPeriod echo | DIO output capability and single-source SemiPeriod must both be available on the selected physical channels; add a hardware smoke test. |
| PWM output / PWMVictorSPX | `NOT_TESTED` | PWMGenerator-capable logical PWM | Logical PWM 0–27 maps to physical 0–21 and 26–31; SDK output capability and shared resource ownership are enforced. |
| ADXL345_I2C | `SUPPORTED` | WPILib `I2C` → VMX MXP I2C HAL | Standard WPILib register transactions now reach the shared VMX I2C resource. The ADXL345 WHO_AM_I and one-register read/write sequence are the first sensor-level I2C smoke test. |
| ADXL345_SPI | `BLOCKED_BY_HAL` | Required SPI HAL is not present in the VMX backend | Schedule after SPI Core; use the existing WPILib driver unchanged. |
| ADXL362 | `BLOCKED_BY_HAL` | Required SPI HAL is not present in the VMX backend | Schedule after SPI Core; no VMX-specific ADXL362 wrapper is needed. |
| ADXRS450_Gyro | `BLOCKED_BY_HAL` | Required SPI HAL is not present in the VMX backend | Schedule after SPI Core; preserve the existing WPILib driver and calibration semantics. |
| ADIS16448_IMU | `BLOCKED_BY_HAL` | Required SPI HAL is not present in the VMX backend | Schedule after SPI Core; verify reset, initialization, and update timing on hardware. |
| ADIS16470_IMU | `BLOCKED_BY_HAL` | Required SPI HAL is not present in the VMX backend | Schedule after SPI Core; verify reset, initialization, and update timing on hardware. |
| AnalogGyro | `BLOCKED_BY_HAL` | Required AnalogGyro HAL is not present in the VMX backend | AnalogInput and accumulator support the underlying ADC path, but the WPILib class calls the separate AnalogGyro HAL ABI for angle/rate/calibration. Add the compatibility test after that HAL is implemented. |
| BuiltInAccelerometer | `BLOCKED_BY_HAL` | Required accelerometer HAL is not present in the VMX backend | VMX onboard IMU data may be usable, but range, axis orientation, calibration, and g-vs-m/s² units must be verified before exposing `HAL_GetAccelerometerX/Y/Z`. |

## Current VMX HAL core snapshot

| HAL/core | Status | Scope |
| --- | --- | --- |
| DIO | `SUPPORTED` | Input/output allocation, reads/writes, pulse generation, and ownership coordination for VMX digital channels. |
| PWM | `SUPPORTED` | WPILib pulse/speed/position semantics, quantized readback, disable, and reactivation. Unsupported optional modes remain explicit HAL errors. |
| AnalogInput | `SUPPORTED` | Four logical analog channels backed by the VMX AccumulatorInput resource. |
| AnalogAccumulator | `SUPPORTED` | Standard accumulator surface on logical channels 0 and 1, including atomic value/count output and continuity offsets. |
| Counter | `PARTIAL` | TwoPulse, single-source SemiPeriod, and restricted ExternalDirection InputCapture paths; PulseLength remains unsupported. |
| Encoder | `NOT_TESTED` | Quadrature encoder path with official pair validation and DIO index reset source. |
| Interrupt | `PARTIAL` | DIO interrupt sources and VMX timestamps are available; AnalogTrigger sources remain blocked. |
| AnalogTrigger | `PARTIAL` | AnalogInput-backed raw/voltage trigger state and window semantics are available; filtered mode, pulse outputs, and DutyCycle sources are blocked. |
| Timing / Notifier | `SUPPORTED` | VMX monotonic time and one global VMX timer-notification scheduler. |
| I2C | `SUPPORTED` | One VMX MXP bus, shared and reference-counted; onboard I2C is intentionally unsupported. |
| DutyCycle | `SUPPORTED` | FlexDIO 0-11 VMX PWMCapture adapter with shared timer-group ownership and DIO handoff. |
| SPI | `BLOCKED_BY_HAL` | Planned before SPI-based WPILib sensors. |
| Serial / UART | `BLOCKED_BY_HAL` | Planned communication core. |

## Studica / VMX vendor sensors

These are not substitutes for WPILib core sensor compatibility. They should
be exposed, if desired, through a separate shared native vendor wrapper over
the unchanged `vmxdrivers` sources and then bound to Java, C++, and Python.

| Vendor sensor/module | Status | Boundary |
| --- | --- | --- |
| VMX onboard IMU / NavX-style data | `VENDOR_API` | `vmxdrivers::Imu` exposes orientation, acceleration, gyro, magnetometer, and related values. Do not label it as WPILib `BuiltInAccelerometer` or `AnalogGyro` until the corresponding HAL semantics are verified. |
| Studica Cobra | `VENDOR_API` | Use the Studica driver API through a separate adapter. |
| Studica Sharp | `VENDOR_API` | Distinct from the WPILib `SharpIR` analog class; use the Studica driver API. |
| Studica Parsec | `VENDOR_API` | Use the Studica driver API through a separate adapter. |
| Studica Colore | `VENDOR_API` | Use the Studica driver API through a separate adapter. |

`vmxdrivers/` remains an upstream-source area. It must not receive WPILib
types, HAL status handling, compatibility shims, or sensor-specific fixes.
Those responsibilities belong in `hal/src/main/native/vmx` or in a future
separate vendor wrapper module.

## Validation order

1. Add the AnalogPotentiometer, AnalogAccelerometer, AnalogEncoder, SharpIR,
   AnalogGyro, Ultrasonic, and ADXL345_I2C compatibility tests without
   changing their public APIs.
2. Validate DutyCycle, DutyCycleEncoder, and DutyCycleInput on physical VMX
   hardware.
3. Implement SPI Core before validating ADXL345_SPI, ADXL362, ADXRS450_Gyro,
   ADIS16448_IMU, and ADIS16470_IMU.
4. Close the Counter single-source/semiperiod gap before validating
   Tachometer and Ultrasonic.
5. Verify VMX IMU mappings separately before deciding whether any
   BuiltInAccelerometer compatibility claim is justified.
