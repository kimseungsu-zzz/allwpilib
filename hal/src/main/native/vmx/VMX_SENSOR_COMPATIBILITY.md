# VMX sensor compatibility matrix

This matrix records the current compatibility boundary for the existing
WPILib sensor classes when the VMX HAL backend is selected with `-PvmxBuild`.
VMX-Pi and VMX2 deployment targets are Linux AArch64 only. The locally
available `libvmxpi_hal_cpp.so` is an ELF32 ARM EABI5 legacy/incompatible SDK
artifact; the build rejects it and provides no armhf/32-bit fallback, helper
daemon, IPC bridge, or forced ELF32 link. Linux ARM64 source/header checks can
run without that artifact, but a final HAL link requires a matching ELF64
AArch64 SDK. It is a HAL/API assessment, not a claim that every row has
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

`UNSUPPORTED` is intentionally not used as a synonym for ?쐍ot implemented??in
this snapshot. Missing WPILib primitives are `BLOCKED_BY_HAL`, and paths that
are available but still awaiting sensor verification are `NOT_TESTED`.

For this milestone, `SUPPORTED` means the WPILib HAL/class dependency path is
covered by a host/mock contract test and `hardwareValidated=false`. A row is
changed to `hardwareValidated=true` only after the same WPILib class is run on
physical VMX-Pi or VMX2 hardware; no such claim is made below.

## WPILib sensor classes

Current milestone overrides: AddressableLED and BuiltInAccelerometer are
`SUPPORTED` with `hardwareValidated=false`; their host/mock adapter contracts
are exercised by `VMXSensorIntegrationTest.cpp`.

This milestone adds a separate VMX onboard-IMU vendor contract. It is not a
WPILib sensor-row promotion: BuiltInAccelerometer continues to use the
standard HAL subset and the vendor wrapper does not duplicate it. The native
snapshot ABI and C++ wrapper are covered by `VMXIMUTest.cpp`; Java JNI uses the
same fixed layout and the Python-facing boundary is the exported C ABI. A
physical AHRS smoke test is still required before setting
`hardwareValidated=true`.

The Titan milestone adds a separate Studica vendor contract. It is not a
WPILib motor-controller or encoder-row promotion. Native host tests cover ABI
validation and unavailable-hardware behavior; Java JNI/C++ facades use the
same C ABI, while the VMX adapter uses one shared imported Titan object per CAN
ID and a 50 ms keepalive worker. Physical board validation remains separate.

| WPILib sensor/class | Status | Current HAL path | Evidence and next validation |
| --- | --- | --- | --- |
| DigitalInput / limit switch | `NOT_TESTED` | Input-capable VMX DIO | Logical DIO 0??9 maps through the capability layer; selected FlexDIO/HighCurrent/CommDIO input capability and hardware smoke test remain. |
| Beam Break / digital sensor | `NOT_TESTED` | Input-capable VMX DIO | Same input-capability and interrupt/resource constraints as a limit switch. |
| AnalogInput | `SUPPORTED` | VMX AnalogInput / AccumulatorInput | Logical analog channels 0??, voltage conversion, averaging, and resource lifecycle are implemented and covered by native tests. |
| AnalogPotentiometer | `NOT_TESTED` | Existing WPILib class ??AnalogInput | No additional HAL is required. Add a compatibility test for voltage-to-position scaling and channel lifetime. |
| AnalogAccelerometer | `NOT_TESTED` | Existing WPILib class ??AnalogInput average voltage | No additional HAL is required. Add a compatibility test for zero, sensitivity, and units. |
| AnalogEncoder | `NOT_TESTED` | Existing WPILib class ??AnalogInput voltage | No additional HAL is required. Add a compatibility test for 5 V normalization, range mapping, and inversion. |
| SharpIR (WPILib) | `NOT_TESTED` | Existing WPILib class ??AnalogInput voltage | No additional HAL is required. Add a compatibility test for the published voltage-to-distance curves and clamping. |
| Encoder | `NOT_TESTED` | VMX Encoder + valid DIO pair | Only FRC pairs 0+1, 2+3, 4+5, 6+7, and 8+9 are accepted; invalid pairs are rejected before activation. |
| DutyCycle | `SUPPORTED` | FlexDIO 0-11 + VMX PWMCapture | Existing `hal/DutyCycle.h` ABI is implemented with period/high-time conversion, DIO ownership handoff, shared FlexTimer groups, and stable logical FPGA indices. AnalogTrigger, HighCurrent, and CommDIO sources are explicitly rejected. Hardware smoke test remains. |
| DutyCycleEncoder | `NOT_TESTED` | Existing WPILib class + DutyCycle HAL | The standard class now has its required HAL path. Validate frequency threshold, connected/disconnected behavior, offset/scaling, and selected FlexDIO hardware on VMX. |
| DutyCycleInput | `NOT_TESTED` | Existing WPILib class + DutyCycle HAL | The shared HAL path is present; add a sensor-level test for the WPILib wrapper and source lifetime. |
| Tachometer | `NOT_TESTED` | Counter single-source InputCapture | Single-source hardware-backed activation is available; sensor-level period validation remains. |
| Counter | `PARTIAL` | VMX InputCapture Counter + DIO sources | Six official pairs are recognized. TwoPulse supports the shared VMX up/down capture resource for a valid pair (and the same source for both inputs); SemiPeriod is hardware-backed; ExternalDirection is limited to pairs 0+1 through 8+9; PulseLength remains unsupported. |
| Ultrasonic | `NOT_TESTED` | Output-capable DIO ping + Counter SemiPeriod echo | DIO output capability and single-source SemiPeriod must both be available on the selected physical channels; add a hardware smoke test. |
| PWM output / PWMVictorSPX | `NOT_TESTED` | PWMGenerator-capable logical PWM | Logical PWM 0??7 maps to physical 0??1 and 26??1; SDK output capability and shared resource ownership are enforced. |
| ADXL345_I2C | `SUPPORTED` | WPILib `I2C` ??VMX MXP I2C HAL | Standard WPILib register transactions now reach the shared VMX I2C resource. The ADXL345 WHO_AM_I and one-register read/write sequence are the first sensor-level I2C smoke test. |
| ADXL345_SPI | `NOT_TESTED` | Basic SPI transactions on VMX MXP | The WPILib path uses mode 3, active-high CS, and 500 kHz; the VMX basic path can represent these settings, but construction/configuration and WHO_AM_I data have not been tested on hardware. |
| ADXL362 | `NOT_TESTED` | Basic SPI transactions on VMX SPI connector | The default WPILib constructor selects onboard CS1, which aliases the VMX SPI connector; the basic mode-3, active-low transaction path still needs sensor-level validation. |
| ADXRS450_Gyro | `NOT_TESTED` | Basic SPI plus HAL-owned AutoSPI accumulator | Onboard CS0 aliases the VMX SPI connector and the standard transaction plus `SPI::InitAccumulator` path is now available. Sensor-level construction/device-ID and accumulator integration testing remains. |
| ADIS16448_IMU | `BLOCKED_BY_HAL` | MXP basic SPI, SPI Auto, DIO reset/status pins | The default MXP bus is available for register setup and rate/trigger AutoSPI exists, but the class requires precise `ConfigureAutoStall` semantics, an asynchronous acquire loop, and DIO 18/19/10 resources. Stall timing is the HAL blocker. |
| ADIS16470_IMU | `BLOCKED_BY_HAL` | Onboard-CS0 basic SPI, SPI Auto, DIO reset/data-ready/LED pins | Rate/trigger AutoSPI exists, but the class requires precise `ConfigureAutoStall`; its roboRIO-fixed DIO 27 reset, 28 status LED, and 26 data-ready routing also conflicts with VMX CommDIO/SPI physical resources. |
| AnalogGyro | `NOT_TESTED` | Existing AnalogInput + AccumulatorInput resource through the VMX AnalogGyro HAL | The complete AnalogGyro C ABI is implemented for logical channels 0 and 1 with fixed 46.5 kS/s scaling, injectable five-second calibration, center/offset, deadband, reset, angle, and rate paths. Sensor-level construction and physical validation remain. |
| AddressableLED | `SUPPORTED` | VMX `LEDArray_OneWire` resource through the standard AddressableLED C ABI | Color order, length/data lifecycle, strict bit-timing conversion, start/stop/render, PWM-handle handoff, and physical registry conflicts are covered by `VMXSensorIntegrationTest`; hardwareValidated=false. |
| BuiltInAccelerometer | `SUPPORTED` | VMX AHRS `GetRawAccelX/Y/Z()` through the standard accelerometer C ABI | Raw calibrated sensor-frame values in G (gravity included) use identity X/Y/Z mapping. Active state is HAL-local; requested range is retained but VMX's fixed hardware range is not misreported as changed. Host/mock coverage exists; hardwareValidated=false. |

## Class-level integration closure audit

The WPILib Java classes listed below have existing class-level simulation tests
in `wpilibj/src/test/java` that validate public calculations and lifecycle.
Those tests intentionally exercise WPILib's simulation HAL and are not
counted as VMX evidence. The VMX adapter tests in `hal/src/test/native/cpp`
cover the native resource and mock-SDK side separately; a row is promoted to
`SUPPORTED` only when both sides are connected by a VMX-targeted class test.
This keeps the matrix honest while the AArch64 SDK and board smoke fixtures
are unavailable.

| Class-level audit target | Current result | Remaining VMX evidence |
| --- | --- | --- |
| DigitalInput / limit switch | `NOT_TESTED` | VMX HAL class test with a mocked DIO input and physical capability claim. |
| AnalogPotentiometer / AnalogAccelerometer / AnalogEncoder / SharpIR | `NOT_TESTED` | VMX-targeted Java class test must feed mock AnalogInput values through the selected native backend. |
| Encoder / DutyCycleEncoder / DutyCycleInput / Tachometer | `NOT_TESTED` | VMX-targeted class construction, readback, and close test; native pair/timer tests already exist. |
| Ultrasonic | `NOT_TESTED` | VMX-targeted ping/echo class test with mocked DIO/period capture. |
| ADXL345_I2C / ADXL345_SPI / ADXL362 / ADXRS450_Gyro | `NOT_TESTED` | Device-register mock plus VMX Java class path; ADXRS additionally needs AutoSPI accumulator validation. |
| AnalogGyro / BuiltInAccelerometer / AddressableLED | `NOT_TESTED` / `SUPPORTED` / `SUPPORTED` | AnalogGyro still needs class-level VMX test; the latter two already have native host/mock contracts but remain hardwareValidated=false. |

## WPILib CAN hardware dependency audit

These rows record the existing WPILib vendor-device dependency on CANAPI. A
working CAN HAL is not evidence that a particular physical device has been
validated; each remains pending frame-level and hardware integration tests.

| WPILib device class | Status | Current dependency | Next validation |
| --- | --- | --- | --- |
| CTRE PCM | `NOT_TESTED` | Existing `CTREPCM` implementation over logical CANAPI handle and timeout reads | Construct against a VMX CAN bus and validate compressor/solenoid frame decoding on hardware. |
| REV Pneumatics Hub | `NOT_TESTED` | Existing `REVPH` implementation over logical CANAPI handle and timeout reads | Validate device discovery, solenoid/compressor status, and frame timing on hardware. |
| PDP | `NOT_TESTED` | Existing `CTREPDP` implementation over CANAPI and stream sessions | Validate status/energy stream frames and current/voltage decoding on hardware. |
| PDH | `NOT_TESTED` | Existing `REVPDH` implementation over CANAPI and stream sessions | Validate status/current/energy stream frames and sticky-fault behavior on hardware. |

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
| DriverStation | `PARTIAL` | KauaiLabs VMX-pi UDP v1 control/joystick transport on 1110 plus TCP metadata on 1740, snapshot state, timeout/malformed-packet failsafe, eStop output interlock, refresh generation, events, modes, match info, and output peer. Local error/console output is supported; full physical DS smoke and outbound TCP message validation remain. |
| RobotController power/status | `PARTIAL` | VMX system voltage, Linux thermal sysfs, runtime readiness, and wall-clock validity | Vin voltage is hardware-backed and CPU temperature uses a discovered CPU-like thermal zone. Current, user rails, fault reset, brownout/undervoltage, FPGA button, RSL, and Driver Station disable count are explicit `INCOMPATIBLE_STATE` results because the SDK has no equivalent APIs; VMX overcurrent is not brownout. RTC bootstrap is a separate best-effort runtime service; monotonic FPGA time is unchanged. |
| CAN / raw CAN C ABI | `SUPPORTED` | One global VMX CAN bus, one wildcard SDK receive stream, adapter-owned software streams, one-shot/periodic/cancel TX, masked RX, 11/29-bit and RTR flags, hardware timestamps, and hardware status mapping | VMX physical CAN loopback and device-bus smoke tests remain deployment validation. |
| CANAPI | `SUPPORTED` | FIRST-layout logical device handles, packet write/repeat/RTR/stop, New/Latest/Timeout generation and age semantics, and stream cleanup | CTRE/REV/PDP/PDH frame-level integration remains pending. |
| Hardware Watchdog | `SUPPORTED` | VMXIO 100 ms watchdog; FlexDIO and HighCurrentDIO managed, CommDIO unmanaged; DS freshness, program heartbeat, runtime health, eStop, and shutdown failsafe gate | Physical actuator-output disable and recovery test on VMX-Pi/VMX2 remain pending. |
| I2C | `SUPPORTED` | One VMX physical bus at 32/33, shared and reference-counted by the kOnboard/kMXP aliases. |
| AddressableLED | `SUPPORTED` | One VMX LEDArray_OneWire resource selected through the central channel capability and physical registry layers; the WPILib PWM handle is suspended and restored around LED ownership. |
| BuiltInAccelerometer | `SUPPORTED` | Shared VMX AHRS raw acceleration reader; no separate IMU allocation or AHRS shutdown is performed for standby. |
| DutyCycle | `SUPPORTED` | FlexDIO 0-11 VMX PWMCapture adapter with shared timer-group ownership and DIO handoff. |
| SPI | `NOT_TESTED` | One physical CommDIO SPI bus at 28/29/30/31; `HAL_InitializeSPI`, transaction/write/read, clock, mode 0-3, and CS polarity use one shared resource. All five WPILib port names alias it. HAL-owned AutoSPI rate and DIO trigger (rising/falling/both) use timestamped fixed-capacity buffering; AnalogTrigger sources and exact AutoStall timing are unsupported. Sensor-level integration and physical validation remain. |
| Serial / UART | `PARTIAL` | WPILib `kMXP` ??VMX TTL UART at 26/27 | Baud 0..230400, blocking read/write, default 8-N-1/no-flow configuration, software chunk limits for read/write buffers, timeout, termination, blocking flush, and receive clear are adapter-backed. Non-default data/parity/stop/flow settings, raw FD, onboard RS-232, and USB ports are explicit incompatible-state results. |

## Studica / VMX vendor sensors

These are not substitutes for WPILib core sensor compatibility. They are
exposed, if desired, through separate shared native vendor wrappers over the
unchanged `vmxdrivers` sources and then bound to Java, C++, and Python.

| Vendor sensor/module | Status | Boundary |
| --- | --- | --- |
| VMX onboard IMU / NavX-style data | `VENDOR_API` | `studica::VMXIMU` reads the shared VMXRuntime AHRS and exposes orientation, continuous angle/rate, quaternion, raw/world acceleration, gyro/magnetometer, heading, state, timestamp, temperature, firmware, and validity-guarded pressure/altitude. `hardwareValidated=false`; it is not WPILib `BuiltInAccelerometer` or `AnalogGyro`. |
| Titan Quad / Titan Quad Encoder | `VENDOR_API` | `studica::TitanQuad`, `studica::TitanQuadEncoder`, Java `com.studica.frc` wrappers, and `StudicaTitan_*` C ABI. CAN IDs 1--62, motor ports 0--3, shared controller per CAN ID, 50 ms watchdog refresh, motor/encoder/absolute-angle/limit/configuration paths, and explicit safe zero/disable on stale DS/e-stop/close. `hardwareValidated=false`; this does not promote WPILib motor-controller, Encoder, or DutyCycleEncoder rows. |
| Studica Cobra | `VENDOR_API` | Use the Studica driver API through a separate adapter. |
| Studica Sharp | `VENDOR_API` | Distinct from the WPILib `SharpIR` analog class; use the Studica driver API. |
| Studica Parsec | `VENDOR_API` | Use the Studica driver API through a separate adapter. |
| Studica Colore | `VENDOR_API` | Use the Studica driver API through a separate adapter. |
| Studica Light Tower | `VENDOR_API` | Future separate adapter; do not add product-specific fields to WPILib HAL. |
| VMX-specific Power/SOC | `VENDOR_API` | Future capability-gated telemetry wrapper; standard RobotController semantics remain separate. |

`vmxdrivers/` remains an upstream-source area. It must not receive WPILib
types, HAL status handling, compatibility shims, or sensor-specific fixes.
Those responsibilities belong in `hal/src/main/native/vmx` or in a future
separate vendor wrapper module.

## Validation order

1. Run the VMX-targeted class-level mock suite for the remaining `NOT_TESTED`
   rows (AnalogPotentiometer, AnalogAccelerometer, AnalogEncoder, SharpIR,
   AnalogGyro, Ultrasonic, and ADXL345_I2C first) without changing public
   APIs; simulation-only tests do not promote a row.
2. Validate DutyCycle, DutyCycleEncoder, and DutyCycleInput on physical VMX
   hardware.
3. Validate the basic SPI sensor paths for ADXL345_SPI and ADXL362, then run
   the ADXRS450_Gyro standard-SPI plus AutoSPI accumulator integration path.
   ADIS16448_IMU and ADIS16470_IMU remain blocked until precise AutoStall and
   their additional DIO routing requirements are resolved.
4. Run a real VMX-Pi/VMX2 Driver Station smoke test: UDP control-word modes,
   timeout/reconnect, all six joystick slots, TCP descriptors/match/game data,
   new-data events, observe-program heartbeats, and rumble/output. Keep the
   local-console-only error/console boundary explicit until outbound TCP
   encoding is validated.
5. Validate VMX CAN loopback and then CTRE PCM, REV PH, PDP, and PDH frame
   integration without changing their public WPILib APIs.
6. Validate hardware watchdog output disable/recovery with fresh/stale DS data,
   program heartbeat timeout, eStop, and HAL shutdown.
7. Close the Counter single-source/semiperiod gap before validating
   Tachometer and Ultrasonic.
8. Run physical AddressableLED and BuiltInAccelerometer class smoke tests and
   promote their separate `hardwareValidated` field only after those tests.
9. Implement vendor adapters in order: Cobra, Parsec/Colore, and Light Tower,
   with VMX-specific Power/SOC as a capability-gated API; Titan is complete
   at the native/Java/Python-ABI layer and awaits physical board validation.
