# VMX HAL 2026 ABI coverage

This is the VMX backend's machine-checked coverage manifest for the public
WPILib HAL headers. `hal/tools/verify_hal_coverage.py` scans every production
header (including the generated `FRCUsageReporting.h`), assigns the header's
default status to every declared `HAL_*` function, and applies the explicit
symbol overrides below. A new public declaration without a row fails the
`hal:checkHalCoverage` task. An `IMPLEMENTED` declaration must also have a
definition in `hal/src/main/native/vmx` or `hal/src/main/native/shared`.

The statuses are deliberately about the VMX HAL contract, not physical-board
validation:

All software classifications in this manifest keep `hardwareValidated=false`
until the corresponding WPILib class is run on physical VMX-Pi/VMX2 hardware.
The VMX deployment target is Linux AArch64 only; no ELF32/ARM32 fallback is
represented by a coverage status.

| Status | Meaning |
| --- | --- |
| `IMPLEMENTED` | VMX/shared implementation exists and is exercised by HAL tests. |
| `PARTIAL` | The header is available, but one or more modes/options are incomplete. |
| `MISSING_FEASIBLE` | No VMX implementation yet; a future adapter is plausible. |
| `UNSUPPORTED_HARDWARE` | The current VMX SDK/hardware has no honest equivalent. |
| `NOT_APPLICABLE` | RoboRIO/FPGA/simulation-only surface outside VMX production scope. |
| `HARDWARE_VALIDATION_REQUIRED` | Software contract exists; a physical VMX test is still required. |

Status describes the hardware. Linkage is a separate question, and conflating
the two hid real breakage: a symbol can be honestly `UNSUPPORTED_HARDWARE` and
still have to exist, because the HAL C ABI is what wpilibc and the JNI layer
link against. `HAL_GetErrorMessage` was marked unsupported and simply absent,
which is not a missing feature but a link error.

| Linkage | Meaning |
| --- | --- |
| `PROVIDED` | The VMX `wpiHal` defines every symbol in this header. An unsupported one still resolves, returning an error or empty result. |
| `ABSENT` | The VMX `wpiHal` defines none of them. Any consumer referencing one fails to link. |

`hal/tools/verify_vmx_crosscompile.py` enforces this column against the object
symbol tables it builds, in both directions: a `PROVIDED` symbol that nothing
defines, and an `ABSENT` symbol that something does, are each a failure. Only
that gate can check it honestly -- a text search cannot tell a definition from
a forward declaration, which is precisely how the absences went unnoticed.

The checker is intentionally strict about manifest completeness. It does not
turn a `PARTIAL` or `HARDWARE_VALIDATION_REQUIRED` row into a support claim.

| Header | Default status | Linkage | Symbol overrides | Notes |
| --- | --- | --- | --- | --- |
| `Accelerometer.h` | `IMPLEMENTED` | `PROVIDED` | - | Standard accelerometer subset is VMX AHRS-backed. |
| `AddressableLED.h` | `IMPLEMENTED` | `PROVIDED` | - | VMX LEDArray_OneWire adapter. |
| `AnalogAccumulator.h` | `IMPLEMENTED` | `PROVIDED` | - | Atomic value/count and continuity offsets. |
| `AnalogGyro.h` | `IMPLEMENTED` | `PROVIDED` | - | AnalogInput/AccumulatorInput-backed implementation. |
| `AnalogInput.h` | `IMPLEMENTED` | `PROVIDED` | - | Four logical analog channels. |
| `AnalogOutput.h` | `UNSUPPORTED_HARDWARE` | `ABSENT` | - | No VMX analog-output resource. |
| `AnalogTrigger.h` | `PARTIAL` | `PROVIDED` | - | Raw/window state works; filtered/pulse modes report unsupported. |
| `CAN.h` | `IMPLEMENTED` | `PROVIDED` | - | VMX CANAPI backend. |
| `CANAPI.h` | `IMPLEMENTED` | `PROVIDED` | - | Logical device and stream sessions over one bus. |
| `Constants.h` | `IMPLEMENTED` | `PROVIDED` | - | VMX monotonic time is expressed in microseconds, so the legacy tick conversion is one tick per microsecond. |
| `Counter.h` | `PARTIAL` | `PROVIDED` | HAL_SetCounterPulseLengthMode=UNSUPPORTED_HARDWARE | InputCapture-backed modes; pulse-length semantics are not equivalent. |
| `CTREPCM.h` | `IMPLEMENTED` | `PROVIDED` | - | Shared canonical CANAPI implementation. |
| `DIO.h` | `PARTIAL` | `PROVIDED` | HAL_SetDigitalPWMPPS=UNSUPPORTED_HARDWARE; HAL_SetFilterSelect=UNSUPPORTED_HARDWARE; HAL_GetFilterSelect=UNSUPPORTED_HARDWARE; HAL_SetFilterPeriod=UNSUPPORTED_HARDWARE; HAL_GetFilterPeriod=UNSUPPORTED_HARDWARE | Normal DIO, pulse, output, and general DigitalPWM paths use the VMX PWMGenerator resource. PPS and glitch filtering remain deliberate boundaries. |
| `DMA.h` | `UNSUPPORTED_HARDWARE` | `ABSENT` | - | FPGA DMA has no VMX SDK equivalent. |
| `DriverStation.h` | `PARTIAL` | `PROVIDED` | - | VMX KauaiLabs-compatible transport/state adapter. |
| `DutyCycle.h` | `IMPLEMENTED` | `PROVIDED` | - | FlexDIO PWMCapture adapter. |
| `Encoder.h` | `IMPLEMENTED` | `PROVIDED` | - | Valid-pair, index-source, and readback paths. |
| `Extensions.h` | `NOT_APPLICABLE` | `ABSENT` | - | roboRIO extension-loader surface. |
| `I2C.h` | `IMPLEMENTED` | `PROVIDED` | - | Shared CommDIO I2C resource. |
| `Interrupts.h` | `IMPLEMENTED` | `PROVIDED` | - | DIO edge interrupts and VMX timestamps. |
| `LEDs.h` | `UNSUPPORTED_HARDWARE` | `ABSENT` | - | roboRIO radio/RSL LED surface has no VMX equivalent. |
| `Main.h` | `NOT_APPLICABLE` | `PROVIDED` | - | roboRIO main-loop launcher surface. |
| `Notifier.h` | `IMPLEMENTED` | `PROVIDED` | - | One global VMX timer-notification scheduler. |
| `Ports.h` | `IMPLEMENTED` | `PROVIDED` | - | Static VMX logical port counts are exposed; zero counts mark unsupported relay/analog-output surfaces. |
| `Power.h` | `PARTIAL` | `PROVIDED` | - | VMX voltage/thermal subset; FPGA-only rails are explicit errors. |
| `PowerDistribution.h` | `IMPLEMENTED` | `PROVIDED` | - | Shared CTREPDP/REVPDH CANAPI implementation. |
| `PWM.h` | `PARTIAL` | `PROVIDED` | HAL_LatchPWMZero=UNSUPPORTED_HARDWARE; HAL_SetPWMPeriodScale=UNSUPPORTED_HARDWARE; HAL_SetPWMAlwaysHighMode=UNSUPPORTED_HARDWARE; HAL_GetPWMLoopTiming=UNSUPPORTED_HARDWARE; HAL_GetPWMCycleStartTime=UNSUPPORTED_HARDWARE | Normal PWM, Servo, and PWMVictorSPX paths are implemented. |
| `Relay.h` | `UNSUPPORTED_HARDWARE` | `ABSENT` | - | VMX has no relay hardware. |
| `REVPH.h` | `IMPLEMENTED` | `PROVIDED` | - | Shared REV PH CANAPI implementation. |
| `SerialPort.h` | `PARTIAL` | `PROVIDED` | - | kMXP TTL UART works; RS-232/USB aliases remain explicit gaps. |
| `SimDevice.h` | `NOT_APPLICABLE` | `ABSENT` | - | Simulation-only API is not a VMX production resource. |
| `SPI.h` | `PARTIAL` | `PROVIDED` | - | Basic SPI and HAL-owned AutoSPI; exact AutoStall remains unsupported. |
| `Threads.h` | `IMPLEMENTED` | `PROVIDED` | - | VMX thread priority/interruptible sleep adapter. |
| `Value.h` | `NOT_APPLICABLE` | `ABSENT` | - | Simulation value helper types. |
| `FRCUsageReporting.h` | `IMPLEMENTED` | `PROVIDED` | - | `HAL_Report` is a no-op-compatible VMX implementation. |
| `HAL.h` | `NOT_APPLICABLE` | `PROVIDED` | - | Umbrella include; declarations are covered by their owning headers. |
| `HALBase.h` | `PARTIAL` | `PROVIDED` | HAL_GetComments=UNSUPPORTED_HARDWARE; HAL_GetErrorMessage=UNSUPPORTED_HARDWARE; HAL_GetFPGARevision=UNSUPPORTED_HARDWARE; HAL_GetFPGAVersion=UNSUPPORTED_HARDWARE; HAL_GetLastError=UNSUPPORTED_HARDWARE; HAL_GetSerialNumber=UNSUPPORTED_HARDWARE; HAL_GetTeamNumber=UNSUPPORTED_HARDWARE | VMX lifecycle/time/runtime-type subset is implemented; roboRIO identity metadata has no SDK equivalent. |
| `UsageReporting.h` | `NOT_APPLICABLE` | `PROVIDED` | - | Legacy generated usage-reporting declarations/types are not a VMX device resource. |

The current machine count is emitted by the checker rather than copied into
this document. Run:

```text
./gradlew :hal:checkHalCoverage
```

The current audit is 38 headers / 502 symbols with
`IMPLEMENTED=226`, `PARTIAL=169`, `MISSING_FEASIBLE=0`,
`UNSUPPORTED_HARDWARE=62`, and `NOT_APPLICABLE=45`. `PARTIAL` is retained
only where the VMX SDK or roboRIO FPGA semantics cannot honestly be emulated;
the release gate rejects any return to `MISSING_FEASIBLE`.

Next implementation candidates, in priority order, are hardware AnalogTrigger
coexistence, Counter pulse-length mode, USB serial
`kUSB1`/`kUSB2`, and DIO glitch filtering. FPGA-only DMA/relay/radio-LED/main
surfaces remain deliberate `NOT_APPLICABLE`/`UNSUPPORTED_HARDWARE` statuses.
