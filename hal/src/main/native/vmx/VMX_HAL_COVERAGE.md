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

The checker is intentionally strict about manifest completeness. It does not
turn a `PARTIAL` or `HARDWARE_VALIDATION_REQUIRED` row into a support claim.

| Header | Default status | Symbol overrides | Notes |
| --- | --- | --- | --- |
| `Accelerometer.h` | `IMPLEMENTED` | - | Standard accelerometer subset is VMX AHRS-backed. |
| `AddressableLED.h` | `IMPLEMENTED` | - | VMX LEDArray_OneWire adapter. |
| `AnalogAccumulator.h` | `IMPLEMENTED` | - | Atomic value/count and continuity offsets. |
| `AnalogGyro.h` | `IMPLEMENTED` | - | AnalogInput/AccumulatorInput-backed implementation. |
| `AnalogInput.h` | `IMPLEMENTED` | - | Four logical analog channels. |
| `AnalogOutput.h` | `UNSUPPORTED_HARDWARE` | - | No VMX analog-output resource. |
| `AnalogTrigger.h` | `PARTIAL` | - | Raw/window state works; filtered/pulse modes report unsupported. |
| `CAN.h` | `IMPLEMENTED` | - | VMX CANAPI backend. |
| `CANAPI.h` | `IMPLEMENTED` | - | Logical device and stream sessions over one bus. |
| `Constants.h` | `IMPLEMENTED` | - | VMX monotonic time is expressed in microseconds, so the legacy tick conversion is one tick per microsecond. |
| `Counter.h` | `PARTIAL` | HAL_SetCounterPulseLengthMode=UNSUPPORTED_HARDWARE | InputCapture-backed modes; pulse-length semantics are not equivalent. |
| `CTREPCM.h` | `IMPLEMENTED` | - | Shared canonical CANAPI implementation. |
| `DIO.h` | `PARTIAL` | HAL_SetDigitalPWMPPS=UNSUPPORTED_HARDWARE; HAL_SetFilterSelect=UNSUPPORTED_HARDWARE; HAL_GetFilterSelect=UNSUPPORTED_HARDWARE; HAL_SetFilterPeriod=UNSUPPORTED_HARDWARE; HAL_GetFilterPeriod=UNSUPPORTED_HARDWARE | Normal DIO, pulse, output, and general DigitalPWM paths use the VMX PWMGenerator resource. PPS and glitch filtering remain deliberate boundaries. |
| `DMA.h` | `UNSUPPORTED_HARDWARE` | - | FPGA DMA has no VMX SDK equivalent. |
| `DriverStation.h` | `PARTIAL` | - | VMX KauaiLabs-compatible transport/state adapter. |
| `DutyCycle.h` | `IMPLEMENTED` | - | FlexDIO PWMCapture adapter. |
| `Encoder.h` | `IMPLEMENTED` | - | Valid-pair, index-source, and readback paths. |
| `Extensions.h` | `NOT_APPLICABLE` | - | roboRIO extension-loader surface. |
| `I2C.h` | `IMPLEMENTED` | - | Shared CommDIO I2C resource. |
| `Interrupts.h` | `IMPLEMENTED` | - | DIO edge interrupts and VMX timestamps. |
| `LEDs.h` | `UNSUPPORTED_HARDWARE` | - | roboRIO radio/RSL LED surface has no VMX equivalent. |
| `Main.h` | `NOT_APPLICABLE` | - | roboRIO main-loop launcher surface. |
| `Notifier.h` | `IMPLEMENTED` | - | One global VMX timer-notification scheduler. |
| `Ports.h` | `IMPLEMENTED` | - | Static VMX logical port counts are exposed; zero counts mark unsupported relay/analog-output surfaces. |
| `Power.h` | `PARTIAL` | - | VMX voltage/thermal subset; FPGA-only rails are explicit errors. |
| `PowerDistribution.h` | `IMPLEMENTED` | - | Shared CTREPDP/REVPDH CANAPI implementation. |
| `PWM.h` | `PARTIAL` | HAL_LatchPWMZero=UNSUPPORTED_HARDWARE; HAL_SetPWMPeriodScale=UNSUPPORTED_HARDWARE; HAL_SetPWMAlwaysHighMode=UNSUPPORTED_HARDWARE; HAL_GetPWMLoopTiming=UNSUPPORTED_HARDWARE; HAL_GetPWMCycleStartTime=UNSUPPORTED_HARDWARE | Normal PWM, Servo, and PWMVictorSPX paths are implemented. |
| `Relay.h` | `UNSUPPORTED_HARDWARE` | - | VMX has no relay hardware. |
| `REVPH.h` | `IMPLEMENTED` | - | Shared REV PH CANAPI implementation. |
| `SerialPort.h` | `PARTIAL` | - | kMXP TTL UART works; RS-232/USB aliases remain explicit gaps. |
| `SimDevice.h` | `NOT_APPLICABLE` | - | Simulation-only API is not a VMX production resource. |
| `SPI.h` | `PARTIAL` | - | Basic SPI and HAL-owned AutoSPI; exact AutoStall remains unsupported. |
| `Threads.h` | `IMPLEMENTED` | - | VMX thread priority/interruptible sleep adapter. |
| `Value.h` | `NOT_APPLICABLE` | - | Simulation value helper types. |
| `FRCUsageReporting.h` | `IMPLEMENTED` | - | `HAL_Report` is a no-op-compatible VMX implementation. |
| `HAL.h` | `NOT_APPLICABLE` | - | Umbrella include; declarations are covered by their owning headers. |
| `HALBase.h` | `PARTIAL` | HAL_GetComments=UNSUPPORTED_HARDWARE; HAL_GetErrorMessage=UNSUPPORTED_HARDWARE; HAL_GetFPGARevision=UNSUPPORTED_HARDWARE; HAL_GetFPGAVersion=UNSUPPORTED_HARDWARE; HAL_GetLastError=UNSUPPORTED_HARDWARE; HAL_GetSerialNumber=UNSUPPORTED_HARDWARE; HAL_GetTeamNumber=UNSUPPORTED_HARDWARE | VMX lifecycle/time/runtime-type subset is implemented; roboRIO identity metadata has no SDK equivalent. |
| `UsageReporting.h` | `NOT_APPLICABLE` | - | Legacy generated usage-reporting declarations/types are not a VMX device resource. |

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
