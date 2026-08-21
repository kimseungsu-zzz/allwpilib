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
| `Constants.h` | `PARTIAL` | HAL_GetSystemClockTicksPerMicrosecond=MISSING_FEASIBLE | VMX time conversion is not yet exposed through this legacy query. |
| `Counter.h` | `PARTIAL` | HAL_SetCounterPulseLengthMode=UNSUPPORTED_HARDWARE | InputCapture-backed modes; pulse-length semantics are not equivalent. |
| `CTREPCM.h` | `IMPLEMENTED` | - | Shared canonical CANAPI implementation. |
| `DIO.h` | `PARTIAL` | HAL_AllocateDigitalPWM=MISSING_FEASIBLE; HAL_FreeDigitalPWM=MISSING_FEASIBLE; HAL_SetDigitalPWMRate=MISSING_FEASIBLE; HAL_SetDigitalPWMDutyCycle=MISSING_FEASIBLE; HAL_SetDigitalPWMPPS=UNSUPPORTED_HARDWARE; HAL_SetDigitalPWMOutputChannel=MISSING_FEASIBLE; HAL_SetFilterSelect=UNSUPPORTED_HARDWARE; HAL_GetFilterSelect=UNSUPPORTED_HARDWARE; HAL_SetFilterPeriod=UNSUPPORTED_HARDWARE; HAL_GetFilterPeriod=UNSUPPORTED_HARDWARE | Normal DIO, pulse, and output paths work. General DigitalPWM is the next feasible gap; PPS and glitch filtering remain deliberate boundaries. |
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
| `Ports.h` | `MISSING_FEASIBLE` | HAL_GetNumAnalogTriggers=IMPLEMENTED | Static VMX channel counts are available for a future complete port-count ABI. |
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

Next implementation candidates, in priority order, are general DigitalPWM,
hardware AnalogTrigger coexistence, Counter pulse-length mode, USB serial
`kUSB1`/`kUSB2`, and DIO glitch filtering. FPGA-only DMA/relay/radio-LED/main
surfaces remain deliberate `NOT_APPLICABLE`/`UNSUPPORTED_HARDWARE` statuses.
