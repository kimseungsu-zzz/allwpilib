# VMX HAL backend

This directory contains the Linux aarch64 VMX implementation of WPILib's
existing public HAL API. It is selected only by the explicit `-PvmxBuild`
Gradle flavor.

The dependency direction is:

```text
WPILib public HAL API
  -> VMX HAL adapter (this directory)
  -> vmxdrivers
  -> VMXPi HAL SDK
  -> VMX-Pi / VMX2 hardware
```

Do not put public WPILib API changes, simulation fallbacks, Studica product
APIs, or direct `/usr/local` SDK paths here. Runtime/context initialization and
individual HAL facilities use this adapter boundary.

## Language bindings

This backend is the single hardware implementation shared by all supported
languages:

```text
Java (WPILibJ -> HAL JNI) ---+
C++  (WPILibC) -------------+-> wpiHal -> VMX backend
Python (RobotPy bindings) ---+
```

Do not create VMX-specific Java, C++, or Python hardware APIs. Java's Linux
aarch64 JNI artifact must link this `wpiHal`; C++ links it directly; and a
future mostrobotpy integration must package/select this same native HAL through
`robotpy-native-wpihal`. Python bindings and deployment remain in the
mostrobotpy repository rather than allwpilib.

## Sensor compatibility matrix

The current WPILib sensor compatibility boundary, including the distinction
between supported HAL paths, incomplete primitives, and separate Studica
vendor APIs, is maintained in
[VMX_SENSOR_COMPATIBILITY.md](VMX_SENSOR_COMPATIBILITY.md).

## I/O capability and channel mapping audit

The public WPILib channel number is a logical address, not always a VMX pin:

| WPILib logical range | VMX physical range | VMX bank | Direction/capability rule |
| --- | --- | --- | --- |
| DIO 0-11 / PWM 0-11 | 0-11 | FlexDIO | SDK-selected input/output; interrupt, encoder, counter and PWM routes are available when advertised. |
| DIO 12-21 / PWM 12-21 | 12-21 | HighCurrent DIO | One bank-wide jumper selects INPUT or OUTPUT. `HAL_SetDIODirection()` never changes it. |
| DIO 22-29 / PWM 22-27 | 26-33 | CommDIO | Fixed UART/I2C/SPI and digital capabilities; no runtime direction switch. |
| Analog 0-3 | 22-25 | AnalogIn | Dedicated analog input and AccumulatorInput resources. |

`VMXChannelCapabilities` is the adapter-side capability layer. Static FRC
compatibility maps enforce encoder pairs `0+1` through `8+9` and counter
pairs `0+1` through `10+11`; the SDK `ChannelSupportsCapability()` query is
then used for live jumper/fixed-port state. DIO, PWM, Encoder, Counter,
Interrupt, and communication reservations share one physical registry, so
logical CommDIO DIO/PWM and I2C/SPI/UART resources cannot be allocated
simultaneously.

## Runtime lifecycle

`HAL_Initialize()` creates one process-local `VMXPi(true, 50)` context and
requires `VMXPi::IsOpen()` to succeed. All future HAL subsystems must obtain
that shared context through `GetRuntimeContext()` and pass it to vmxdrivers;
they must not use the drivers' convenience constructors that allocate another
VMXPi object.

The imported driver/SDK-facing code exposes no explicit shutdown call. The
runtime therefore follows its existing RAII contract and releases the final
`shared_ptr<VMXPi>` during idempotent `HAL_Shutdown()` (and through an `atexit`
safety hook). Initialization and shutdown are serialized, exceptions are
contained inside the C HAL boundary, and failure to open hardware is fatal to
initialization—there is no simulation fallback.

The available SDK-facing sources do not document general concurrent I/O safety.
The runtime guarantees lifecycle safety only; each future DIO/PWM/I2C/etc.
adapter must provide any synchronization required by its VMX resource and SDK
calls.

## Timing and Notifier support

VMX HAL time uses the VMX SDK's monotonic hardware clock,
`VMXPi::getTime().GetCurrentTotalMicroseconds()`, as its canonical WPILib time
domain. `HAL_GetFPGATime()` returns that raw 64-bit timestamp without a HAL
initialization offset or a host-clock conversion. The SDK's
[VMXTime API](https://www.kauailabs.com/public_files/vmx-pi/apidocs/hal_cpp/html/class_v_m_x_time.html)
documents the monotonic timer and its microsecond accessor; the shared
[VMXPi context](https://www.kauailabs.com/public_files/vmx-pi/apidocs/hal_cpp/html/class_v_m_x_pi.html)
is used for every read.

The existing `hal/Notifier.h` ABI is implemented by one process-wide VMX
notifier scheduler thread. Each notifier owns its alarm, generation, waiter,
and diagnostic name. The global scheduler registers only its single earliest
deadline with `VMXTime::RegisterTimerNotificationAbsolute()` and uses the VMX
callback as the primary wakeup; it deregisters and re-arms that one notification
whenever the earliest deadline changes. A host condition variable remains as a
bounded safety wakeup and for update/cancel/stop operations, and every wake
re-reads the VMX clock. This keeps WPILib's unbounded notifier count above the
SDK's maximum of ten timer registrations. Alarm comparisons use 64-bit
wrap-safe subtraction, and an alarm at or before the current timestamp fires
immediately. Updating an alarm replaces the previous deadline, while
cancellation removes only the alarm and never releases the waiter. Stop is
permanent and wakes waiters with timestamp zero; clean is idempotent and
performs the same safe wake-up. Wait returns the fired absolute VMX timestamp.

Notifier thread priority uses the Linux thread scheduling ABI. Normal mode is
priority zero, real-time mode validates priorities 1 through 99, and scheduler
permission failures are returned as HAL errors rather than reported as fake
success. `HAL_Shutdown()` stops the scheduler before releasing the shared VMX
runtime, so blocked waits cannot outlive the hardware context.

Interrupt edge timestamps remain in the SDK timestamp domain and therefore can
be compared directly with `HAL_GetFPGATime()`. No Java, C++, or Python public
API changes are required, and `vmxdrivers/` remains an unchanged upstream
driver area.

RTC is deliberately a separate wall-clock concern. VMX `GetRTCTime()`,
`GetRTCDate()`, RTC setters, and daylight-savings settings are not used by
`HAL_GetFPGATime()`, Notifier, interrupt timestamps, or TimedRobot scheduling;
the SDK explicitly distinguishes its RTC and hardware timer APIs. A future VMX
runtime/service will use the VMX RTC as the offline boot-time source for the
Linux system clock and will provide an administrative Linux/PC-to-VMX RTC sync
command. That service boundary does not alter the monotonic HAL time domain.

## DIO support

The VMX DIO core supports logical channels 0 through 29 through the existing WPILib
`hal/DIO.h` ABI. Allocation uses WPILib digital handles, rejects duplicate
channel ownership with the original allocation location, and rolls the handle
back if VMX resource activation fails. Set, get, free, and runtime direction
changes are implemented. Direction changes keep the HAL handle stable while
recreating the underlying driver; a failed change attempts to restore the
last successfully active mode.

VMX inputs are configured by `studica_driver::DIO` as pull-up inputs, so an
otherwise floating input normally reads high. Outputs use push-pull mode. All
DIO drivers receive the single `VMXPi` context owned by `VMXRuntime`; the HAL
never invokes the driver's convenience context constructor.

FlexDIO direction changes are supported. HighCurrent direction is accepted
only when the SDK reports the corresponding bank-wide jumper capability, and
CommDIO direction is fixed by the SDK. An unsupported request returns an
incompatible-state HAL error without changing jumper state.

Current DIO feature status:

- DIO core: supported
- Digital PWM over DIO: not yet implemented (explicit incompatible-state error)
- Timed high pulse generation and hardware pulse-state readback: supported
- Multi-channel pulse requests: supported for allocated VMX DIO outputs
- Digital glitch filters: not yet implemented (explicit incompatible-state error)

## I2C support

The VMX SDK exposes one independent I2C bus: the sole channels advertising
`I2C_SDA` and `I2C_SCL`. The VMX-pi communication port map identifies this as
the WPILib MXP I2C port. The VMX HAL maps that bus to
`HAL_I2C_kMXP`; `HAL_I2C_kOnboard` is explicitly unsupported rather than being
reported as a second fake bus. One VMX I2C resource is shared by all objects
using the supported port, with reference-counted initialization and final-close
deallocation, so different device addresses can be used concurrently on the
same bus.

`HAL_TransactionI2C`, `HAL_WriteI2C`, and `HAL_ReadI2C` validate 7-bit device
addresses (`0x00` through `0x7f`) without applying an 8-bit address shift.
Negative or oversized lengths and non-null buffers for nonzero transfers are
rejected. Zero-length operations may use null pointers. The bus mutex remains
held across each complete blocking VMX operation, including the write/read
sequence of a transaction, and close waits for an in-flight operation before
releasing the resource.

The SDK's blocking `I2C_Transaction()` is the canonical primitive for combined
and read-only operations. WPILib write buffers retain their normal layout: the
first byte is the register address and the remaining bytes are passed through
the SDK's register-aware `I2C_Write()` call. The SDK documents that a combined
transaction writes before reading; the exact repeated-start versus
stop/start wire behavior remains a hardware smoke-test item for register-based
sensors.

## PWM support

The VMX PWM core supports logical channels 0 through 27 through the existing
`hal/PWM.h` ABI. DIO and PWM use one shared VMX digital-channel reservation
registry, so neither facility can claim a channel already owned by the other.
PWM handles remain WPILib `HAL_DigitalHandle` values; they never expose driver
pointers.

PWM state and scaling use WPILib microsecond semantics. The default bounds are
2000/1501/1500/1499/1000 us, custom bounds and eliminate-deadband state are
preserved, and speed and position calculations follow the upstream positive,
negative, and full-range formulas. Non-finite speed requests become zero speed
(the center pulse). Getter state is read back from the active VMX PWM generator
duty-cycle register and converted to the applied, quantized pulse. Disabled
resources report zero.

The available VMX driver and public PWM example establish a 50 Hz generator
with a configured maximum duty value of 5000. A 20,000 us period divided into
5000 steps gives a 4 us native step. WPILib continues to accept integer
microsecond requests; the driver rounds each request to the nearest native
step and reports the applied value. No available SDK contract confirms that a
20,000-step duty range is supported, so this backend does not assume one.

Disabled output is distinct from zero speed. Disable deallocates the VMX PWM
resource while retaining the HAL handle; the next pulse, speed, or position
write reactivates the resource. This behavior follows the SDK operation shown
in the [VMX PWM generation example](https://gist.github.com/kauailabs/199385055741b26ddede7cec9331027c).

Current PWM feature status:

- Generic PWM pulse, speed, position, bounds, and deadband: supported
- PWM output disable/re-enable: supported by resource deallocation/reactivation
- Zero latch: not implemented (explicit incompatible-state error)
- Period scaling: not implemented (explicit incompatible-state error)
- Always-high mode: not implemented (explicit incompatible-state error)
- Loop timing and cycle start time: not implemented (explicit incompatible-state error)

## Analog input support

WPILib logical analog channels 0 through 3 map internally to VMX physical
channels 22 through 25. Physical VMX channel numbers are not exposed through
the public HAL API. Allocation uses indexed WPILib analog handles, preserves
the previous allocation location, and rolls the handle back if VMX resource
activation fails.

The VMX hardware provides four 12-bit analog inputs sampled at 46.5 kS/s per
channel, and its Accumulator resource supports instantaneous raw values,
averaged raw values, averaged voltage, average bits, and oversample bits. These
capabilities are documented by the
[VMXIO API](https://www.kauailabs.com/public_files/vmx-pi/apidocs/hal_cpp/html/class_v_m_x_i_o.html),
[AccumulatorConfig API](https://www.kauailabs.com/public_files/vmx-pi/apidocs/hal_cpp/html/struct_accumulator_config.html),
and [hardware reference manual](https://pdocs.kauailabs.com/vmx-pi/wp-content/uploads/2018/12/VMX-pi-HardwareReferenceManual_v1.11.pdf).

The SDK reports full-scale voltage at runtime. Instantaneous voltage and raw
conversion helpers use that value with the documented 12-bit, 4096-count
model. LSB weight is derived from the same model and offset is zero because no
VMX per-channel calibration-offset API is exposed. Averaged voltage uses the
SDK's voltage read directly.

Average and oversample configuration is applied when the VMX AccumulatorInput
resource is created. A configuration change keeps the HAL handle stable,
deactivates the old resource, activates a replacement, and restores the prior
configuration if activation fails. Failure of both the change and rollback
faults the handle. The default configuration matches WPILib: 7 average bits
and 0 oversample bits.

The documented per-channel sample rate is fixed, so its getter returns 46,500
samples/second and attempts to change it return an explicit incompatible-state
error.

## Analog accumulator support

The standard WPILib accumulator surface is supported on logical analog
channels 0 and 1. Logical channels 2 and 3 remain fully usable as ordinary
analog inputs, but `HAL_IsAccumulatorChannel()` returns false for them. No
second hardware allocation is introduced: accumulator operations travel from
the existing `HAL_AnalogInputHandle`, through its existing analog port and
`studica_driver::AnalogInput`, to the same VMX `AccumulatorInput` resource.

Initialization recreates that resource with the VMX accumulation counter
enabled. Center and deadband use the SDK's signed 16-bit center and nonnegative
15-bit deadband fields. Reset and every value/count query use the SDK's counter
operations; `HAL_GetAccumulatorOutput()` uses one atomic
`Accumulator_Counter_GetValueAndCount()` call.

Average bits, oversample bits, center, and deadband changes require VMX resource
recreation. Before deactivation, the backend atomically snapshots the hardware
value and count and folds both into software offsets. The replacement counter
starts at zero, while HAL output remains the saved offset plus the new hardware
output. A failed replacement is rolled back with the same saved offsets, and a
snapshot failure leaves the live resource untouched. Only an explicit
`HAL_ResetAccumulator()` clears both the hardware counter and software offsets.

## Interrupt support

The VMX Interrupt adapter implements the existing `hal/Interrupts.h` C ABI for
DIO sources. `HAL_InitializeInterrupts()` allocates only the WPILib interrupt
handle; `HAL_RequestInterrupts()` validates the existing DIO input handle and
then activates the VMX `InterruptInput` resource. AnalogTrigger sources remain
an explicit incompatible-state result until the AnalogTrigger milestone.

The VMX callback does not enter JNI, Python, or any other public HAL API. It
only records rising/falling sequence numbers and the SDK callback timestamp in
a native callback state, then notifies a condition variable. Waits use those
independent sequences so `ignorePrevious`, rapid consecutive edges, rising /
falling masks, timeout, and `HAL_ReleaseWaitingInterrupt()` retain WPILib
semantics. `HAL_CleanInterrupts()` closes the callback state before disabling,
deactivating, and deallocating the SDK resource, so a teardown callback cannot
touch a destroyed port.

The adapter leaves the existing DIO resource and ownership registry intact.
This preserves `DigitalInput.Get()` while the SDK decides whether the DIO and
Interrupt routes can coexist; a routing conflict is reported as a hardware
activation failure rather than being hidden by a fake-success fallback.

Rising and falling timestamps are read from the VMX hardware APIs
`Interrupt_GetLastRisingEdgeTimestampMicroseconds()` and
`Interrupt_GetLastFallingEdgeTimestampMicroseconds()`, not from a host clock.
Those values share the time domain that VMX
`HAL_GetFPGATime()` exposes. The Timing + Notifier core now supplies that
shared domain directly from the VMX hardware timer, so a hardware smoke check
can verify each interrupt timestamp is no later than the current VMX time.

## AnalogTrigger support

The VMX AnalogTrigger adapter intentionally does not allocate a separate VMX
AnalogTrigger resource. Each trigger retains an internal shared reference to
the existing `AnalogInputPort`, so its reads continue through the same VMX
`AccumulatorInput` resource used by `AnalogInput` and the accumulator. The
public AnalogInput handle remains owned by the caller; freeing it closes the
underlying port, while an already-created trigger safely observes a hardware
failure instead of dereferencing a reclaimed object.

Raw limits use the VMX 12-bit ADC domain `[0, 4095]`. Voltage limits use the
runtime AnalogInput full-scale voltage and remain as doubles, without an
intermediate raw quantization. Both domains enforce lower `<=` upper and the
standard `ANALOG_TRIGGER_LIMIT_ORDER_ERROR` / `PARAMETER_OUT_OF_RANGE` HAL
statuses.

Trigger evaluation follows WPILib hysteresis semantics:

- `value < lower` sets TriggerState false
- `value > upper` sets TriggerState true
- values inside the window preserve the previous TriggerState
- InWindow includes both lower and upper bounds

Instantaneous mode reads the existing raw or voltage AnalogInput value;
`HAL_SetAnalogTriggerAveraged(true)` selects the existing averaged raw or
averaged voltage path. Filtered mode is deliberately explicit: enabling it
returns `INCOMPATIBLE_STATE`, while disabling it succeeds. Rising and falling
pulse outputs return `ANALOG_TRIGGER_PULSE_OUTPUT_ERROR`, and DutyCycle-based
AnalogTrigger initialization/limits return `INCOMPATIBLE_STATE` until a future
DutyCycle Core exists.

AnalogTrigger handles use stable logical WPILib indices (0 through 7), not
physical VMX or FPGA numbers. `HAL_GetAnalogTriggerFPGAIndex()` returns that
logical handle slot for Java, C++, and Python compatibility. No public HAL
header or language binding changes are required, and `vmxdrivers/` remains
unchanged.
