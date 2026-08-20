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

## DIO support

The VMX DIO core supports channels 0 through 21 through the existing WPILib
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

Current DIO feature status:

- DIO core: supported
- Digital PWM over DIO: not yet implemented (explicit incompatible-state error)
- Timed high pulse generation and hardware pulse-state readback: supported
- Multi-channel pulse requests: supported for allocated VMX DIO outputs
- Digital glitch filters: not yet implemented (explicit incompatible-state error)

## PWM support

The VMX PWM core supports channels 0 through 21 through the existing
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
