# VMX drivers native component

This directory contains the standalone Studica hardware-driver sources. It is
intentionally independent of ROS2 and does not depend on WPILib. WPILib's VMX
HAL backend depends on this component; the dependency must not point
in the opposite direction.

## SDK layout

Set `VMX_SDK_ROOT` (or pass `-PvmxSdkRoot=...`) to a VMX Linux aarch64 SDK or
cross-compilation sysroot with this layout:

```text
VMX_SDK_ROOT/
  include/
    VMXPi.h             # or include/vmxpi/VMXPi.h
    ...
  lib/
    libvmxpi_hal_cpp.a  # preferred
    # or libvmxpi_hal_cpp.so
```

The driver headers use the VMX SDK directly. A final executable or shared HAL
backend that consumes `libvmxdrivers.a` must also link `vmxpi_hal_cpp`, `rt`,
`pthread`, and (for modern GCC aarch64 toolchains) `atomic`. The static driver
archive itself does not embed those libraries.

`DIO::Set(bool)` and `DIO::Get(bool&)` return operation success separately
from the pin value. This lets hardware adapters distinguish a valid low input
from an SDK read failure without introducing any WPILib status types into this
standalone component. DIO inputs use the VMX SDK pull-up mode; outputs use
push-pull mode.

The generic `PWM` driver exposes pulse widths rather than servo angle or speed:

- `SetPulseTimeMicroseconds()` converts to the configured VMX duty units.
- `Disable()` deallocates the generator resource and a later set reactivates it.
- `GetLastPulseTimeMicroseconds()` returns the last successfully applied,
  quantized pulse.

The driver retains the established 50 Hz, 5000-step VMX configuration. This is
a 20,000 us period with 4 us duty resolution; requested microseconds are
rounded to the nearest 4 us. The public SDK material available to this project
does not establish that 20,000 duty steps are supported, so the driver does
not raise the configured limit speculatively. Servo remains a higher-level
class built on this generic PWM resource.

The generic `AnalogInput` driver now activates one configurable
`AccumulatorInput` resource and exposes distinct success-returning operations
for instantaneous raw values, averaged raw values, instantaneous voltage, and
averaged voltage. Its constructor accepts average and oversample bits rather
than hard-coding nine average bits. The full-scale voltage is queried from the
SDK, and destruction explicitly calls `DeactivateResource()` so a channel can
be allocated again safely. The same constructor can enable the resource's
hardware accumulation counter and configure its center and deadband. Counter
reset and atomic value/count reads are exposed as success-returning driver
operations. No WPILib status or handle types enter the driver.

No `/usr/local` path is used by the Gradle build. This keeps host builds and
cross compilation separate.

## Build

From the allwpilib root:

```shell
VMX_SDK_ROOT=/path/to/vmx-sdk ./gradlew :vmxdrivers:build
```

The only buildable native variant is the WPILib `linuxarm64` static library.
The release result is
`vmxdrivers/build/libs/vmxdrivers/static/linuxarm64/release/libvmxdrivers.a`;
the debug result is in the adjacent `debug` directory and is named
`libvmxdriversd.a`.

Use `./gradlew :vmxdrivers:printVmxSdkConfiguration` to inspect the resolved SDK
contract without compiling. Merely configuring allwpilib, or building an
unrelated desktop/roboRIO project, does not require the VMX SDK.
