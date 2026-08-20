# VMX drivers native component

This directory is based on the `drivers` code from the
[Studica Robotics ROS2 repository](https://github.com/Studica-Robotics/ROS2).
It is not a new driver collection written by this project. The upstream
copyright and license notices must be preserved, and the `.cpp` and `.hpp`
driver sources in this directory must remain unmodified so a future Studica
upstream refresh can replace them cleanly.

The driver sources were audited against Studica ROS2 commit
`1a52cfbf3c3eab51807b9d2d7fa42dd6143882df`. The upstream Apache License 2.0
is retained in [`LICENSE`](LICENSE).

This project does not use Studica's ROS2 node layer. It integrates only the
standalone driver layer needed to access VMX-Pi and VMX2 hardware. WPILib API
adaptation, HAL status handling, lifecycle correction, and semantic conversion
belong in `hal/src/main/native/vmx`, never in these driver sources.

The dependency direction is strictly:

```text
WPILib HAL
  -> VMX adapter
  -> vmxdrivers (Studica upstream source, unmodified)
  -> VMXPi HAL SDK
```

`vmxdrivers` is independent of WPILib and must not contain WPILib types, HAL
status values, or WPILib-specific wrappers.

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

The VMX-Pi and VMX2 deployment target is Linux AArch64 only. The shared
library or every ELF object in the static archive must be ELF64 little-endian
with `e_machine = AArch64` (183). An ELF32 ARM/EABI5 SDK is a legacy,
incompatible artifact and is rejected by `verifyVmxSdk` before compilation;
there is no armhf target, 32-bit fallback, helper daemon, IPC bridge, or forced
link workaround. Obtain a matching AArch64 SDK from the current VMX OS/WPILib
image or build the native VMX HAL sources for AArch64 before setting this
path.

The upstream driver headers use the VMX SDK directly. A final executable or shared HAL
backend that consumes `libvmxdrivers.a` must also link `vmxpi_hal_cpp`, `rt`,
`pthread`, and (for modern GCC aarch64 toolchains) `atomic`. The static driver
archive itself does not embed those libraries.

The VMX HAL adapter owns the WPILib-specific resource lifecycle and semantics.
Where the upstream convenience classes do not expose the operation WPILib
needs, the adapter uses the public VMXPi HAL SDK directly; it does not extend or
patch a driver class. This includes DIO pulse/readback, PWM readback and
disable/re-enable behavior, configurable Analog Input/Accumulator operation,
and Encoder handle/source coordination.

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
