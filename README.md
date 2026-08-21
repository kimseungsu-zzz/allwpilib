# WPILib Project

[![Gradle](https://github.com/wpilibsuite/allwpilib/actions/workflows/gradle.yml/badge.svg?branch=main)](https://github.com/wpilibsuite/allwpilib/actions/workflows/gradle.yml)
[![C++ Documentation](https://img.shields.io/badge/documentation-c%2B%2B-blue)](https://github.wpilib.org/allwpilib/docs/development/cpp/)
[![Java Documentation](https://img.shields.io/badge/documentation-java-orange)](https://github.wpilib.org/allwpilib/docs/development/java/)

Welcome to the WPILib project. This repository contains the HAL, WPILibJ, and WPILibC projects. These are the core libraries for creating robot programs for the roboRIO.

- [WPILib Project](#wpilib-project)
  - [WPILib Mission](#wpilib-mission)
  - [VMX backend status](#vmx-backend-status)
- [Building WPILib](#building-wpilib)
  - [Requirements](#requirements)
  - [Setup](#setup)
  - [Building](#building)
    - [Faster builds](#faster-builds)
    - [Using Development Builds](#using-development-builds)
    - [Custom toolchain location](#custom-toolchain-location)
    - [Formatting/Linting](#formattinglinting)
    - [CMake](#cmake)
    - [Bazel](#bazel)
  - [Running examples in simulation](#running-examples-in-simulation)
  - [Publishing](#publishing)
  - [Structure and Organization](#structure-and-organization)
- [Contributing to WPILib](./CONTRIBUTING.md)

## WPILib Mission

The WPILib Mission is to enable FIRST Robotics teams to focus on writing game-specific software rather than focusing on hardware details - "raise the floor, don't lower the ceiling". We work to enable teams with limited programming knowledge and/or mentor experience to be as successful as possible, while not hampering the abilities of teams with more advanced programming capabilities. We support Kit of Parts control system components directly in the library. We also strive to keep parity between major features of each language (Java, C++, Python, and NI's LabVIEW), so that teams aren't at a disadvantage for choosing a specific programming language. WPILib is an open source project, licensed under the BSD 3-clause license. You can find a copy of the license [here](LICENSE.md).

## VMX backend status

This repository also contains an experimental VMX-Pi/VMX2 HAL backend under
`hal/src/main/native/vmx`. It keeps the existing Java, C++, and Python WPILib
APIs and adapts them through the HAL; it does not add a second public robot
framework. The detailed, synchronized status is maintained in the [VMX HAL
README](hal/src/main/native/vmx/README.md) and [sensor compatibility
matrix](hal/src/main/native/vmx/VMX_SENSOR_COMPATIBILITY.md).

The current communication-port mapping is fixed by the VMX CommDIO connector:
TTL UART `26/27`, SPI `28/29/30/31` (CLK/MOSI/MISO/CS), and I2C `32/33`.
WPILib I2C onboard/MXP names and SPI onboard CS0-3/MXP names are aliases for
the same physical resources and share the registry. Serial `kMXP` is backed by
the SDK UART with baud rates `0..230400`; onboard RS-232 and USB serial
enumeration remain explicit compatibility boundaries. The basic SPI HAL
primitive is available, and the HAL-owned AutoSPI engine supports periodic
rate transfers plus DIO rising, falling, and both-edge triggers. AnalogTrigger
sources and exact FPGA AutoSPI stall timing remain explicitly unsupported;
sensor-level status is still separate from the matrix's HAL/API status.
ADXRS450 now has its required standard-SPI plus AutoSPI HAL path, with sensor
integration testing pending. VMX-Pi and VMX2 deployment targets are Linux
AArch64 only. The currently available VMXPi SDK shared library is ELF32 ARM
EABI5 and is classified as a legacy/incompatible artifact; Gradle rejects it
before compilation/linking. ARM32/armhf targets, helper bridges, and forced
ELF32 links are intentionally absent. Linux ARM64 Debug/Release source checks
remain available, while the final VMX HAL link requires a matching ELF64
AArch64 SDK.

The VMX AnalogGyro HAL is now implemented on the existing AnalogInput and
AccumulatorInput resource. It accepts only logical accumulator channels 0 and
1, uses the VMX fixed 46,500 samples/second rate for angle/rate scaling, and
keeps the five-second calibration wait injectable for native tests. The
AddressableLED HAL now maps the standard color-order, length/data, bit-timing,
sync, start/stop, and render ABI to the VMX `LEDArray_OneWire` resource. It
borrows the WPILib-created PWM handle through the central physical registry,
rejecting SPI/I2C/UART/DIO/PWM/encoder conflicts and restoring PWM ownership on
free. VMX exposes one target frequency, so timing tuples with different bit
periods are rejected rather than silently approximated. BuiltInAccelerometer
uses AHRS raw calibrated X/Y/Z acceleration in G (gravity included); active
state is local to HAL and range requests are retained without claiming a
hardware sensitivity change because the SDK has no range setter. Host/mock
contract tests cover these paths; physical `hardwareValidated` status remains
false until VMX-Pi/VMX2 smoke tests run.
The VMX RTC bootstrap is a separate best-effort wall-clock service invoked at
HAL runtime startup; `HAL_GetFPGATime()`, Notifier, interrupts, and TimedRobot
continue to use VMX monotonic time. A Linux/PC-to-RTC synchronization helper is
kept internal for a future runtime management command. The
RobotController power path reports hardware-backed VMX input voltage and
Linux thermal-sysfs CPU temperature. VMX input current, 3V3/5V/6V rail
telemetry/control, brownout status/threshold, FPGA button, RSL, and Driver
Station disable count are explicit incompatible-state results because the
current SDK has no equivalent APIs; VMX overcurrent is never treated as
brownout. System-time validity uses the Linux wall clock and remains separate
from VMX monotonic FPGA time and RTC synchronization.

The VMX Driver Station core follows the historical KauaiLabs VMX-pi UDP v1
and TCP metadata protocol through a VMX-owned transport/state adapter. It
provides validated control-word and joystick snapshots, mode/alliance/FMS
state, match metadata, new-data events, refresh sequence semantics, observe
program heartbeats, and rumble/output peer handling, with a disabled/detached
failsafe when packets time out or are malformed; an eStop bit also forces
`enabled=false` and disables outputs. Host parser/state tests are
transport-injectable; physical DS enable/disable, joystick, reconnect, and
match smoke tests remain `PARTIAL`, and error/console calls are guaranteed
locally rather than claiming unverified TCP forwarding.

The VMX CAN core now covers the raw CAN C ABI and CANAPI on one global VMX CAN
bus. It sets the bus to FRC-compatible 1 Mbps normal mode once at runtime
startup, maps one-shot/periodic/cancel sends, standard/extended/RTR IDs,
hardware timestamps/status, software-owned stream sessions, logical CANAPI
handles, and New/Latest/Timeout receive generations. CTRE PCM, REV Pneumatics
Hub, PDP, and PDH dependencies have been audited but remain physical-device
integration work rather than premature support claims. The hardware watchdog
is SDK-backed with FlexDIO and HighCurrentDIO managed, CommDIO left unmanaged,
and feeding gated by fresh Driver Station data, a recent user-program
heartbeat, runtime health, and eStop state; timeout, eStop, runtime failure,
and shutdown expire it immediately.

The VMX onboard AHRS is now available through the separate
`studica::VMXIMU` vendor layer and a versioned `StudicaVMXIMU_*` C ABI. Java
JNI and the future Python/ctypes binding use the same fixed-layout snapshot over
the shared `VMXRuntime` `VMXPi::getAHRS()` instance; no second VMXPi is created
and no yaw/pitch/roll fields were added to WPILib core HAL. Raw acceleration
(sensor-frame G including gravity) remains distinct from world-linear
acceleration (gravity-corrected/world-frame). The SDK's last sensor timestamp
is exposed as an opaque sensor-domain value, never converted to FPGA time;
pressure/altitude carry an explicit validity flag. The unchanged
`vmxdrivers/` source area remains upstream-derived and has zero modifications.
The AutoTransmit SDK engine was audited but is not selected for AutoSPI because
it lacks WPILib-compatible per-sample timestamps, full edge semantics, and
precise AutoStall configuration; the tested HAL-owned software engine remains
the backend. The Titan Quad vendor layer is now available as
`studica::TitanQuad`/`studica::TitanQuadEncoder`, Java
`com.studica.frc` compatibility wrappers, and a fixed-width
`StudicaTitan_*` C ABI suitable for Python ctypes. It keeps one shared
`vmxdrivers::Titan` controller per CAN ID, refreshes outputs every 50 ms,
exposes motor/encoder/RPM/Cypher-angle/limit/configuration paths, and
explicitly zeros/disables on close, stale Driver Station state, e-stop, or HAL
shutdown. `hardwareValidated=false` until board tests run. The Cobra vendor
layer now wraps immutable `vmxdrivers::Cobra` through the fixed-width
`StudicaCobra_*` ABI, `studica::Cobra`, and `com.studica.frc.Cobra`; it exposes
four raw/voltage channels and shares only the physical I2C bus with WPILib I2C.
The Light Tower vendor layer similarly exposes `StudicaLightTower_*`,
`studica::LightTower`, and `com.studica.frc.LightTower`, delegating
solid/blink, red/yellow/green/buzzer, and off to immutable
`vmxdrivers::LightTower` while centrally reserving five physical outputs.
Both vendor rows are `softwareValidated=true` and `hardwareValidated=false`.
The synchronized matrix now records the class-level closure harness:
`wpilibc/src/test/native/cpp/VMXSensorClassIntegrationTest.cpp` instantiates
the real WPILib classes and drives public reads through HAL mock data, while
the VMX native tests cover the corresponding adapter/mock-SDK contracts. The
listed analog, encoder, duty-cycle, tachometer, ultrasonic, ADXL, ADXRS450,
and AnalogGyro rows are software-validated; physical validation is still
separate and never implied.

The ordinary Android CMake workflow remains VMX-isolated and now passes the
ABI explicitly as both `ANDROID_ABI` and `CMAKE_ANDROID_ARCH_ABI`, including
the Android X64 configure path. VMX production builds remain explicit Linux
AArch64-only builds.

# Quick Start

Below is a list of instructions that guide you through cloning, building, publishing and using local allwpilib binaries in a robot project. This quick start is not intended as a replacement for the information further listed in this document.

1. Clone the repository with `git clone https://github.com/wpilibsuite/allwpilib.git`
2. Build the repository with `./gradlew build` or `./gradlew build --build-cache` if you have an internet connection
3. Publish the artifacts locally by running `./gradlew publish`
4. [Update your](DevelopmentBuilds.md) `build.gradle` [to use the artifacts](DevelopmentBuilds.md)

# Building WPILib

Using Gradle makes building WPILib very straightforward. It only has a few dependencies on outside tools, such as the ARM cross compiler for creating roboRIO binaries.

## Requirements

- [JDK 17](https://adoptium.net/temurin/releases/?version=17)
    - Note that the JRE is insufficient; the full JDK is required
    - On Ubuntu, run `sudo apt install openjdk-17-jdk`
    - On Windows, install the JDK 17 .msi from the link above
    - On macOS, install the JDK 17 .pkg from the link above
- C++ compiler
    - On Linux, install GCC 11 or greater
    - On Windows, install [Visual Studio Community 2022](https://visualstudio.microsoft.com/vs/community/) and select the C++ programming language during installation (Gradle can't use the build tools for Visual Studio)
    - On macOS, install the Xcode command-line build tools via `xcode-select --install`. Xcode 14 or later is required.
- ARM compiler toolchain
    - Run `./gradlew installRoboRioToolchain` after cloning this repository
    - If the WPILib installer was used, this toolchain is already installed
- Raspberry Pi toolchain (optional)
    - Run `./gradlew installArm32Toolchain` after cloning this repository

On macOS ARM, run `softwareupdate --install-rosetta`. This is necessary to be able to use the macOS x86 roboRIO toolchain on ARM.

## Setup

Clone the WPILib repository and follow the instructions above for installing any required tooling. The build process uses versioning information from git. Downloading the source is not sufficient to run the build.

See the [styleguide README](https://github.com/wpilibsuite/styleguide/blob/main/README.md) for wpiformat setup instructions.

## Building

All build steps are executed using the Gradle wrapper, `gradlew`. Each target that Gradle can build is referred to as a task. The most common Gradle task to use is `build`. This will build all the outputs created by WPILib. To run, open a console and cd into the cloned WPILib directory. Then:

```bash
./gradlew build
```

To build a specific subproject, such as WPILibC, you must access the subproject and run the build task only on that project. Accessing a subproject in Gradle is quite easy. Simply use `:subproject_name:task_name` with the Gradle wrapper. For example, building just WPILibC:

```bash
./gradlew :wpilibc:build
```

The gradlew wrapper only exists in the root of the main project, so be sure to run all commands from there. All of the subprojects have build tasks that can be run. Gradle automatically determines and rebuilds dependencies, so if you make a change in the HAL and then run `./gradlew :wpilibc:build`, the HAL will be rebuilt, then WPILibC.

There are a few tasks other than `build` available. To see them, run the meta-task `tasks`. This will print a list of all available tasks, with a description of each task.

If opening from a fresh clone, generated java dependencies will not exist. Most IDEs will not run the generation tasks, which will cause lots of IDE errors. Manually run `./gradlew compileJava` from a terminal to run all the compile tasks, and then refresh your IDE's configuration (In VS Code open settings.gradle and save).

### Faster builds

`./gradlew build` builds _everything_, which includes debug and release builds for desktop and all installed cross compilers. Many developers don't need or want to build all of this. Therefore, common tasks have shortcuts to only build necessary components for common development and testing tasks.

`./gradlew testDesktopCpp` and `./gradlew testDesktopJava` will build and run the tests for `wpilibc` and `wpilibj` respectively. They will only build the minimum components required to run the tests. `./gradlew testDesktop` will run both `testDesktopJava` and `testDesktopCpp`.

`testDesktopCpp`, `testDesktopJava`, and `testDesktop` tasks also exist for the following projects:

- `apriltag`
- `cameraserver`
- `cscore`
- `hal`
- `ntcore`
- `wpilibNewCommands`
- `wpimath`
- `wpinet`
- `wpiunits`
- `wpiutil`
- `romiVendordep`
- `xrpVendordep`

These can be ran with `./gradlew :projectName:task`.

`./gradlew buildDesktopCpp` and `./gradlew buildDesktopJava` will compile `wpilibcExamples` and `wpilibjExamples` respectively. The results can't be ran, but they can compile.

### Build Cache

Run with `--build-cache` on the command-line to use the shared [build cache](https://docs.gradle.org/current/userguide/build_cache.html) artifacts generated by the continuous integration server. Example:

```bash
./gradlew build --build-cache
```

### Using Development Builds

Please read the documentation available [here](DevelopmentBuilds.md)

### Custom toolchain location

If you have installed the FRC Toolchain to a directory other than the default, or if the Toolchain location is not on your System PATH, you can pass the `toolChainPath` property to specify where it is located. Example:

```bash
./gradlew build -PtoolChainPath=some/path/to/frc/toolchain/bin
```

### Formatting/linting

Once a PR has been submitted, formatting can be run in CI by commenting `/format` on the PR. A new commit will be pushed with the formatting changes.

> [!NOTE]
> The `/format` action has been temporarily disabled. The individual formatting commands can be run locally as shown below. Alternately, the Lint and Format action for a PR will upload a patch file that can be downloaded and applied manually.

#### wpiformat

wpiformat can be executed anywhere in the repository via `py -3 -m wpiformat` on Windows or `python3 -m wpiformat` on other platforms.

#### Java Code Quality Tools

The Java code quality tools Checkstyle, PMD, and Spotless can be run via `./gradlew javaFormat`. SpotBugs can be run via the `spotbugsMain`, `spotbugsTest`, and `spotbugsDev` tasks. These tools will all be run automatically by the `build` task. To disable this behavior, pass the `-PskipJavaFormat` flag.

If you only want to run the Java autoformatter, run `./gradlew spotlessApply`.

### Generated files

Several files within WPILib are generated using Jinja. If a PR is opened that modifies these templates then the files can be generated through CI by commenting `/pregen` on the PR. A new commit will be pushed with the regenerated files. See [GeneratedFiles.md](GeneratedFiles.md) for more information.

### CMake

CMake is also supported for building. See [README-CMake.md](README-CMake.md).

### Bazel

Bazel is also supported for building. See [README-Bazel.md](README-Bazel.md).

## Running examples in simulation

Examples can be run in simulation with the following command:

```bash
./gradlew wpilibcExamples:runExample
./gradlew wpilibjExamples:runExample
```
where `Example` is the example's folder name.

## Publishing

If you are building to test with other dependencies or just want to export the build as a Maven-style dependency, simply run the `publish` task. This task will publish all available packages to ~/releases/maven/development. If you need to publish the project to a different repo, you can specify it with `-Prepo=repo_name`. Valid options are:

- development - The default repo.
- beta - Publishes to ~/releases/maven/beta.
- stable - Publishes to ~/releases/maven/stable.
- release - Publishes to ~/releases/maven/release.

The maven artifacts are described in [MavenArtifacts.md](MavenArtifacts.md)

## Structure and Organization

The main WPILib code you're probably looking for is in WPILibJ and WPILibC. Those directories are split into shared, sim, and athena. Athena contains the WPILib code meant to run on your roboRIO. Sim is WPILib code meant to run on your computer, and shared is code shared between the two. Shared code must be platform-independent, since it will be compiled with both the ARM cross-compiler and whatever desktop compiler you are using (g++, msvc, etc...).

The integration test directories for C++ and Java contain test code that runs on our test-system. When you submit code for review, it is tested by those programs. If you add new functionality you should make sure to write tests for it so we don't break it in the future.

The hal directory contains more C++ code meant to run on the roboRIO. HAL is an acronym for "Hardware Abstraction Layer", and it interfaces with the NI Libraries. The NI Libraries contain the low-level code for controlling devices on your robot. The NI Libraries are found in the [ni-libraries](https://github.com/wpilibsuite/ni-libraries) project.

The upstream_utils directory contains scripts for updating copies of thirdparty code in the repository.

The [styleguide repository](https://github.com/wpilibsuite/styleguide) contains our style guides for C++ and Java code. Anything submitted to the WPILib project needs to follow the code style guides outlined in there. For details about the style, please see the contributors document [here](CONTRIBUTING.md#coding-guidelines).
