// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// HAL_IsInitialized, RunInitialize, CheckInit and the CAN power/pneumatics
// initializers are declared once in the shared header. The athena build
// compiles src/main/native/shared alongside this directory, so redeclaring
// CheckInit here would put two different inline definitions in one binary.
#include "../shared/HALInitializer.h"

namespace hal::init {
extern void InitializeAccelerometer();
extern void InitializeAddressableLED();
extern void InitializeAnalogAccumulator();
extern void InitializeAnalogGyro();
extern void InitializeAnalogInput();
extern void InitializeAnalogInternal();
extern void InitializeAnalogOutput();
extern void InitializeAnalogTrigger();
extern void InitializeCAN();
extern void InitializeCANAPI();
extern void InitializeConstants();
extern void InitializeCounter();
extern void InitializeDigitalInternal();
extern void InitializeDIO();
extern void InitializeDMA();
extern void InitializeDutyCycle();
extern void InitializeEncoder();
extern void InitializeFPGAEncoder();
extern void InitializeFRCDriverStation();
extern void InitializeHAL();
extern void InitializeI2C();
extern void InitializeInterrupts();
extern void InitializeLEDs();
extern void InitializeMain();
extern void InitializeNotifier();
extern void InitializePorts();
extern void InitializePower();
extern void InitializePWM();
extern void InitializeRelay();
extern void InitializeSerialPort();
extern void InitializeSPI();
extern void InitializeThreads();
}  // namespace hal::init
