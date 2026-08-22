// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// The VMX build compiles src/main/native/shared alongside this directory, so
// CheckInit and the CAN power/pneumatics initializers are declared there once
// rather than duplicated per platform.
#include "../shared/HALInitializer.h"
