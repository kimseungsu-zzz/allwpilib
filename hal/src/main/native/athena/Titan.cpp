// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Titan is a VMX vendor device and is not present on roboRIO.  Reuse the
// host-safe unavailable implementation so the public vendor ABI remains
// linkable without introducing a second hardware implementation.
#include "../sim/Titan.cpp"
