// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdint>

namespace hal::vmx {

constexpr int32_t kNumDigitalChannels = 22;
constexpr int32_t kNumDIOChannels = kNumDigitalChannels;
constexpr int32_t kNumPWMChannels = kNumDigitalChannels;
constexpr int32_t kNumAnalogInputs = 4;
constexpr int32_t kNumAnalogAccumulators = 2;
constexpr int32_t kNumVMXAnalogTriggers = 8;
constexpr int32_t kNumVMXEncoders = 5;
constexpr int32_t kFirstVMXAnalogChannel = 22;

constexpr bool IsDIOChannelValid(int32_t channel) {
  return channel >= 0 && channel < kNumDIOChannels;
}

constexpr bool IsPWMChannelValid(int32_t channel) {
  return channel >= 0 && channel < kNumPWMChannels;
}

constexpr bool IsAnalogInputChannelValid(int32_t channel) {
  return channel >= 0 && channel < kNumAnalogInputs;
}

constexpr bool IsAnalogAccumulatorChannelValid(int32_t channel) {
  return channel >= 0 && channel < kNumAnalogAccumulators;
}

constexpr int32_t ToVMXAnalogChannel(int32_t logicalChannel) {
  return logicalChannel + kFirstVMXAnalogChannel;
}

}  // namespace hal::vmx
