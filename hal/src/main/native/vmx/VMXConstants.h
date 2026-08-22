// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdint>

namespace hal::vmx {

// WPILib exposes 30 logical DIO channels.  VMX's physical channel map is
// intentionally kept separate: 0..11 FlexDIO, 12..21 HighCurrent DIO,
// 22..25 analog inputs, and 26..33 CommDIO.
constexpr int32_t kNumPhysicalChannels = 34;
constexpr int32_t kNumDigitalChannels = 30;
constexpr int32_t kNumDIOChannels = kNumDigitalChannels;
constexpr int32_t kNumPWMChannels = 28;
constexpr int32_t kNumAnalogInputs = 4;
constexpr int32_t kNumAnalogAccumulators = 2;
constexpr int32_t kNumVMXAnalogTriggers = 8;
constexpr int32_t kNumVMXEncoders = 5;
constexpr int32_t kFirstVMXAnalogChannel = 22;

// Public HAL port-count ABI.  These values describe the VMX logical surface,
// not a claim that every channel has every FPGA-era capability.
constexpr int32_t kNumAccumulators = kNumAnalogAccumulators;
constexpr int32_t kNumAnalogTriggers = kNumVMXAnalogTriggers;
constexpr int32_t kNumAnalogOutputs = 0;
constexpr int32_t kNumCounters = 8;
constexpr int32_t kNumDigitalHeaders = kNumDigitalChannels;
constexpr int32_t kNumPWMHeaders = kNumPWMChannels;
constexpr int32_t kNumDigitalPWMOutputs = kNumDIOChannels;
constexpr int32_t kNumEncoders = kNumVMXEncoders;
constexpr int32_t kNumInterrupts = 8;
constexpr int32_t kNumRelayChannels = 0;
constexpr int32_t kNumRelayHeaders = 0;
constexpr int32_t kNumCTREPCMModules = 63;
constexpr int32_t kNumCTRESolenoidChannels = 8;
constexpr int32_t kNumCTREPDPModules = 63;
constexpr int32_t kNumCTREPDPChannels = 16;
constexpr int32_t kNumREVPDHModules = 63;
constexpr int32_t kNumREVPDHChannels = 24;
constexpr int32_t kNumREVPHModules = 63;
constexpr int32_t kNumREVPHChannels = 16;
constexpr int32_t kNumDutyCycles = 8;
constexpr int32_t kNumAddressableLEDs = 1;
constexpr int32_t kSystemClockTicksPerMicrosecond = 1;

constexpr bool IsDIOChannelValid(int32_t channel) {
  return channel >= 0 && channel < kNumDIOChannels;
}

constexpr bool IsPWMChannelValid(int32_t channel) {
  return channel >= 0 && channel < kNumPWMChannels;
}

constexpr bool IsPhysicalChannelValid(int32_t channel) {
  return channel >= 0 && channel < kNumPhysicalChannels;
}

// WPILib logical DIO 0..21 are direct VMX channels.  Logical 22..29 are the
// eight CommDIO channels, which occupy VMX physical channels 26..33.
constexpr int32_t ToVMXDigitalChannel(int32_t logicalChannel) {
  return logicalChannel < 22 ? logicalChannel : logicalChannel + 4;
}

constexpr bool IsFlexDIOChannel(int32_t logicalChannel) {
  return logicalChannel >= 0 && logicalChannel < 12;
}

constexpr bool IsHighCurrentDIOChannel(int32_t logicalChannel) {
  return logicalChannel >= 12 && logicalChannel < 22;
}

constexpr bool IsCommDIOChannel(int32_t logicalChannel) {
  return logicalChannel >= 22 && logicalChannel < 30;
}

constexpr bool IsValidEncoderPair(int32_t channelA, int32_t channelB) {
  return channelA >= 0 && channelA < 10 && (channelA % 2 == 0) &&
         channelB == channelA + 1;
}

constexpr bool IsValidCounterPair(int32_t channelA, int32_t channelB) {
  return channelA >= 0 && channelA < 12 && (channelA % 2 == 0) &&
         channelB == channelA + 1;
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
