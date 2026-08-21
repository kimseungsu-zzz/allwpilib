// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <cstdint>
#include <functional>

#include "VMXConstants.h"

namespace hal::vmx {

// HAL-facing capability names.  The VMX SDK remains the source of truth for
// their runtime values; keeping this enum SDK-independent makes the mapping
// and capability tests host-testable.
enum class VMXCapability {
  kDigitalInput,
  kDigitalOutput,
  kInterruptInput,
  kPWMGenerator,
  kPWMCapture,
  kEncoderA,
  kEncoderB,
  kCounterInput,
  kAnalogInput,
  kAccumulatorInput,
  kAnalogTriggerInput,
  kI2CSDA,
  kI2CSCL,
  kSPICLK,
  kSPIMISO,
  kSPIMOSI,
  kSPICS,
  kUARTTX,
  kUARTRX,
  kAddressableLED,
};

struct VMXLogicalChannelInfo {
  int32_t logicalChannel = -1;
  int32_t physicalChannel = -1;
  bool digitalInput = false;
  bool digitalOutput = false;
  bool interruptInput = false;
  bool pwmGenerator = false;
  bool pwmCapture = false;
  bool encoderInput = false;
  bool counterInput = false;
};

struct VMXChannelInventory {
  uint8_t flexDIO = 0;
  uint8_t analogInput = 0;
  uint8_t highCurrentDIO = 0;
  uint8_t commDIO = 0;
  bool valid = false;
};

// The eight CommDIO channels are shared by the communication peripherals and
// the logical DIO surface.  Keep this map in the capability layer so every
// adapter (DIO, I2C, SPI, and UART) makes the same physical reservation.
struct VMXCommDIOChannelMap {
  // The documented CommDIO connector order is TTL UART (26/27), SPI
  // (28/29/30/31), then I2C (32/33). These are physical channels, not WPILib
  // port numbers; all aliases reserve the same physical resources.
  int32_t uartTX = 26;
  int32_t uartRX = 27;
  int32_t spiCLK = 28;
  int32_t spiMOSI = 29;
  int32_t spiMISO = 30;
  int32_t spiCS = 31;
  int32_t i2cSDA = 32;
  int32_t i2cSCL = 33;
  bool valid = true;
  bool i2cValid = true;
  bool spiValid = true;
  bool uartValid = true;
};

constexpr VMXCommDIOChannelMap kDefaultVMXCommDIOChannelMap{};

// Reads the board's canonical capability-to-channel mapping.  The host
// fallback is intentionally the documented VMX-pi map so internal adapters
// remain testable without the VMX SDK.
VMXCommDIOChannelMap GetVMXCommDIOChannelMap() noexcept;

// A small injectable seam around VMXIO::ChannelSupportsCapability.  The
// production instance queries the SDK; tests provide lambdas that model
// FlexDIO, HighCurrent jumper, and CommDIO capabilities.
class VMXCapabilityProvider final {
 public:
  using SupportsFunction =
      std::function<bool(int32_t physicalChannel, VMXCapability)>;

  VMXCapabilityProvider() = default;
  explicit VMXCapabilityProvider(SupportsFunction supports)
      : m_supports{std::move(supports)} {}

  bool SupportsPhysical(int32_t physicalChannel,
                        VMXCapability capability) const noexcept {
    try {
      return m_supports && m_supports(physicalChannel, capability);
    } catch (...) {
      return false;
    }
  }

  bool SupportsLogicalDIO(int32_t logicalChannel,
                          VMXCapability capability) const noexcept {
    if (!IsDIOChannelValid(logicalChannel)) {
      return false;
    }
    return SupportsPhysical(ToVMXDigitalChannel(logicalChannel), capability);
  }

  VMXLogicalChannelInfo GetLogicalDIOInfo(
      int32_t logicalChannel) const noexcept {
    VMXLogicalChannelInfo info;
    info.logicalChannel = logicalChannel;
    if (!IsDIOChannelValid(logicalChannel)) {
      return info;
    }
    info.physicalChannel = ToVMXDigitalChannel(logicalChannel);
    info.digitalInput =
        SupportsPhysical(info.physicalChannel, VMXCapability::kDigitalInput);
    info.digitalOutput = SupportsPhysical(info.physicalChannel,
                                          VMXCapability::kDigitalOutput);
    info.interruptInput = SupportsPhysical(
        info.physicalChannel, VMXCapability::kInterruptInput);
    info.pwmGenerator = SupportsPhysical(info.physicalChannel,
                                         VMXCapability::kPWMGenerator);
    info.pwmCapture = SupportsPhysical(info.physicalChannel,
                                       VMXCapability::kPWMCapture);
    info.encoderInput =
        SupportsPhysical(info.physicalChannel, VMXCapability::kEncoderA) ||
        SupportsPhysical(info.physicalChannel, VMXCapability::kEncoderB);
    info.counterInput = SupportsPhysical(info.physicalChannel,
                                         VMXCapability::kCounterInput);
    return info;
  }

 private:
  SupportsFunction m_supports;
};

const VMXCapabilityProvider& GetVMXCapabilityProvider() noexcept;

// Runtime audit helpers. These intentionally mirror the SDK enumeration APIs
// so board revisions cannot silently change the adapter's assumptions.
bool ReadVMXChannelInventory(VMXChannelInventory& inventory) noexcept;
bool ReadVMXResourceChannels(uint32_t resourceHandle, int32_t& firstChannel,
                             uint8_t& channelCount) noexcept;

// FRC compatibility is stricter than merely having two channels with encoder
// flags.  These are the only official quadrature pairs.
constexpr std::array<std::array<int32_t, 2>, 5> kVMXEncoderPairs{{
    {{0, 1}}, {{2, 3}}, {{4, 5}}, {{6, 7}}, {{8, 9}},
}};

constexpr std::array<std::array<int32_t, 2>, 6> kVMXCounterPairs{{
    {{0, 1}}, {{2, 3}}, {{4, 5}}, {{6, 7}}, {{8, 9}}, {{10, 11}},
}};

constexpr bool IsVMXEncoderPair(int32_t channelA, int32_t channelB) noexcept {
  return IsValidEncoderPair(channelA, channelB);
}

constexpr bool IsVMXCounterPair(int32_t channelA, int32_t channelB) noexcept {
  return IsValidCounterPair(channelA, channelB);
}

}  // namespace hal::vmx
