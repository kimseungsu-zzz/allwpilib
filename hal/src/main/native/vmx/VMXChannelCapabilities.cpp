// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VMXChannelCapabilities.h"

#include "VMXPi.h"
#include "VMXRuntime.h"

namespace hal::vmx {
namespace {

VMXChannelCapability ToSdkCapability(VMXCapability capability) {
  switch (capability) {
    case VMXCapability::kDigitalInput:
      return VMXChannelCapability::DigitalInput;
    case VMXCapability::kDigitalOutput:
      return VMXChannelCapability::DigitalOutput;
    case VMXCapability::kInterruptInput:
      return VMXChannelCapability::InterruptInput;
    case VMXCapability::kPWMGenerator:
      return VMXChannelCapability::PWMGeneratorOutput;
    case VMXCapability::kPWMCapture:
      return VMXChannelCapability::InputCaptureInput;
    case VMXCapability::kEncoderA:
      return VMXChannelCapability::EncoderAInput;
    case VMXCapability::kEncoderB:
      return VMXChannelCapability::EncoderBInput;
    case VMXCapability::kCounterInput:
      return VMXChannelCapability::InputCaptureInput;
    case VMXCapability::kAnalogInput:
      return VMXChannelCapability::AnalogTriggerInput;
    case VMXCapability::kAccumulatorInput:
      return VMXChannelCapability::AccumulatorInput;
    case VMXCapability::kAnalogTriggerInput:
      return VMXChannelCapability::AnalogTriggerInput;
    case VMXCapability::kI2CSDA:
      return VMXChannelCapability::I2C_SDA;
    case VMXCapability::kI2CSCL:
      return VMXChannelCapability::I2C_SCL;
    case VMXCapability::kSPICLK:
      return VMXChannelCapability::SPI_CLK;
    case VMXCapability::kSPIMISO:
      return VMXChannelCapability::SPI_MISO;
    case VMXCapability::kSPIMOSI:
      return VMXChannelCapability::SPI_MOSI;
    case VMXCapability::kSPICS:
      return VMXChannelCapability::SPI_CS;
    case VMXCapability::kUARTTX:
      return VMXChannelCapability::UART_TX;
    case VMXCapability::kUARTRX:
      return VMXChannelCapability::UART_RX;
    case VMXCapability::kAddressableLED:
      return VMXChannelCapability::LEDArray_OneWire;
  }
  return VMXChannelCapability::NoCapabilities;
}

}  // namespace

const VMXCapabilityProvider& GetVMXCapabilityProvider() noexcept {
  static const VMXCapabilityProvider provider{[](int32_t physical,
                                                 VMXCapability capability) {
    auto context = GetRuntimeContext();
    if (!context || !context->IsOpen() || !IsPhysicalChannelValid(physical)) {
      return false;
    }
    return context->io.ChannelSupportsCapability(
        static_cast<VMXChannelIndex>(physical), ToSdkCapability(capability));
  }};
  return provider;
}

bool ReadVMXChannelInventory(VMXChannelInventory& inventory) noexcept {
  inventory = {};
  auto context = GetRuntimeContext();
  if (!context || !context->IsOpen()) {
    return false;
  }
  VMXChannelIndex first = INVALID_VMX_CHANNEL_INDEX;
  inventory.flexDIO = context->io.GetNumChannelsByType(FlexDIO, first);
  inventory.analogInput = context->io.GetNumChannelsByType(AnalogIn, first);
  inventory.highCurrentDIO =
      context->io.GetNumChannelsByType(HiCurrDIO, first);
  inventory.commDIO = context->io.GetNumChannelsByType(CommDIO, first);
  inventory.valid = true;
  return true;
}

bool ReadVMXResourceChannels(uint32_t resourceHandle, int32_t& firstChannel,
                             uint8_t& channelCount) noexcept {
  firstChannel = -1;
  channelCount = 0;
  auto context = GetRuntimeContext();
  if (!context || !context->IsOpen()) {
    return false;
  }
  VMXChannelIndex first = INVALID_VMX_CHANNEL_INDEX;
  if (!context->io.GetChannelsCompatibleWithResource(
          resourceHandle, first, channelCount)) {
    return false;
  }
  firstChannel = first == INVALID_VMX_CHANNEL_INDEX ? -1 : first;
  return firstChannel >= 0;
}

VMXCommDIOChannelMap GetVMXCommDIOChannelMap() noexcept {
  auto map = kDefaultVMXCommDIOChannelMap;
  auto context = GetRuntimeContext();
  if (!context || !context->IsOpen()) {
    return map;
  }

  const auto get = [&context](VMXChannelCapability capability,
                              int32_t& destination) {
    auto channel = context->io.GetSoleChannelIndex(capability);
    if (channel == INVALID_VMX_CHANNEL_INDEX) {
      return false;
    }
    destination = static_cast<int32_t>(channel);
    return true;
  };
  map.i2cValid = get(VMXChannelCapability::I2C_SDA, map.i2cSDA) &&
                 get(VMXChannelCapability::I2C_SCL, map.i2cSCL);
  map.spiValid = get(VMXChannelCapability::SPI_CLK, map.spiCLK) &&
                 get(VMXChannelCapability::SPI_MOSI, map.spiMOSI) &&
                 get(VMXChannelCapability::SPI_MISO, map.spiMISO) &&
                 get(VMXChannelCapability::SPI_CS, map.spiCS);
  map.uartValid = get(VMXChannelCapability::UART_TX, map.uartTX) &&
                  get(VMXChannelCapability::UART_RX, map.uartRX);
  return map;
}

}  // namespace hal::vmx
