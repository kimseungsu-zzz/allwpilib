#include "analog_input.hpp"

using namespace studica_driver;

AnalogInput::AnalogInput(VMXChannelIndex port, std::shared_ptr<VMXPi> vmx,
                         int32_t average_bits, int32_t oversample_bits,
                         bool enable_accumulator, int32_t center,
                         int32_t deadband)
    : vmx_{std::move(vmx)},
      port_{port},
      average_bits_{average_bits},
      oversample_bits_{oversample_bits},
      accumulator_enabled_{enable_accumulator},
      center_{center},
      deadband_{deadband} {
  if (!vmx_ || !vmx_->IsOpen() || average_bits < 0 || average_bits > 255 ||
      oversample_bits < 0 || oversample_bits > 255 || center < INT16_MIN ||
      center > INT16_MAX || deadband < 0 || deadband > INT16_MAX) {
    return;
  }

  VMXErrorCode vmxerr;
  float full_scale_voltage = 0.0f;
  if (!vmx_->io.Accumulator_GetFullScaleVoltage(full_scale_voltage, &vmxerr)) {
    return;
  }
  if (full_scale_voltage <= 0.0f) {
    return;
  }

  AccumulatorConfig config;
  config.SetNumAverageBits(static_cast<uint8_t>(average_bits));
  config.SetNumOversampleBits(static_cast<uint8_t>(oversample_bits));
  config.SetEnableAccumulationCounter(enable_accumulator);
  config.SetAccumulationCounterCenter(static_cast<int16_t>(center));
  config.SetAccumulationCounterDeadband(static_cast<int16_t>(deadband));
  if (!vmx_->io.ActivateSinglechannelResource(
          VMXChannelInfo(port_, VMXChannelCapability::AccumulatorInput),
          &config, accumulator_res_handle_, &vmxerr)) {
    return;
  }

  full_scale_voltage_ = full_scale_voltage;
  initialized_ = true;
}

AnalogInput::~AnalogInput() {
  if (!initialized_) {
    return;
  }
  VMXErrorCode vmxerr;
  vmx_->io.DeactivateResource(accumulator_res_handle_, &vmxerr);
}

bool AnalogInput::GetValue(uint32_t& value) {
  if (!initialized_) {
    return false;
  }
  VMXErrorCode vmxerr;
  return vmx_->io.Accumulator_GetInstantaneousValue(accumulator_res_handle_,
                                                    value, &vmxerr);
}

bool AnalogInput::GetAverageValue(uint32_t& value) {
  if (!initialized_) {
    return false;
  }
  VMXErrorCode vmxerr;
  return vmx_->io.Accumulator_GetAverageValue(accumulator_res_handle_, value,
                                              &vmxerr);
}

bool AnalogInput::GetVoltage(double& voltage) {
  uint32_t value = 0;
  if (!GetValue(value)) {
    voltage = 0.0;
    return false;
  }
  voltage = static_cast<double>(value) * full_scale_voltage_ / 4096.0;
  return true;
}

bool AnalogInput::GetAverageVoltage(double& voltage) {
  if (!initialized_) {
    voltage = 0.0;
    return false;
  }
  VMXErrorCode vmxerr;
  float average_voltage = 0.0f;
  if (!vmx_->io.Accumulator_GetAverageVoltage(accumulator_res_handle_,
                                              average_voltage, &vmxerr)) {
    voltage = 0.0;
    return false;
  }
  voltage = average_voltage;
  return true;
}

bool AnalogInput::GetAverageVoltage(float& voltage) {
  double converted = 0.0;
  bool success = GetAverageVoltage(converted);
  voltage = static_cast<float>(converted);
  return success;
}

bool AnalogInput::ResetAccumulator() {
  if (!initialized_ || !accumulator_enabled_) {
    return false;
  }
  VMXErrorCode vmxerr;
  return vmx_->io.Accumulator_Counter_Reset(accumulator_res_handle_, &vmxerr);
}

bool AnalogInput::GetAccumulatorOutput(int64_t& value, uint32_t& count) {
  if (!initialized_ || !accumulator_enabled_) {
    value = 0;
    count = 0;
    return false;
  }
  VMXErrorCode vmxerr;
  return vmx_->io.Accumulator_Counter_GetValueAndCount(
      accumulator_res_handle_, value, count, &vmxerr);
}
