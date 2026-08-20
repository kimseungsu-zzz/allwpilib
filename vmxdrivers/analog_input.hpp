#pragma once

#include "VMXPi.h"

#include <cstdint>
#include <memory>

namespace studica_driver {

class AnalogInput {
 public:
  AnalogInput(VMXChannelIndex port,
              std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50),
              int32_t average_bits = 7, int32_t oversample_bits = 0,
              bool enable_accumulator = false, int32_t center = 0,
              int32_t deadband = 0);
  ~AnalogInput();

  bool IsInitialized() const { return initialized_; }

  bool GetValue(uint32_t& value);
  bool GetAverageValue(uint32_t& value);
  bool GetVoltage(double& voltage);
  bool GetAverageVoltage(double& voltage);
  bool GetAverageVoltage(float& voltage);
  bool ResetAccumulator();
  bool GetAccumulatorOutput(int64_t& value, uint32_t& count);

  int32_t GetAverageBits() const { return average_bits_; }
  int32_t GetOversampleBits() const { return oversample_bits_; }
  double GetFullScaleVoltage() const { return full_scale_voltage_; }

 private:
  std::shared_ptr<VMXPi> vmx_;
  VMXChannelIndex port_;
  VMXResourceHandle accumulator_res_handle_ = 0;
  int32_t average_bits_ = 7;
  int32_t oversample_bits_ = 0;
  bool accumulator_enabled_ = false;
  int32_t center_ = 0;
  int32_t deadband_ = 0;
  double full_scale_voltage_ = 0.0;
  bool initialized_ = false;
};

}  // namespace studica_driver
