#pragma once

#include "VMXPi.h"
#include <cstdint>
#include <memory>
#include <stdio.h>

namespace studica_driver {

enum class PWMType { Standard, Continuous, Linear };

class PWM {
 public:
  PWM(VMXChannelIndex port, PWMType type, int min = -150, int max = 150,
      std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
  ~PWM();

  bool IsInitialized() const { return initialized_; }
  bool SetPulseTimeMicroseconds(int32_t microseconds);
  bool Disable();
  bool GetLastPulseTimeMicroseconds(int32_t& microseconds) const;

  void SetBounds(double min, double center, double max);

 protected:
  VMXChannelIndex port_;
  PWMType type_;
  int min_;
  int max_;
  std::shared_ptr<VMXPi> vmx_;
  VMXResourceHandle pwm_res_handle_ = 0;
  int min_us_;
  int max_us_;
  int center_us_;
  int prev_pwm_pwm_value_;
  bool initialized_ = false;
  bool has_last_pulse_ = false;
  int32_t last_pulse_microseconds_ = 0;

  int Map(int value);
  bool Activate();
  void DisplayVMXError(VMXErrorCode vmxerr);
};

}  // namespace studica_driver
