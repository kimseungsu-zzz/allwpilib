#include "pwm.hpp"

#include <cmath>

using namespace studica_driver;

namespace {
constexpr int32_t kFrequencyHz = 50;
constexpr int32_t kPeriodMicroseconds = 1'000'000 / kFrequencyHz;
constexpr int32_t kMaxDutyCycleValue = 5000;
}  // namespace

PWM::PWM(VMXChannelIndex port, PWMType type, int min, int max,
         std::shared_ptr<VMXPi> vmx)
    : port_(port),
      type_(type),
      min_(min),
      max_(max),
      vmx_(vmx),
      prev_pwm_pwm_value_(min - 1) {
  if (port_ < 0 || port_ > 21) {
    printf("Port %d is not a valid PWM port!\n", port_);
    return;
  }
  if (!vmx_ || !vmx_->IsOpen()) {
    printf("VMX HAL unavailable; skipping PWM initialization on port %d\n",
           port_);
    return;
  }

  SetBounds(0.5, 1.5, 2.5);
  Activate();
}

PWM::~PWM() {
  Disable();
}

bool PWM::Activate() {
  if (initialized_)
    return true;
  if (!vmx_ || !vmx_->IsOpen() || port_ < 0 || port_ > 21)
    return false;

  PWMGeneratorConfig pwmgen_cfg(kFrequencyHz);
  pwmgen_cfg.SetMaxDutyCycleValue(kMaxDutyCycleValue);
  VMXErrorCode vmxerr;
  if (!vmx_->io.ActivateSinglechannelResource(
          VMXChannelInfo(port_, VMXChannelCapability::PWMGeneratorOutput),
          &pwmgen_cfg, pwm_res_handle_, &vmxerr)) {
    printf("Failed to initialize PWM output on port %d\n", port_);
    DisplayVMXError(vmxerr);
    return false;
  }
  initialized_ = true;
  return true;
}

bool PWM::SetPulseTimeMicroseconds(int32_t microseconds) {
  if (microseconds <= 0 || microseconds > kPeriodMicroseconds)
    return false;
  bool was_initialized = initialized_;
  if (!Activate())
    return false;

  int32_t duty = static_cast<int32_t>(
      std::lround(static_cast<double>(microseconds) * kMaxDutyCycleValue /
                  kPeriodMicroseconds));
  VMXErrorCode vmxerr;
  if (!vmx_->io.PWMGenerator_SetDutyCycle(pwm_res_handle_, port_, duty,
                                          &vmxerr)) {
    printf("Failed to set PWM duty cycle on port %d\n", port_);
    DisplayVMXError(vmxerr);
    if (!was_initialized)
      Disable();
    return false;
  }

  last_pulse_microseconds_ = static_cast<int32_t>(std::lround(
      static_cast<double>(duty) * kPeriodMicroseconds / kMaxDutyCycleValue));
  has_last_pulse_ = true;
  return true;
}

bool PWM::Disable() {
  if (!initialized_) {
    last_pulse_microseconds_ = 0;
    has_last_pulse_ = true;
    return true;
  }

  VMXErrorCode vmxerr;
  if (!vmx_->io.DeallocateResource(pwm_res_handle_, &vmxerr)) {
    printf("Failed to disable PWM output on port %d\n", port_);
    DisplayVMXError(vmxerr);
    return false;
  }
  initialized_ = false;
  last_pulse_microseconds_ = 0;
  has_last_pulse_ = true;
  return true;
}

bool PWM::GetLastPulseTimeMicroseconds(int32_t& microseconds) const {
  if (!has_last_pulse_)
    return false;
  microseconds = last_pulse_microseconds_;
  return true;
}

bool PWM::GetPulseTimeMicroseconds(int32_t& microseconds) {
  if (!initialized_) {
    if (has_last_pulse_ && last_pulse_microseconds_ == 0) {
      microseconds = 0;
      return true;
    }
    return false;
  }

  uint16_t duty = 0;
  VMXErrorCode vmxerr;
  if (!vmx_->io.PWMGenerator_GetDutyCycle(pwm_res_handle_, port_, &duty,
                                           &vmxerr)) {
    return false;
  }
  microseconds = static_cast<int32_t>(std::lround(
      static_cast<double>(duty) * kPeriodMicroseconds / kMaxDutyCycleValue));
  last_pulse_microseconds_ = microseconds;
  has_last_pulse_ = true;
  return true;
}

void PWM::SetBounds(double min, double center, double max) {
  min_us_ = static_cast<int>((min / 20) * 5000);
  center_us_ = static_cast<int>((center / 20) * 5000);
  max_us_ = static_cast<int>((max / 20) * 5000);
}

int PWM::Map(int value) {
  if (value < min_)
    value = min_;
  if (value > max_)
    value = max_;
  return static_cast<int>((value - min_) * (max_us_ - min_us_) / (max_ - min_) +
                          min_us_);
}

void PWM::DisplayVMXError(VMXErrorCode vmxerr) {
  const char* p_err_description = GetVMXErrorString(vmxerr);
  printf("VMXError %d: %s\n", vmxerr, p_err_description);
}
