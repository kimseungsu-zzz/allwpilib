// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <utility>

namespace hal::vmx {

/** A UTC wall-clock value exchanged by the VMX RTC bootstrap seam. */
struct VMXRTCDateTime {
  int year = 0;
  int month = 0;
  int day = 0;
  int weekday = 0;
  int hours = 0;
  int minutes = 0;
  int seconds = 0;
  int milliseconds = 0;
};

using VMXRTCReader = std::function<bool(VMXRTCDateTime&)>;
using VMXSystemClockSetter = std::function<bool(const VMXRTCDateTime&)>;

inline bool IsValidVMXRTCDateTime(const VMXRTCDateTime& value) noexcept {
  if (value.year < 2000 || value.year > 2099 || value.month < 1 ||
      value.month > 12 || value.day < 1 || value.hours < 0 ||
      value.hours > 23 || value.minutes < 0 || value.minutes > 59 ||
      value.seconds < 0 || value.seconds > 59 || value.milliseconds < 0 ||
      value.milliseconds > 999) {
    return false;
  }
  const bool leap = value.year % 4 == 0 &&
                    (value.year % 100 != 0 || value.year % 400 == 0);
  constexpr int days[] = {31, 28, 31, 30, 31, 30,
                          31, 31, 30, 31, 30, 31};
  const int maxDay = days[value.month - 1] +
                     ((value.month == 2 && leap) ? 1 : 0);
  return value.day <= maxDay;
}

/** Host-testable RTC-to-system-clock bootstrap. */
class VMXRTCSystemClockBootstrap final {
 public:
  VMXRTCSystemClockBootstrap(VMXRTCReader reader,
                             VMXSystemClockSetter setter)
      : m_reader{std::move(reader)}, m_setter{std::move(setter)} {}

  bool Bootstrap() const noexcept {
    if (!m_reader || !m_setter) {
      return false;
    }
    VMXRTCDateTime value;
    try {
      return m_reader(value) && IsValidVMXRTCDateTime(value) &&
             m_setter(value);
    } catch (...) {
      return false;
    }
  }

 private:
  VMXRTCReader m_reader;
  VMXSystemClockSetter m_setter;
};

/** Best-effort production bootstrap; failure is non-fatal to HAL startup. */
bool BootstrapLinuxSystemClockFromVMXRTC() noexcept;

/** Administrative helper for a future runtime/service RTC sync command. */
bool SyncVMXRTCFromLinuxSystemClock() noexcept;

}  // namespace hal::vmx
