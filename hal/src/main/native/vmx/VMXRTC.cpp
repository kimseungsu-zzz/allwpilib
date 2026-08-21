// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VMXRTCInternal.h"

#include <chrono>
#include <ctime>
#include <memory>

#include <time.h>

#include "VMXPi.h"
#include "VMXRuntime.h"

namespace hal::vmx {

namespace {

bool ReadVMXRTC(VMXRTCDateTime& value) noexcept {
  auto context = GetRuntimeContext();
  if (!context || !context->IsOpen()) {
    return false;
  }
  uint8_t weekday = 0;
  uint8_t day = 0;
  uint8_t month = 0;
  uint8_t years = 0;
  uint8_t hours = 0;
  uint8_t minutes = 0;
  uint8_t seconds = 0;
  uint32_t milliseconds = 0;
  VMXErrorCode error = 0;
  auto& time = context->getTime();
  if (!time.GetRTCDate(weekday, day, month, years, &error) ||
      !time.GetRTCTime(hours, minutes, seconds, milliseconds, &error)) {
    return false;
  }
  // VMX RTC stores years as a two-digit value in the 2000-2099 range.
  value = {2000 + static_cast<int>(years), static_cast<int>(month),
           static_cast<int>(day), static_cast<int>(weekday),
           static_cast<int>(hours), static_cast<int>(minutes),
           static_cast<int>(seconds), static_cast<int>(milliseconds)};
  return IsValidVMXRTCDateTime(value);
}

bool SetLinuxSystemClock(const VMXRTCDateTime& value) noexcept {
  if (!IsValidVMXRTCDateTime(value)) {
    return false;
  }
  std::tm utc{};
  utc.tm_year = value.year - 1900;
  utc.tm_mon = value.month - 1;
  utc.tm_mday = value.day;
  utc.tm_hour = value.hours;
  utc.tm_min = value.minutes;
  utc.tm_sec = value.seconds;
  utc.tm_isdst = 0;
  const auto epoch = timegm(&utc);
  if (epoch < 0) {
    return false;
  }
  timespec timestamp{epoch, value.milliseconds * 1'000'000L};
  return clock_settime(CLOCK_REALTIME, &timestamp) == 0;
}

bool ReadLinuxSystemClock(VMXRTCDateTime& value) noexcept {
  const auto now = std::chrono::system_clock::now();
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()) %
                      1000;
  const auto epoch = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  if (gmtime_r(&epoch, &utc) == nullptr) {
    return false;
  }
  value = {utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday, utc.tm_wday,
           utc.tm_hour, utc.tm_min, utc.tm_sec, static_cast<int>(millis.count())};
  return IsValidVMXRTCDateTime(value);
}

bool WriteVMXRTC(const VMXRTCDateTime& value) noexcept {
  auto context = GetRuntimeContext();
  if (!context || !context->IsOpen() || !IsValidVMXRTCDateTime(value)) {
    return false;
  }
  VMXErrorCode error = 0;
  auto& time = context->getTime();
  return time.SetRTCDate(static_cast<uint8_t>(value.weekday),
                         static_cast<uint8_t>(value.day),
                         static_cast<uint8_t>(value.month),
                         static_cast<uint8_t>(value.year - 2000), &error) &&
         time.SetRTCTime(static_cast<uint8_t>(value.hours),
                         static_cast<uint8_t>(value.minutes),
                         static_cast<uint8_t>(value.seconds), &error);
}

}  // namespace

bool BootstrapLinuxSystemClockFromVMXRTC() noexcept {
  return VMXRTCSystemClockBootstrap{ReadVMXRTC, SetLinuxSystemClock}.Bootstrap();
}

bool SyncVMXRTCFromLinuxSystemClock() noexcept {
  VMXRTCDateTime value;
  return ReadLinuxSystemClock(value) && WriteVMXRTC(value);
}

}  // namespace hal::vmx
