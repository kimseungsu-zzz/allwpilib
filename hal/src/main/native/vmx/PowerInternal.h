// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdint>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace hal::vmx {

enum class PowerResult { kOk, kUnavailable, kUnsupported };

using VoltageReader = std::function<bool(double&)>;
using TemperatureReader = std::function<bool(double&)>;
using RuntimeReader = std::function<bool()>;
using TimeValidReader = std::function<bool()>;

class PowerFacade final {
 public:
  PowerFacade(VoltageReader voltageReader, TemperatureReader temperatureReader,
              RuntimeReader runtimeReader, TimeValidReader timeValidReader)
      : m_voltageReader{std::move(voltageReader)},
        m_temperatureReader{std::move(temperatureReader)},
        m_runtimeReader{std::move(runtimeReader)},
        m_timeValidReader{std::move(timeValidReader)} {}

  PowerResult GetVinVoltage(double& voltage) const noexcept {
    try {
      if (!m_voltageReader || !m_voltageReader(voltage)) {
        voltage = 0.0;
        return PowerResult::kUnavailable;
      }
      return PowerResult::kOk;
    } catch (...) {
      voltage = 0.0;
      return PowerResult::kUnavailable;
    }
  }

  PowerResult GetCPUTemp(double& temperature) const noexcept {
    try {
      if (!m_temperatureReader || !m_temperatureReader(temperature)) {
        temperature = 0.0;
        return PowerResult::kUnavailable;
      }
      return PowerResult::kOk;
    } catch (...) {
      temperature = 0.0;
      return PowerResult::kUnavailable;
    }
  }

  PowerResult GetSystemActive(bool& active) const noexcept {
    try {
      if (!m_runtimeReader) {
        active = false;
        return PowerResult::kUnavailable;
      }
      active = m_runtimeReader();
      return PowerResult::kOk;
    } catch (...) {
      active = false;
      return PowerResult::kUnavailable;
    }
  }

  // VMXPower exposes overcurrent, but that is not a brownout indication.  Do
  // not silently reinterpret one safety signal as the other.
  PowerResult GetBrownedOut(bool& brownedOut) const noexcept {
    brownedOut = false;
    return PowerResult::kUnsupported;
  }

  PowerResult GetSystemTimeValid(bool& valid) const noexcept {
    try {
      if (!m_timeValidReader) {
        valid = false;
        return PowerResult::kUnavailable;
      }
      valid = m_timeValidReader();
      return PowerResult::kOk;
    } catch (...) {
      valid = false;
      return PowerResult::kUnavailable;
    }
  }

 private:
  VoltageReader m_voltageReader;
  TemperatureReader m_temperatureReader;
  RuntimeReader m_runtimeReader;
  TimeValidReader m_timeValidReader;
};

inline bool ParseThermalTemperature(std::string_view text,
                                    double& temperatureCelsius) noexcept {
  try {
    std::size_t parsed = 0;
    const auto value = std::stod(std::string{text}, &parsed);
    while (parsed < text.size() &&
           std::isspace(static_cast<unsigned char>(text[parsed]))) {
      ++parsed;
    }
    if (parsed != text.size() || !std::isfinite(value) ||
        value < -273150.0 || value > 1000000.0) {
      return false;
    }
    temperatureCelsius = value / 1000.0;
    return std::isfinite(temperatureCelsius);
  } catch (...) {
    return false;
  }
}

inline bool ReadCpuTemperatureFromSysfs(std::string_view thermalRoot,
                                        double& temperatureCelsius) noexcept {
  try {
    std::error_code ec;
    const std::filesystem::path root{thermalRoot};
    if (!std::filesystem::is_directory(root, ec) || ec) {
      return false;
    }
    for (const auto& entry : std::filesystem::directory_iterator(root, ec)) {
      if (ec || !entry.is_directory(ec) || ec) {
        continue;
      }
      const auto name = entry.path().filename().string();
      if (name.rfind("thermal_zone", 0) != 0) {
        continue;
      }
      std::ifstream typeFile{entry.path() / "type"};
      std::string type;
      std::getline(typeFile, type);
      std::transform(type.begin(), type.end(), type.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      const bool cpuType = type.find("cpu") != std::string::npos ||
                           type.find("package") != std::string::npos ||
                           type.find("pkg") != std::string::npos;
      if (!cpuType) {
        continue;
      }
      std::ifstream tempFile{entry.path() / "temp"};
      std::string raw;
      std::getline(tempFile, raw);
      if (ParseThermalTemperature(raw, temperatureCelsius)) {
        return true;
      }
    }
  } catch (...) {
    return false;
  }
  return false;
}

inline bool IsSystemTimeValidUnixSeconds(int64_t unixSeconds) noexcept {
  // A synchronized wall clock is expected to be after the beginning of 2020.
  // This is deliberately independent from VMX monotonic/FPGATime and RTC.
  return unixSeconds >= 1577836800;
}

inline bool ReadSystemUnixSeconds(int64_t& unixSeconds) noexcept {
  try {
    const auto now = std::chrono::system_clock::now();
    unixSeconds = std::chrono::duration_cast<std::chrono::seconds>(
                      now.time_since_epoch())
                      .count();
    return true;
  } catch (...) {
    unixSeconds = 0;
    return false;
  }
}

inline bool ReadSystemTimeValidNow() noexcept {
  int64_t seconds = 0;
  return ReadSystemUnixSeconds(seconds) && IsSystemTimeValidUnixSeconds(seconds);
}

}  // namespace hal::vmx
