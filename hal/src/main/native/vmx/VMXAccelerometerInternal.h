// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>
#include <functional>
#include <mutex>
#include <utility>

#include "hal/Accelerometer.h"

namespace hal::vmx {

/**
 * Adapter-side state for the single VMX onboard accelerometer.
 *
 * The VMX SDK exposes calibrated raw acceleration in G, including gravity,
 * but does not expose an accelerometer range setter.  Active/standby is
 * therefore kept in this HAL state instead of stopping the shared AHRS.
 */
class BuiltInAccelerometerState final {
 public:
  using AxisReader = std::function<double(int)>;

  explicit BuiltInAccelerometerState(AxisReader reader = {})
      : m_reader{std::move(reader)} {}

  void SetActive(bool active) noexcept {
    std::scoped_lock lock{m_mutex};
    m_active = active;
  }

  void SetRange(HAL_AccelerometerRange range) noexcept {
    std::scoped_lock lock{m_mutex};
    if (range >= HAL_AccelerometerRange_k2G &&
        range <= HAL_AccelerometerRange_k8G) {
      // Retain the requested WPILib value for compatibility, while the
      // fixed-range VMX hardware remains unchanged.
      m_requestedRange = range;
    }
  }

  double GetAxis(int axis) const noexcept {
    AxisReader reader;
    {
      std::scoped_lock lock{m_mutex};
      if (!m_active || axis < 0 || axis > 2) {
        return 0.0;
      }
      reader = m_reader;
    }
    try {
      const auto value = reader ? reader(axis) : 0.0;
      return std::isfinite(value) ? value : 0.0;
    } catch (...) {
      return 0.0;
    }
  }

  bool IsActive() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_active;
  }

  HAL_AccelerometerRange GetRequestedRange() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_requestedRange;
  }

 private:
  mutable std::mutex m_mutex;
  AxisReader m_reader;
  bool m_active = true;
  HAL_AccelerometerRange m_requestedRange = HAL_AccelerometerRange_k2G;
};

BuiltInAccelerometerState& GetBuiltInAccelerometerState();

}  // namespace hal::vmx
