// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cstdint>
#include <functional>
#include <utility>

namespace hal::vmx {

using VMXTimeReader = std::function<bool(uint64_t&)>;

/** Small seam around the VMX monotonic hardware clock for host tests. */
class VMXTimeSource final {
 public:
  explicit VMXTimeSource(VMXTimeReader reader) : m_reader{std::move(reader)} {}

  bool Read(uint64_t& timestampMicroseconds) const noexcept {
    try {
      return m_reader && m_reader(timestampMicroseconds);
    } catch (...) {
      timestampMicroseconds = 0;
      return false;
    }
  }

 private:
  VMXTimeReader m_reader;
};

/** Reads VMXTime::GetCurrentTotalMicroseconds() in the HAL time domain. */
uint64_t GetTimeMicroseconds(int32_t* status);

}  // namespace hal::vmx
