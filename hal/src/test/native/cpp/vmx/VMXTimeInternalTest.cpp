// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "../../../../main/native/vmx/VMXTimeInternal.h"

namespace hal::vmx {
namespace {

TEST(VMXTimeInternalTest, ReadsMonotonic64BitHardwareValues) {
  const std::vector<uint64_t> values{0x00000000ffffffffULL,
                                     0x0000000100000000ULL,
                                     0x123456789abcdef0ULL};
  size_t index = 0;
  VMXTimeSource source{[&](uint64_t& value) {
    value = values[index++];
    return true;
  }};

  uint64_t previous = 0;
  for (const auto expected : values) {
    uint64_t actual = 0;
    ASSERT_TRUE(source.Read(actual));
    EXPECT_EQ(actual, expected);
    EXPECT_GE(actual, previous);
    previous = actual;
  }
}

TEST(VMXTimeInternalTest, PropagatesUnavailableClock) {
  VMXTimeSource source{[](uint64_t&) { return false; }};
  uint64_t timestamp = 42;
  EXPECT_FALSE(source.Read(timestamp));
  EXPECT_EQ(timestamp, 42U);
}

TEST(VMXTimeInternalTest, ConvertsReaderExceptionsToFailure) {
  VMXTimeSource source{[](uint64_t&) -> bool {
    throw std::runtime_error{"clock unavailable"};
  }};
  uint64_t timestamp = 42;
  EXPECT_FALSE(source.Read(timestamp));
  EXPECT_EQ(timestamp, 0U);
}

}  // namespace
}  // namespace hal::vmx
