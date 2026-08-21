// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <vector>

#include "../../../../main/native/vmx/VMXWatchdogInternal.h"

namespace hal::vmx {
namespace {

struct WatchdogCalls final {
  int setTimeout = 0;
  int setManaged = 0;
  int setEnabled = 0;
  int feed = 0;
  int expire = 0;
  std::vector<bool> enabledValues;
  bool flex = false;
  bool highCurrent = false;
  bool comm = true;
};

VMXWatchdogBackend MakeBackend(WatchdogCalls& calls) {
  VMXWatchdogBackend backend;
  backend.setTimeout = [&calls](uint16_t, int32_t& status) {
    ++calls.setTimeout;
    status = HAL_SUCCESS;
    return true;
  };
  backend.setManagedOutputs = [&calls](bool flex, bool highCurrent, bool comm,
                                       int32_t& status) {
    ++calls.setManaged;
    calls.flex = flex;
    calls.highCurrent = highCurrent;
    calls.comm = comm;
    status = HAL_SUCCESS;
    return true;
  };
  backend.setEnabled = [&calls](bool enabled, int32_t& status) {
    ++calls.setEnabled;
    calls.enabledValues.push_back(enabled);
    status = HAL_SUCCESS;
    return true;
  };
  backend.feed = [&calls](int32_t& status) {
    ++calls.feed;
    status = HAL_SUCCESS;
    return true;
  };
  backend.expireNow = [&calls](int32_t& status) {
    ++calls.expire;
    status = HAL_SUCCESS;
    return true;
  };
  return backend;
}

TEST(VMXWatchdogTest, ConfiguresManagedOutputGroupsAndFeedsOnlyWhenSafe) {
  WatchdogCalls calls;
  bool safe = false;
  VMXHardwareWatchdog watchdog{MakeBackend(calls),
                               [&safe](uint64_t) { return safe; }, {}};
  ASSERT_TRUE(watchdog.Configure(100, true, true, false, false));
  EXPECT_EQ(calls.setTimeout, 1);
  EXPECT_EQ(calls.setManaged, 1);
  EXPECT_TRUE(calls.flex);
  EXPECT_TRUE(calls.highCurrent);
  EXPECT_FALSE(calls.comm);
  EXPECT_FALSE(watchdog.Tick(100));
  EXPECT_EQ(calls.expire, 1);
  safe = true;
  EXPECT_TRUE(watchdog.Tick(200));
  EXPECT_EQ(calls.feed, 1);
  EXPECT_EQ(watchdog.LastFeed(), 200U);
}

TEST(VMXWatchdogTest, UnsafeTransitionExpiresOnlyOnceUntilRecovery) {
  WatchdogCalls calls;
  bool safe = true;
  VMXHardwareWatchdog watchdog{MakeBackend(calls),
                               [&safe](uint64_t) { return safe; }};
  ASSERT_TRUE(watchdog.Configure(100, true, true, false, false));
  EXPECT_TRUE(watchdog.Tick(1));
  safe = false;
  EXPECT_FALSE(watchdog.Tick(2));
  EXPECT_FALSE(watchdog.Tick(3));
  EXPECT_EQ(calls.expire, 1);
  safe = true;
  EXPECT_TRUE(watchdog.Tick(4));
  safe = false;
  EXPECT_FALSE(watchdog.Tick(5));
  EXPECT_EQ(calls.expire, 2);
}

TEST(VMXWatchdogTest, FeedFailureIsAWatchdogRuntimeFault) {
  WatchdogCalls calls;
  auto backend = MakeBackend(calls);
  backend.feed = [&calls](int32_t& status) {
    ++calls.feed;
    status = INCOMPATIBLE_STATE;
    return false;
  };
  VMXHardwareWatchdog watchdog{std::move(backend), [](uint64_t) {
    return true;
  }};
  ASSERT_TRUE(watchdog.Configure(100, true, true, false, false));
  EXPECT_FALSE(watchdog.Tick(1));
  EXPECT_EQ(calls.feed, 1);
  EXPECT_EQ(calls.expire, 1);
  EXPECT_NE(watchdog.LastError(), HAL_SUCCESS);
}

TEST(VMXWatchdogTest, ShutdownExpiresThenDisablesAndIsIdempotent) {
  WatchdogCalls calls;
  VMXHardwareWatchdog watchdog{MakeBackend(calls), [](uint64_t) { return true; }};
  ASSERT_TRUE(watchdog.Configure(100, true, true, false, false));
  watchdog.Shutdown();
  EXPECT_EQ(calls.expire, 1);
  ASSERT_EQ(calls.enabledValues.size(), 2U);
  EXPECT_TRUE(calls.enabledValues[0]);
  EXPECT_FALSE(calls.enabledValues[1]);
  watchdog.Shutdown();
  EXPECT_EQ(calls.expire, 1);
  EXPECT_FALSE(watchdog.Tick(10));
}

TEST(VMXWatchdogTest, ConfigurationFailureDoesNotClaimHardwareEnabled) {
  WatchdogCalls calls;
  auto backend = MakeBackend(calls);
  backend.setTimeout = [](uint16_t, int32_t& status) {
    status = INCOMPATIBLE_STATE;
    return false;
  };
  VMXHardwareWatchdog watchdog{std::move(backend), [](uint64_t) { return true; }};
  EXPECT_FALSE(watchdog.Configure(100, true, true, false, false));
  EXPECT_FALSE(watchdog.IsConfigured());
  EXPECT_EQ(calls.setEnabled, 0);
}

}  // namespace
}  // namespace hal::vmx
