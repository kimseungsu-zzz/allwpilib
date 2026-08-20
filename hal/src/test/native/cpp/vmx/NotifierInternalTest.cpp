// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <thread>

#include "../../../../main/native/vmx/NotifierInternal.h"

namespace hal::vmx {
namespace {

using namespace std::chrono_literals;

struct ManualClock {
  std::atomic<uint64_t> now{0};

  uint64_t Read() const noexcept {
    return now.load(std::memory_order_acquire);
  }
};

TEST(NotifierInternalTest, ImmediateAndFutureAlarmsReturnHardwareDomainTime) {
  ManualClock clock;
  NotifierScheduler scheduler{[&] { return clock.Read(); }};
  auto state = std::make_shared<NotifierState>();
  ASSERT_EQ(scheduler.Add(state), NotifierResult::kOk);

  ASSERT_EQ(state->Update(0), NotifierResult::kOk);
  scheduler.Wake();
  auto immediate = std::async(std::launch::async, [&] { return state->Wait(); });
  ASSERT_EQ(immediate.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(immediate.get(), 0U);

  clock.now.store(100, std::memory_order_release);
  ASSERT_EQ(state->Update(250), NotifierResult::kOk);
  auto future = std::async(std::launch::async, [&] { return state->Wait(); });
  EXPECT_EQ(future.wait_for(20ms), std::future_status::timeout);
  clock.now.store(250, std::memory_order_release);
  scheduler.Wake();
  ASSERT_EQ(future.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(future.get(), 250U);
}

TEST(NotifierInternalTest, UpdatingReplacesEarlierAndLaterAlarms) {
  ManualClock clock;
  NotifierScheduler scheduler{[&] { return clock.Read(); }};
  auto state = std::make_shared<NotifierState>();
  ASSERT_EQ(scheduler.Add(state), NotifierResult::kOk);

  clock.now.store(100, std::memory_order_release);
  ASSERT_EQ(state->Update(1000), NotifierResult::kOk);
  ASSERT_EQ(state->Update(200), NotifierResult::kOk);
  auto earlier = std::async(std::launch::async, [&] { return state->Wait(); });
  clock.now.store(200, std::memory_order_release);
  scheduler.Wake();
  ASSERT_EQ(earlier.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(earlier.get(), 200U);

  ASSERT_EQ(state->Update(1000), NotifierResult::kOk);
  auto later = std::async(std::launch::async, [&] { return state->Wait(); });
  clock.now.store(999, std::memory_order_release);
  scheduler.Wake();
  EXPECT_EQ(later.wait_for(20ms), std::future_status::timeout);
  clock.now.store(1000, std::memory_order_release);
  scheduler.Wake();
  ASSERT_EQ(later.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(later.get(), 1000U);
}

TEST(NotifierInternalTest, CancelDoesNotReleaseWaiterButUpdateCanRearm) {
  ManualClock clock;
  NotifierScheduler scheduler{[&] { return clock.Read(); }};
  auto state = std::make_shared<NotifierState>();
  ASSERT_EQ(scheduler.Add(state), NotifierResult::kOk);
  ASSERT_EQ(state->Update(100), NotifierResult::kOk);
  auto waiter = std::async(std::launch::async, [&] { return state->Wait(); });

  ASSERT_EQ(state->Cancel(), NotifierResult::kOk);
  scheduler.Wake();
  EXPECT_EQ(waiter.wait_for(20ms), std::future_status::timeout);

  ASSERT_EQ(state->Update(50), NotifierResult::kOk);
  clock.now.store(50, std::memory_order_release);
  scheduler.Wake();
  ASSERT_EQ(waiter.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(waiter.get(), 50U);
}

TEST(NotifierInternalTest, StopIsPermanentAndReleasesWaiter) {
  NotifierScheduler scheduler{[] { return uint64_t{0}; }};
  auto state = std::make_shared<NotifierState>();
  ASSERT_EQ(scheduler.Add(state), NotifierResult::kOk);
  ASSERT_EQ(state->Update(100), NotifierResult::kOk);
  auto waiter = std::async(std::launch::async, [&] { return state->Wait(); });

  state->Stop();
  scheduler.Wake();
  ASSERT_EQ(waiter.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(waiter.get(), 0U);
  EXPECT_EQ(state->Update(0), NotifierResult::kStopped);
}

TEST(NotifierInternalTest, SpuriousWakeDoesNotFireEarly) {
  ManualClock clock;
  NotifierScheduler scheduler{[&] { return clock.Read(); }};
  auto state = std::make_shared<NotifierState>();
  ASSERT_EQ(scheduler.Add(state), NotifierResult::kOk);
  ASSERT_EQ(state->Update(1000), NotifierResult::kOk);
  auto waiter = std::async(std::launch::async, [&] { return state->Wait(); });

  scheduler.Wake();
  EXPECT_EQ(waiter.wait_for(20ms), std::future_status::timeout);
  clock.now.store(1000, std::memory_order_release);
  scheduler.Wake();
  ASSERT_EQ(waiter.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(waiter.get(), 1000U);
}

TEST(NotifierInternalTest, AlarmOrderingSurvivesUint64Rollover) {
  ManualClock clock;
  NotifierScheduler scheduler{[&] { return clock.Read(); }};
  auto state = std::make_shared<NotifierState>();
  ASSERT_EQ(scheduler.Add(state), NotifierResult::kOk);
  clock.now.store(std::numeric_limits<uint64_t>::max() - 5,
                  std::memory_order_release);
  ASSERT_EQ(state->Update(3), NotifierResult::kOk);
  auto waiter = std::async(std::launch::async, [&] { return state->Wait(); });
  scheduler.Wake();
  EXPECT_EQ(waiter.wait_for(20ms), std::future_status::timeout);
  clock.now.store(3, std::memory_order_release);
  scheduler.Wake();
  ASSERT_EQ(waiter.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(waiter.get(), 3U);
}

TEST(NotifierInternalTest, SameDeadlineWakesAllNotifiers) {
  ManualClock clock;
  NotifierScheduler scheduler{[&] { return clock.Read(); }};
  auto first = std::make_shared<NotifierState>();
  auto second = std::make_shared<NotifierState>();
  ASSERT_EQ(scheduler.Add(first), NotifierResult::kOk);
  ASSERT_EQ(scheduler.Add(second), NotifierResult::kOk);
  ASSERT_EQ(first->Update(100), NotifierResult::kOk);
  ASSERT_EQ(second->Update(100), NotifierResult::kOk);
  auto firstWaiter =
      std::async(std::launch::async, [&] { return first->Wait(); });
  auto secondWaiter =
      std::async(std::launch::async, [&] { return second->Wait(); });
  clock.now.store(100, std::memory_order_release);
  scheduler.Wake();
  ASSERT_EQ(firstWaiter.wait_for(500ms), std::future_status::ready);
  ASSERT_EQ(secondWaiter.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(firstWaiter.get(), 100U);
  EXPECT_EQ(secondWaiter.get(), 100U);
}

TEST(NotifierInternalTest, UsesOneHardwareTimerForGlobalEarliestDeadline) {
  ManualClock clock;
  std::atomic<int> arms{0};
  std::atomic<int> disarms{0};
  std::atomic<bool> parameterValid{true};
  NotifierScheduler scheduler{
      [&] { return clock.Read(); }, {},
      [&](uint64_t, void* parameter) {
        parameterValid.store(parameter != nullptr, std::memory_order_release);
        ++arms;
        return true;
      },
      [&] { ++disarms; }};
  auto first = std::make_shared<NotifierState>();
  auto second = std::make_shared<NotifierState>();
  ASSERT_EQ(scheduler.Add(first), NotifierResult::kOk);
  ASSERT_EQ(scheduler.Add(second), NotifierResult::kOk);
  clock.now.store(100, std::memory_order_release);
  ASSERT_EQ(first->Update(1000000), NotifierResult::kOk);
  ASSERT_EQ(second->Update(2000000), NotifierResult::kOk);
  scheduler.Wake();
  const auto waitFor = [](auto&& predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(1ms);
    }
    return predicate();
  };
  ASSERT_TRUE(waitFor([&] { return arms.load() >= 1; }));
  EXPECT_EQ(arms.load(), 1);

  ASSERT_EQ(second->Update(500000), NotifierResult::kOk);
  scheduler.Wake();
  ASSERT_TRUE(waitFor([&] { return arms.load() >= 2; }));
  EXPECT_GE(arms.load(), 2);
  ASSERT_TRUE(waitFor([&] { return disarms.load() >= 1; }));
  EXPECT_TRUE(parameterValid.load(std::memory_order_acquire));
}

TEST(NotifierInternalTest, SchedulerShutdownStopsWaiters) {
  NotifierScheduler scheduler{[] { return uint64_t{0}; }};
  auto state = std::make_shared<NotifierState>();
  ASSERT_EQ(scheduler.Add(state), NotifierResult::kOk);
  ASSERT_EQ(state->Update(1000), NotifierResult::kOk);
  auto waiter = std::async(std::launch::async, [&] { return state->Wait(); });
  scheduler.Shutdown();
  ASSERT_EQ(waiter.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(waiter.get(), 0U);
  scheduler.Shutdown();
}

TEST(NotifierInternalTest, AllocationAndDoubleCleanAreSafe) {
  ManualClock clock;
  NotifierManager manager{[&] { return clock.Read(); }};
  auto first = manager.Allocate();
  auto second = manager.Allocate();
  ASSERT_EQ(first.result, NotifierResult::kOk);
  ASSERT_EQ(second.result, NotifierResult::kOk);
  EXPECT_NE(first.handle, second.handle);

  ASSERT_EQ(manager.Update(second.handle, 1000), NotifierResult::kOk);
  auto waiter = std::async(std::launch::async,
                           [&] { return manager.Wait(second.handle).second; });
  manager.Free(second.handle);
  ASSERT_EQ(waiter.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(waiter.get(), 0U);
  manager.Free(second.handle);
  manager.Free(first.handle);
  manager.Shutdown();
  manager.Shutdown();
}

TEST(NotifierInternalTest, InvalidHandlesOnlyReturnHandleErrors) {
  NotifierManager manager{[] { return uint64_t{0}; }};
  EXPECT_EQ(manager.Wait(HAL_kInvalidHandle).first,
            NotifierResult::kInvalidHandle);
  EXPECT_EQ(manager.Update(HAL_kInvalidHandle, 0),
            NotifierResult::kInvalidHandle);
  EXPECT_EQ(manager.Cancel(HAL_kInvalidHandle),
            NotifierResult::kInvalidHandle);
  EXPECT_EQ(manager.Stop(HAL_kInvalidHandle), NotifierResult::kInvalidHandle);
  EXPECT_EQ(manager.SetName(HAL_kInvalidHandle, "invalid"),
            NotifierResult::kInvalidHandle);
}

TEST(NotifierInternalTest, ConcurrentMutationAndStopAreLifetimeSafe) {
  ManualClock clock;
  NotifierScheduler scheduler{[&] { return clock.Read(); }};
  auto state = std::make_shared<NotifierState>();
  ASSERT_EQ(scheduler.Add(state), NotifierResult::kOk);
  std::thread updater{[&] {
    for (uint64_t i = 1; i <= 100; ++i) {
      state->Update(i);
    }
  }};
  std::thread canceller{[&] {
    for (int i = 0; i < 100; ++i) {
      state->Cancel();
    }
  }};
  std::thread waker{[&] {
    for (int i = 0; i < 100; ++i) {
      scheduler.Wake();
    }
  }};
  updater.join();
  canceller.join();
  waker.join();
  state->Stop();
  EXPECT_EQ(state->Update(0), NotifierResult::kStopped);
  scheduler.Shutdown();
}

TEST(NotifierInternalTest, PriorityValidationAndPermissionFailure) {
  std::atomic<int> calls{0};
  NotifierScheduler scheduler{
      [] { return uint64_t{0}; },
      [&](std::thread::native_handle_type, bool, int32_t, int32_t& status) {
        ++calls;
        status = HAL_THREAD_PRIORITY_ERROR;
        return false;
      }};
  EXPECT_EQ(scheduler.SetPriority(true, 0), NotifierResult::kPriorityRange);
  EXPECT_EQ(scheduler.SetPriority(false, 42), NotifierResult::kOk);

  ASSERT_EQ(scheduler.SetPriority(true, 50), NotifierResult::kOk);
  auto state = std::make_shared<NotifierState>();
  EXPECT_EQ(scheduler.Add(state), NotifierResult::kPriorityFailure);
  EXPECT_GE(calls.load(), 1);
  EXPECT_EQ(state->Wait(), 0U);
}

}  // namespace
}  // namespace hal::vmx
