// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "../../../../main/native/vmx/VMXRuntime.h"

namespace hal::vmx {
namespace {

class FakeSdkContext final : public SdkContext {
 public:
  FakeSdkContext(bool ready, std::atomic_int* destroys,
                 std::shared_ptr<VMXPi> context = {})
      : m_ready{ready}, m_destroys{destroys}, m_context{std::move(context)} {}

  ~FakeSdkContext() override {
    if (m_destroys) {
      ++*m_destroys;
    }
  }

  bool IsReady() const noexcept override { return m_ready; }
  std::shared_ptr<VMXPi> GetVMXPi() const noexcept override {
    return m_context;
  }
  std::string_view GetError() const noexcept override {
    return "fake initialization failure";
  }

 private:
  bool m_ready;
  std::atomic_int* m_destroys;
  std::shared_ptr<VMXPi> m_context;
};

TEST(VMXRuntimeTest, InitializesOnlyOnce) {
  std::atomic_int creates{0};
  Runtime runtime{[&] {
    ++creates;
    return std::make_unique<FakeSdkContext>(true, nullptr);
  }};

  EXPECT_TRUE(runtime.Initialize());
  EXPECT_TRUE(runtime.Initialize());
  EXPECT_EQ(creates, 1);
  EXPECT_EQ(runtime.GetState(), RuntimeState::kReady);
}

TEST(VMXRuntimeTest, ReturnsTheSameSharedSdkContext) {
  auto context = std::shared_ptr<VMXPi>{reinterpret_cast<VMXPi*>(1),
                                        [](VMXPi*) {}};
  Runtime runtime{[&] {
    return std::make_unique<FakeSdkContext>(true, nullptr, context);
  }};

  ASSERT_TRUE(runtime.Initialize());
  auto first = runtime.GetContext();
  auto second = runtime.GetContext();
  EXPECT_EQ(first.get(), context.get());
  EXPECT_EQ(second.get(), context.get());
}

TEST(VMXRuntimeTest, ConcurrentInitializationCallsFactoryOnce) {
  std::atomic_int creates{0};
  Runtime runtime{[&] {
    ++creates;
    std::this_thread::sleep_for(std::chrono::milliseconds{20});
    return std::make_unique<FakeSdkContext>(true, nullptr);
  }};

  std::vector<std::thread> threads;
  std::atomic_int successes{0};
  for (int i = 0; i < 8; ++i) {
    threads.emplace_back([&] {
      if (runtime.Initialize()) {
        ++successes;
      }
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }

  EXPECT_EQ(creates, 1);
  EXPECT_EQ(successes, 8);
}

TEST(VMXRuntimeTest, FailureIsVisibleAndCanBeShutDown) {
  std::atomic_int destroys{0};
  Runtime runtime{[&] {
    return std::make_unique<FakeSdkContext>(false, &destroys);
  }};

  EXPECT_FALSE(runtime.Initialize());
  EXPECT_EQ(runtime.GetState(), RuntimeState::kFailed);
  EXPECT_EQ(runtime.GetLastError(), "fake initialization failure");
  EXPECT_EQ(destroys, 1);

  runtime.Shutdown();
  runtime.Shutdown();
  EXPECT_EQ(runtime.GetState(), RuntimeState::kShutdown);
}

TEST(VMXRuntimeTest, ExceptionsDoNotEscapeInitialization) {
  Runtime runtime{[]() -> std::unique_ptr<SdkContext> {
    throw std::runtime_error{"fake SDK exception"};
  }};

  EXPECT_FALSE(runtime.Initialize());
  EXPECT_EQ(runtime.GetState(), RuntimeState::kFailed);
  EXPECT_EQ(runtime.GetLastError(), "fake SDK exception");
}

TEST(VMXRuntimeTest, ShutdownReleasesContextOnce) {
  std::atomic_int destroys{0};
  Runtime runtime{[&] {
    return std::make_unique<FakeSdkContext>(true, &destroys);
  }};

  ASSERT_TRUE(runtime.Initialize());
  runtime.Shutdown();
  runtime.Shutdown();

  EXPECT_EQ(destroys, 1);
  EXPECT_FALSE(runtime.Initialize());
}

}  // namespace
}  // namespace hal::vmx
