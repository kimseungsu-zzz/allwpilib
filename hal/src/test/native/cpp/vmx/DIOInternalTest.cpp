// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "../../../../main/native/vmx/DIOInternal.h"

namespace hal::vmx {
namespace {

struct FakeDIOHardware {
  std::mutex mutex;
  std::array<bool, kNumDIOChannels> values{};
  std::atomic_int creates{0};
  std::atomic_int destroys{0};
  std::atomic_int failedCreatesRemaining{0};
  std::atomic_bool failSet{false};
  std::atomic_bool failGet{false};
  std::atomic_bool failPulse{false};
  std::atomic_bool pulsing{false};
  std::atomic_uint32_t lastPulseMicroseconds{0};
};

class FakeDIOBackend final : public DIOBackend {
 public:
  FakeDIOBackend(std::shared_ptr<FakeDIOHardware> hardware, int32_t channel)
      : m_hardware{std::move(hardware)}, m_channel{channel} {}

  ~FakeDIOBackend() override { ++m_hardware->destroys; }

  bool Set(bool value) noexcept override {
    if (m_hardware->failSet) {
      return false;
    }
    std::scoped_lock lock{m_hardware->mutex};
    m_hardware->values[m_channel] = value;
    return true;
  }

  bool Get(bool& value) noexcept override {
    if (m_hardware->failGet) {
      value = false;
      return false;
    }
    std::scoped_lock lock{m_hardware->mutex};
    value = m_hardware->values[m_channel];
    return true;
  }

  bool Pulse(uint32_t microseconds) noexcept override {
    if (m_hardware->failPulse) {
      return false;
    }
    m_hardware->lastPulseMicroseconds = microseconds;
    m_hardware->pulsing = true;
    return true;
  }

  bool IsPulsing(bool& isPulsing) noexcept override {
    if (m_hardware->failPulse) {
      isPulsing = false;
      return false;
    }
    isPulsing = m_hardware->pulsing;
    return true;
  }

 private:
  std::shared_ptr<FakeDIOHardware> m_hardware;
  int32_t m_channel;
};

struct DIOFixture {
  DIOFixture()
      : manager{
            [this](int32_t channel, bool input) {
              static_cast<void>(input);
              ++hardware->creates;
              int remaining = hardware->failedCreatesRemaining.load();
              while (remaining > 0 &&
                     !hardware->failedCreatesRemaining.compare_exchange_weak(
                         remaining, remaining - 1)) {
              }
              if (remaining > 0) {
                return std::unique_ptr<DIOBackend>{};
              }
              return std::unique_ptr<DIOBackend>{
                  std::make_unique<FakeDIOBackend>(hardware, channel)};
            },
            registry} {}

  std::shared_ptr<FakeDIOHardware> hardware =
      std::make_shared<FakeDIOHardware>();
  DigitalChannelRegistry registry;
  DIOManager manager;
};

TEST(VMXDIOTest, ChannelRangeIsZeroThroughTwentyNine) {
  EXPECT_FALSE(IsDIOChannelValid(-1));
  EXPECT_TRUE(IsDIOChannelValid(0));
  EXPECT_TRUE(IsDIOChannelValid(21));
  EXPECT_TRUE(IsDIOChannelValid(22));
  EXPECT_TRUE(IsDIOChannelValid(29));
  EXPECT_FALSE(IsDIOChannelValid(30));
}

TEST(VMXDIOTest, CapabilityProviderEnforcesHighCurrentAndCommDioRules) {
  auto hardware = std::make_shared<FakeDIOHardware>();
  DigitalChannelRegistry registry;
  VMXCapabilityProvider capabilities{[](int32_t physical,
                                        VMXCapability capability) {
    if (physical >= 12 && physical <= 21) {
      return capability == VMXCapability::kDigitalOutput ||
             capability == VMXCapability::kPWMGenerator;
    }
    if (physical >= 26 && physical <= 33) {
      return capability == VMXCapability::kDigitalInput ||
             capability == VMXCapability::kInterruptInput;
    }
    return true;
  }};
  DIOManager manager{
      [hardware](int32_t channel, bool) {
        return std::unique_ptr<DIOBackend>{
            std::make_unique<FakeDIOBackend>(hardware, channel)};
      },
      registry, &capabilities};

  EXPECT_EQ(manager.Allocate(12, false, "high-current output").result,
            DIOResult::kOk);
  EXPECT_EQ(manager.Allocate(13, true, "high-current input").result,
            DIOResult::kUnsupportedCapability);
  auto commInput = manager.Allocate(22, true, "comm input");
  ASSERT_EQ(commInput.result, DIOResult::kOk);
  EXPECT_EQ(manager.SetDirection(commInput.handle, false),
            DIOResult::kUnsupportedCapability);
}

TEST(VMXDIOTest, AllocatesInputAndOutputAndRejectsDuplicate) {
  DIOFixture fixture;
  auto output = fixture.manager.Allocate(5, false, "first allocation");
  ASSERT_EQ(output.result, DIOResult::kOk);

  auto duplicate = fixture.manager.Allocate(5, true, "second allocation");
  EXPECT_EQ(duplicate.result, DIOResult::kAlreadyAllocated);
  EXPECT_EQ(duplicate.previousAllocation, "first allocation");

  auto input = fixture.manager.Allocate(6, true, "input allocation");
  EXPECT_EQ(input.result, DIOResult::kOk);
}

TEST(VMXDIOTest, FailedHardwareAllocationRollsBackHandle) {
  DIOFixture fixture;
  fixture.hardware->failedCreatesRemaining = 1;
  auto failed = fixture.manager.Allocate(3, false, "failed");
  EXPECT_EQ(failed.result, DIOResult::kHardwareFailure);

  auto retry = fixture.manager.Allocate(3, false, "retry");
  EXPECT_EQ(retry.result, DIOResult::kOk);
}

TEST(VMXDIOTest, InvalidHandlesAreRejected) {
  DIOFixture fixture;
  EXPECT_EQ(fixture.manager.Set(HAL_kInvalidHandle, true),
            DIOResult::kInvalidHandle);
  EXPECT_EQ(fixture.manager.GetValue(HAL_kInvalidHandle).first,
            DIOResult::kInvalidHandle);
  EXPECT_EQ(fixture.manager.SetDirection(HAL_kInvalidHandle, true),
            DIOResult::kInvalidHandle);
}

TEST(VMXDIOTest, InputCannotBeWritten) {
  DIOFixture fixture;
  auto allocation = fixture.manager.Allocate(2, true, "input");
  ASSERT_EQ(allocation.result, DIOResult::kOk);
  EXPECT_EQ(fixture.manager.Set(allocation.handle, true),
            DIOResult::kInputChannel);
}

TEST(VMXDIOTest, OutputCanBeWrittenAndRead) {
  DIOFixture fixture;
  auto allocation = fixture.manager.Allocate(4, false, "output");
  ASSERT_EQ(allocation.result, DIOResult::kOk);
  ASSERT_EQ(fixture.manager.Set(allocation.handle, true), DIOResult::kOk);
  auto [result, value] = fixture.manager.GetValue(allocation.handle);
  EXPECT_EQ(result, DIOResult::kOk);
  EXPECT_TRUE(value);
}

TEST(VMXDIOTest, DriverReadAndWriteFailuresAreReported) {
  DIOFixture fixture;
  auto allocation = fixture.manager.Allocate(12, false, "driver errors");
  ASSERT_EQ(allocation.result, DIOResult::kOk);

  fixture.hardware->failSet = true;
  EXPECT_EQ(fixture.manager.Set(allocation.handle, true),
            DIOResult::kHardwareFailure);
  fixture.hardware->failGet = true;
  EXPECT_EQ(fixture.manager.GetValue(allocation.handle).first,
            DIOResult::kHardwareFailure);
}

TEST(VMXDIOTest, DirectionChangeRecreatesBackendAndKeepsHandle) {
  DIOFixture fixture;
  auto allocation = fixture.manager.Allocate(7, false, "direction");
  ASSERT_EQ(allocation.result, DIOResult::kOk);
  ASSERT_EQ(fixture.manager.SetDirection(allocation.handle, true),
            DIOResult::kOk);
  auto [result, input] = fixture.manager.GetDirection(allocation.handle);
  EXPECT_EQ(result, DIOResult::kOk);
  EXPECT_TRUE(input);
  EXPECT_EQ(fixture.hardware->creates, 2);
  EXPECT_EQ(fixture.hardware->destroys, 1);
}

TEST(VMXDIOTest, FailedDirectionChangeRestoresPreviousDirection) {
  DIOFixture fixture;
  auto allocation = fixture.manager.Allocate(8, false, "rollback");
  ASSERT_EQ(allocation.result, DIOResult::kOk);
  fixture.hardware->failedCreatesRemaining = 1;

  EXPECT_EQ(fixture.manager.SetDirection(allocation.handle, true),
            DIOResult::kHardwareFailure);
  auto [result, input] = fixture.manager.GetDirection(allocation.handle);
  EXPECT_EQ(result, DIOResult::kOk);
  EXPECT_FALSE(input);
}

TEST(VMXDIOTest, FailedDirectionRollbackFaultsHandle) {
  DIOFixture fixture;
  auto allocation = fixture.manager.Allocate(9, false, "fault");
  ASSERT_EQ(allocation.result, DIOResult::kOk);
  fixture.hardware->failedCreatesRemaining = 2;

  EXPECT_EQ(fixture.manager.SetDirection(allocation.handle, true),
            DIOResult::kRollbackFailure);
  EXPECT_EQ(fixture.manager.GetDirection(allocation.handle).first,
            DIOResult::kHardwareFailure);
}

TEST(VMXDIOTest, FreeInvalidatesHandleAndDoubleFreeIsSafe) {
  DIOFixture fixture;
  auto allocation = fixture.manager.Allocate(10, false, "free");
  ASSERT_EQ(allocation.result, DIOResult::kOk);

  fixture.manager.Free(allocation.handle);
  fixture.manager.Free(allocation.handle);
  EXPECT_EQ(fixture.manager.GetValue(allocation.handle).first,
            DIOResult::kInvalidHandle);

  auto replacement = fixture.manager.Allocate(10, false, "replacement");
  EXPECT_EQ(replacement.result, DIOResult::kOk);
}

TEST(VMXDIOTest, ConcurrentOperationsAndFreeKeepDriverLifetimeSafe) {
  DIOFixture fixture;
  auto allocation = fixture.manager.Allocate(11, false, "concurrent");
  ASSERT_EQ(allocation.result, DIOResult::kOk);

  std::vector<std::thread> threads;
  for (int i = 0; i < 4; ++i) {
    threads.emplace_back([&, i] {
      for (int j = 0; j < 100; ++j) {
        fixture.manager.Set(allocation.handle, (i + j) % 2 != 0);
        fixture.manager.GetValue(allocation.handle);
        fixture.manager.GetDirection(allocation.handle);
      }
    });
  }
  threads.emplace_back([&] {
    for (int i = 0; i < 100; ++i) {
      fixture.manager.SetDirection(allocation.handle, (i % 2) != 0);
    }
  });
  threads.emplace_back([&] { fixture.manager.Free(allocation.handle); });
  for (auto& thread : threads) {
    thread.join();
  }

  EXPECT_EQ(fixture.manager.GetValue(allocation.handle).first,
            DIOResult::kInvalidHandle);
}

TEST(VMXDIOTest, PulseAndPulseReadbackUseOutputHardware) {
  DIOFixture fixture;
  auto output = fixture.manager.Allocate(13, false, "pulse output");
  auto input = fixture.manager.Allocate(14, true, "pulse input");
  ASSERT_EQ(output.result, DIOResult::kOk);
  ASSERT_EQ(input.result, DIOResult::kOk);

  EXPECT_EQ(fixture.manager.Pulse(output.handle, 2500), DIOResult::kOk);
  EXPECT_EQ(fixture.hardware->lastPulseMicroseconds, 2500u);
  EXPECT_TRUE(fixture.manager.IsPulsing(output.handle).second);
  EXPECT_TRUE(fixture.manager.IsAnyPulsing().second);
  EXPECT_EQ(fixture.manager.Pulse(input.handle, 10), DIOResult::kInputChannel);

  fixture.hardware->pulsing = false;
  EXPECT_FALSE(fixture.manager.IsAnyPulsing().second);
}

TEST(VMXDIOTest, PulseMultipleValidatesEveryAllocatedOutput) {
  DIOFixture fixture;
  auto first = fixture.manager.Allocate(15, false, "pulse one");
  auto second = fixture.manager.Allocate(16, false, "pulse two");
  ASSERT_EQ(first.result, DIOResult::kOk);
  ASSERT_EQ(second.result, DIOResult::kOk);
  EXPECT_EQ(fixture.manager.PulseMultiple((1u << 15) | (1u << 16), 800),
            DIOResult::kOk);
  EXPECT_EQ(fixture.hardware->lastPulseMicroseconds, 800u);
  EXPECT_EQ(fixture.manager.PulseMultiple(1u << 17, 800),
            DIOResult::kInvalidHandle);
}

TEST(VMXDIOTest, EncoderClaimSuspendsAndRestoresDIOInputsAtomically) {
  DIOFixture fixture;
  auto sourceA = fixture.manager.Allocate(0, true, "encoder source A");
  auto sourceB = fixture.manager.Allocate(1, true, "encoder source B");
  ASSERT_EQ(sourceA.result, DIOResult::kOk);
  ASSERT_EQ(sourceB.result, DIOResult::kOk);
  EXPECT_EQ(fixture.hardware->creates, 2);

  auto claim = fixture.manager.ClaimEncoderSources(
      sourceA.handle, sourceB.handle, "encoder allocation");
  ASSERT_EQ(claim.result, DIOResult::kOk);
  EXPECT_EQ(claim.channelA, 0);
  EXPECT_EQ(claim.channelB, 1);
  EXPECT_EQ(fixture.hardware->destroys, 2);
  EXPECT_EQ(fixture.manager.GetValue(sourceA.handle).first,
            DIOResult::kHardwareFailure);
  EXPECT_FALSE(fixture.registry
                   .Reserve(0, DigitalChannelOwner::kPWM, "conflicting PWM")
                   .reserved);
  EXPECT_EQ(fixture.manager
                .ClaimEncoderSources(sourceA.handle, sourceB.handle,
                                     "overlapping encoder")
                .result,
            DIOResult::kAlreadyAllocated);

  fixture.manager.ReleaseEncoderSources(sourceA.handle, sourceB.handle,
                                        claim.channelA, claim.channelB);
  EXPECT_EQ(fixture.hardware->creates, 4);
  EXPECT_EQ(fixture.manager.GetValue(sourceA.handle).first, DIOResult::kOk);
  EXPECT_EQ(fixture.manager.GetValue(sourceB.handle).first, DIOResult::kOk);
}

TEST(VMXDIOTest, EncoderClaimRejectsOutputsWithoutChangingOwnership) {
  DIOFixture fixture;
  auto output = fixture.manager.Allocate(2, false, "output");
  auto input = fixture.manager.Allocate(3, true, "input");
  ASSERT_EQ(output.result, DIOResult::kOk);
  ASSERT_EQ(input.result, DIOResult::kOk);

  auto claim = fixture.manager.ClaimEncoderSources(
      output.handle, input.handle, "invalid encoder");
  EXPECT_EQ(claim.result, DIOResult::kOutputChannel);
  EXPECT_EQ(fixture.manager.Set(output.handle, true), DIOResult::kOk);
  EXPECT_EQ(fixture.manager.GetValue(input.handle).first, DIOResult::kOk);
}

TEST(VMXDIOTest, CounterClaimSharesAtomicSourceOwnershipRegistry) {
  DIOFixture fixture;
  auto sourceA = fixture.manager.Allocate(0, true, "counter source A");
  auto sourceB = fixture.manager.Allocate(1, true, "counter source B");
  ASSERT_EQ(sourceA.result, DIOResult::kOk);
  ASSERT_EQ(sourceB.result, DIOResult::kOk);

  auto claim = fixture.manager.ClaimResourceSources(
      sourceA.handle, sourceB.handle, DigitalChannelOwner::kCounter,
      "counter allocation");
  ASSERT_EQ(claim.result, DIOResult::kOk);
  EXPECT_FALSE(fixture.registry
                   .Reserve(0, DigitalChannelOwner::kEncoder,
                            "conflicting encoder")
                   .reserved);

  fixture.manager.ReleaseResourceSources(
      sourceA.handle, sourceB.handle, claim.channelA, claim.channelB,
      DigitalChannelOwner::kCounter);
  EXPECT_EQ(fixture.manager.GetValue(sourceA.handle).first, DIOResult::kOk);
  EXPECT_EQ(fixture.manager.GetValue(sourceB.handle).first, DIOResult::kOk);
}

}  // namespace
}  // namespace hal::vmx
