// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "../../../../main/native/vmx/SPIAutoInternal.h"

namespace hal::vmx {
namespace {

class ManualRateWaiter final : public SPIAutoWaiter {
 public:
  bool WaitUntil(uint64_t deadline, uint64_t& firedAt) noexcept override {
    std::unique_lock lock{m_mutex};
    m_deadline = deadline;
    m_cv.notify_all();
    m_cv.wait(lock, [this] { return m_stopped || m_signaled; });
    if (m_stopped) {
      return false;
    }
    firedAt = m_firedAt;
    m_signaled = false;
    return true;
  }

  void Stop() noexcept override {
    std::scoped_lock lock{m_mutex};
    m_stopped = true;
    m_cv.notify_all();
  }

  bool WaitForDeadline() {
    std::unique_lock lock{m_mutex};
    return m_cv.wait_for(lock, std::chrono::milliseconds{100},
                         [this] { return m_deadline != 0; });
  }

  void Fire(uint64_t timestamp) {
    std::scoped_lock lock{m_mutex};
    m_firedAt = timestamp;
    m_signaled = true;
    m_cv.notify_all();
  }

 private:
  std::mutex m_mutex;
  std::condition_variable m_cv;
  uint64_t m_deadline = 0;
  uint64_t m_firedAt = 0;
  bool m_signaled = false;
  bool m_stopped = false;
};

class ManualTriggerWaiter final : public SPIAutoTriggerWaiter {
 public:
  bool Wait() noexcept override {
    std::unique_lock lock{m_mutex};
    m_cv.wait(lock, [this] { return m_stopped || m_signaled; });
    if (m_stopped) {
      return false;
    }
    m_signaled = false;
    return true;
  }

  void Stop() noexcept override {
    std::scoped_lock lock{m_mutex};
    m_stopped = true;
    m_cv.notify_all();
  }

  void Fire() {
    std::scoped_lock lock{m_mutex};
    m_signaled = true;
    m_cv.notify_all();
  }

 private:
  std::mutex m_mutex;
  std::condition_variable m_cv;
  bool m_signaled = false;
  bool m_stopped = false;
};

class AutoManagerBackend final : public SPIBackend {
 public:
  bool Transaction(uint8_t*, uint8_t* receive, uint16_t size) noexcept override {
    ++transactions;
    if (receive) {
      for (uint16_t i = 0; i < size; ++i) {
        receive[i] = static_cast<uint8_t>(0x40 + i);
      }
    }
    return !failTransfers;
  }

  bool Write(uint8_t*, uint16_t) noexcept override { return !failTransfers; }
  bool Read(uint8_t* receive, uint16_t size) noexcept override {
    if (receive) {
      std::fill(receive, receive + size, 0xa5);
    }
    return !failTransfers;
  }
  bool Reconfigure(const SPIPortConfig&) noexcept override { return true; }
  int32_t GetHandle() const noexcept override { return 7; }

  std::atomic_int transactions{0};
  std::atomic_bool failTransfers{false};
};

struct AutoFixture {
  AutoFixture()
      : engine{
            [this](const uint8_t* send, uint8_t* receive, uint16_t size) {
              ++transfers;
              if (failTransfers) {
                return false;
              }
              for (uint16_t i = 0; i < size; ++i) {
                receive[i] = static_cast<uint8_t>(send[i] + 1);
              }
              return true;
            },
            [this] { return timestamp.load(); },
            [this] {
              rateWaiter = std::make_shared<ManualRateWaiter>();
              return std::unique_ptr<SPIAutoWaiter>{
                  new SharedRateWaiter{rateWaiter}};
            }} {}

  // Adapts the shared test waiter to the ownership expected by the engine.
  class SharedRateWaiter final : public SPIAutoWaiter {
   public:
    explicit SharedRateWaiter(std::shared_ptr<ManualRateWaiter> waiter)
        : m_waiter{std::move(waiter)} {}
    bool WaitUntil(uint64_t deadline, uint64_t& firedAt) noexcept override {
      return m_waiter->WaitUntil(deadline, firedAt);
    }
    void Stop() noexcept override { m_waiter->Stop(); }

   private:
    std::shared_ptr<ManualRateWaiter> m_waiter;
  };

  std::atomic<uint64_t> timestamp{100};
  std::atomic<int> transfers{0};
  bool failTransfers = false;
  std::shared_ptr<ManualRateWaiter> rateWaiter;
  SPIAutoEngine engine;
};

struct AutoManagerFixture {
  AutoManagerFixture()
      : spi{
            [this](HAL_SPIPort, const VMXCommDIOChannelMap&,
                   const SPIPortConfig&) {
              return std::unique_ptr<SPIBackend>{
                  std::make_unique<AutoManagerBackend>()};
            },
            registry,
            [this] { return map; }},
        manager{
            spi,
            [this] { return timestamp.load(); },
            [this] {
              rateWaiter = std::make_shared<ManualRateWaiter>();
              return std::unique_ptr<SPIAutoWaiter>{
                  new AutoFixture::SharedRateWaiter{rateWaiter}};
            },
            [](int32_t, bool, bool) {
              return std::unique_ptr<SPIAutoTriggerWaiter>{};
            }} {}

  DigitalChannelRegistry registry;
  VMXCommDIOChannelMap map;
  SPIManager spi;
  std::atomic<uint64_t> timestamp{1000};
  std::shared_ptr<ManualRateWaiter> rateWaiter;
  SPIAutoManager manager;
};

TEST(VMXAutoSPIInternalTest, InitializesTransmitDataAndReadsTimestampedWords) {
  AutoFixture fixture;
  ASSERT_EQ(fixture.engine.Initialize(16), SPIAutoResult::kOk);
  const uint8_t command[] = {0x10, 0x20};
  ASSERT_EQ(fixture.engine.SetTransmitData(command, 2, 2),
            SPIAutoResult::kOk);
  ASSERT_EQ(fixture.engine.ForceRead(), SPIAutoResult::kOk);

  SPIAutoResult result;
  EXPECT_EQ(fixture.engine.Read(nullptr, 0, 0.0, result), 5);
  EXPECT_EQ(result, SPIAutoResult::kOk);
  uint32_t words[5]{};
  EXPECT_EQ(fixture.engine.Read(words, 2, 0.0, result), 3);
  EXPECT_EQ(words[0], 100u);
  EXPECT_EQ(words[1], 0x11u);
  EXPECT_EQ(fixture.engine.Read(words, 3, 0.0, result), 0);
  EXPECT_EQ(words[0], 0x21u);
  EXPECT_EQ(words[1], 1u);
  EXPECT_EQ(words[2], 1u);
}

TEST(VMXAutoSPIInternalTest, RejectsInvalidParametersAndPropagatesTransferFailure) {
  AutoFixture fixture;
  EXPECT_EQ(fixture.engine.Initialize(0), SPIAutoResult::kInvalidSize);
  ASSERT_EQ(fixture.engine.Initialize(8), SPIAutoResult::kOk);
  EXPECT_EQ(fixture.engine.SetTransmitData(nullptr, 1, 0),
            SPIAutoResult::kNullPointer);
  EXPECT_EQ(fixture.engine.SetTransmitData(nullptr, 0, 128),
            SPIAutoResult::kInvalidSize);
  SPIAutoResult result;
  fixture.failTransfers = true;
  const uint8_t command = 1;
  ASSERT_EQ(fixture.engine.SetTransmitData(&command, 1, 0),
            SPIAutoResult::kOk);
  EXPECT_EQ(fixture.engine.Read(nullptr, 1, -1.0, result), 0);
  EXPECT_EQ(result, SPIAutoResult::kNullPointer);
  EXPECT_EQ(fixture.engine.ForceRead(), SPIAutoResult::kHardwareFailure);
}

TEST(VMXAutoSPIInternalTest, OverflowIncrementsDroppedCountWithoutOverwriting) {
  AutoFixture fixture;
  ASSERT_EQ(fixture.engine.Initialize(3), SPIAutoResult::kOk);
  const uint8_t command = 7;
  ASSERT_EQ(fixture.engine.SetTransmitData(&command, 1, 0),
            SPIAutoResult::kOk);
  ASSERT_EQ(fixture.engine.ForceRead(), SPIAutoResult::kOk);
  ASSERT_EQ(fixture.engine.ForceRead(), SPIAutoResult::kOk);
  SPIAutoResult result;
  EXPECT_EQ(fixture.engine.GetDropped(result), 1);
  uint32_t words[3]{};
  EXPECT_EQ(fixture.engine.Read(words, 3, 0.0, result), 0);
  EXPECT_EQ(words[0], 100u);
  EXPECT_EQ(words[1], 8u);
}

TEST(VMXAutoSPIInternalTest, RateUsesAbsoluteWaiterAndStopsCleanly) {
  AutoFixture fixture;
  ASSERT_EQ(fixture.engine.Initialize(8), SPIAutoResult::kOk);
  const uint8_t command = 3;
  ASSERT_EQ(fixture.engine.SetTransmitData(&command, 1, 0),
            SPIAutoResult::kOk);
  ASSERT_EQ(fixture.engine.StartRate(0.001), SPIAutoResult::kOk);
  ASSERT_TRUE(fixture.rateWaiter->WaitForDeadline());
  fixture.rateWaiter->Fire(1100);
  for (int i = 0; i < 100 && fixture.transfers.load() == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  EXPECT_EQ(fixture.transfers.load(), 1);
  EXPECT_EQ(fixture.engine.Stop(), SPIAutoResult::kOk);
  EXPECT_EQ(fixture.engine.Stop(), SPIAutoResult::kOk);
}

TEST(VMXAutoSPIInternalTest, TriggerWaiterRunsAndStopReleasesWorker) {
  AutoFixture fixture;
  ASSERT_EQ(fixture.engine.Initialize(8), SPIAutoResult::kOk);
  const uint8_t command = 4;
  ASSERT_EQ(fixture.engine.SetTransmitData(&command, 1, 0),
            SPIAutoResult::kOk);
  auto trigger = std::make_unique<ManualTriggerWaiter>();
  auto* triggerPointer = trigger.get();
  ASSERT_EQ(fixture.engine.StartTrigger(std::move(trigger), true, false),
            SPIAutoResult::kOk);
  triggerPointer->Fire();
  for (int i = 0; i < 100 && fixture.transfers.load() == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  EXPECT_EQ(fixture.transfers.load(), 1);
  EXPECT_EQ(fixture.engine.Stop(), SPIAutoResult::kOk);
}

TEST(VMXAutoSPIInternalTest, ManagerSharesSPIAliasesAndOwnsOneReference) {
  AutoManagerFixture fixture;
  ASSERT_EQ(fixture.manager.Initialize(HAL_SPI_kMXP, 16),
            SPIAutoResult::kOk);
  EXPECT_EQ(fixture.manager.Initialize(HAL_SPI_kOnboardCS0, 16),
            SPIAutoResult::kAlreadyAllocated);

  const uint8_t command[] = {0x01, 0x02};
  EXPECT_EQ(fixture.manager.SetTransmitData(HAL_SPI_kOnboardCS0, command, 2, 1),
            SPIAutoResult::kOk);
  EXPECT_EQ(fixture.manager.SetForceRead(HAL_SPI_kOnboardCS0),
            SPIAutoResult::kOk);
  SPIAutoResult result;
  uint32_t words[4]{};
  EXPECT_EQ(fixture.manager.Read(HAL_SPI_kMXP, words, 4, 0.0, result), 0);
  EXPECT_EQ(result, SPIAutoResult::kOk);
  EXPECT_EQ(words[0], 1000u);
  EXPECT_EQ(words[1], 0x40u);
  EXPECT_EQ(words[2], 0x41u);
  EXPECT_EQ(words[3], 0x42u);
  EXPECT_EQ(fixture.manager.ConfigureStall(HAL_SPI_kMXP),
            SPIAutoResult::kUnsupportedStall);

  EXPECT_EQ(fixture.manager.Free(HAL_SPI_kOnboardCS0), SPIAutoResult::kOk);
  EXPECT_EQ(fixture.spi.GetHandle(HAL_SPI_kMXP), 0);
  EXPECT_EQ(fixture.manager.Free(HAL_SPI_kMXP), SPIAutoResult::kOk);
}

TEST(VMXAutoSPIInternalTest, ManagerRejectsPhysicalSPIConflicts) {
  AutoManagerFixture fixture;
  ASSERT_TRUE(fixture.registry
                  .Reserve(fixture.map.spiCLK, DigitalChannelOwner::kDIO,
                           "test DIO")
                  .reserved);
  EXPECT_EQ(fixture.manager.Initialize(HAL_SPI_kMXP, 8),
            SPIAutoResult::kResourceConflict);
}

}  // namespace
}  // namespace hal::vmx
