// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "../../../../main/native/vmx/I2CInternal.h"

namespace hal::vmx {
namespace {

struct FakeI2CHardware {
  bool failTransaction = false;
  bool failWrite = false;
  bool failRead = false;
  bool blockTransaction = false;
  bool transactionEntered = false;
  bool releaseTransaction = false;
  int transactions = 0;
  int writes = 0;
  int reads = 0;
  int activeTransactions = 0;
  int maximumActiveTransactions = 0;
  int backendsCreated = 0;
  int backendsDestroyed = 0;
  std::vector<uint8_t> addresses;
  std::mutex mutex;
  std::condition_variable condition;
};

class FakeI2CBackend final : public I2CBackend {
 public:
  explicit FakeI2CBackend(std::shared_ptr<FakeI2CHardware> hardware)
      : m_hardware{std::move(hardware)} {
    ++m_hardware->backendsCreated;
  }

  ~FakeI2CBackend() override { ++m_hardware->backendsDestroyed; }

  bool Transaction(uint8_t address, const uint8_t*, uint16_t,
                   uint8_t* received, uint16_t receiveSize) noexcept override {
    std::unique_lock lock{m_hardware->mutex};
    ++m_hardware->transactions;
    m_hardware->addresses.push_back(address);
    ++m_hardware->activeTransactions;
    m_hardware->maximumActiveTransactions =
        std::max(m_hardware->maximumActiveTransactions,
                 m_hardware->activeTransactions);
    m_hardware->transactionEntered = true;
    m_hardware->condition.notify_all();
    if (m_hardware->blockTransaction) {
      m_hardware->condition.wait(lock, [this] {
        return m_hardware->releaseTransaction;
      });
    }
    if (received && receiveSize > 0) {
      received[0] = address;
    }
    --m_hardware->activeTransactions;
    m_hardware->condition.notify_all();
    return !m_hardware->failTransaction;
  }

  bool Write(uint8_t address, const uint8_t*, int32_t) noexcept override {
    std::scoped_lock lock{m_hardware->mutex};
    ++m_hardware->writes;
    m_hardware->addresses.push_back(address);
    return !m_hardware->failWrite;
  }

  bool Read(uint8_t address, uint8_t* received,
            int32_t receiveSize) noexcept override {
    std::scoped_lock lock{m_hardware->mutex};
    ++m_hardware->reads;
    m_hardware->addresses.push_back(address);
    if (received && receiveSize > 0) {
      received[0] = address;
    }
    return !m_hardware->failRead;
  }

 private:
  std::shared_ptr<FakeI2CHardware> m_hardware;
};

struct I2CFixture {
  I2CFixture()
      : manager{[this] {
          if (failFactory) {
            return std::unique_ptr<I2CBackend>{};
          }
          return std::unique_ptr<I2CBackend>{
              std::make_unique<FakeI2CBackend>(hardware)};
        }} {}

  std::shared_ptr<FakeI2CHardware> hardware =
      std::make_shared<FakeI2CHardware>();
  bool failFactory = false;
  I2CManager manager;
};

TEST(VMXI2CTest, SupportsOnlyTheMappedMXPBus) {
  I2CFixture fixture;
  EXPECT_EQ(fixture.manager.Initialize(HAL_I2C_kMXP), I2CResult::kOk);
  EXPECT_EQ(fixture.manager.Initialize(HAL_I2C_kOnboard),
            I2CResult::kUnsupportedPort);
  EXPECT_EQ(fixture.manager.Initialize(HAL_I2C_kInvalid),
            I2CResult::kPortOutOfRange);
  EXPECT_EQ(fixture.manager.Initialize(static_cast<HAL_I2CPort>(2)),
            I2CResult::kPortOutOfRange);
}

TEST(VMXI2CTest, InitializationIsReferenceCounted) {
  I2CFixture fixture;
  EXPECT_EQ(fixture.manager.Initialize(HAL_I2C_kMXP), I2CResult::kOk);
  EXPECT_EQ(fixture.manager.Initialize(HAL_I2C_kMXP), I2CResult::kOk);
  EXPECT_EQ(fixture.hardware->backendsCreated, 1);
  EXPECT_EQ(fixture.manager.GetReferenceCount(HAL_I2C_kMXP), 2);

  EXPECT_EQ(fixture.manager.Close(HAL_I2C_kMXP), I2CResult::kOk);
  EXPECT_EQ(fixture.hardware->backendsDestroyed, 0);
  EXPECT_EQ(fixture.manager.Close(HAL_I2C_kMXP), I2CResult::kOk);
  EXPECT_EQ(fixture.hardware->backendsDestroyed, 1);
  EXPECT_EQ(fixture.manager.Close(HAL_I2C_kMXP), I2CResult::kOk);
}

TEST(VMXI2CTest, AllocationFailureDoesNotIncrementReferenceCount) {
  I2CFixture fixture;
  fixture.failFactory = true;
  EXPECT_EQ(fixture.manager.Initialize(HAL_I2C_kMXP),
            I2CResult::kHardwareFailure);
  EXPECT_EQ(fixture.manager.GetReferenceCount(HAL_I2C_kMXP), 0);
  fixture.failFactory = false;
  EXPECT_EQ(fixture.manager.Initialize(HAL_I2C_kMXP), I2CResult::kOk);
  EXPECT_EQ(fixture.hardware->backendsCreated, 1);
}

TEST(VMXI2CTest, ValidatesAddressesSizesAndPointers) {
  I2CFixture fixture;
  ASSERT_EQ(fixture.manager.Initialize(HAL_I2C_kMXP), I2CResult::kOk);

  EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x00, nullptr, 0,
                                        nullptr, 0),
            I2CResult::kOk);
  EXPECT_EQ(fixture.manager.Write(HAL_I2C_kMXP, 0x7f, nullptr, 0),
            I2CResult::kOk);
  EXPECT_EQ(fixture.manager.Read(HAL_I2C_kMXP, 0x20, nullptr, 0),
            I2CResult::kOk);

  uint8_t byte = 0;
  EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, -1, nullptr, 0,
                                        nullptr, 0),
            I2CResult::kInvalidAddress);
  EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x80, nullptr, 0,
                                        nullptr, 0),
            I2CResult::kInvalidAddress);
  EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x20, nullptr, 1,
                                        nullptr, 0),
            I2CResult::kNullPointer);
  EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x20, nullptr, 0,
                                        nullptr, 1),
            I2CResult::kNullPointer);
  EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x20, &byte,
                                        UINT16_MAX + 1, nullptr, 0),
            I2CResult::kInvalidSize);
  EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x20, &byte, -1,
                                        nullptr, 0),
            I2CResult::kInvalidSize);
}

TEST(VMXI2CTest, CallsAreSerializedAndPropagateFailures) {
  I2CFixture fixture;
  ASSERT_EQ(fixture.manager.Initialize(HAL_I2C_kMXP), I2CResult::kOk);
  uint8_t send = 0x10;
  uint8_t received = 0;

  EXPECT_EQ(fixture.manager.Write(HAL_I2C_kMXP, 0x20, &send, 1),
            I2CResult::kOk);
  EXPECT_EQ(fixture.manager.Read(HAL_I2C_kMXP, 0x21, &received, 1),
            I2CResult::kOk);
  EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x22, &send, 1,
                                        &received, 1),
            I2CResult::kOk);
  EXPECT_EQ(fixture.hardware->writes, 1);
  EXPECT_EQ(fixture.hardware->reads, 1);
  EXPECT_EQ(fixture.hardware->transactions, 1);
  EXPECT_EQ(fixture.hardware->addresses.size(), 3u);

  fixture.hardware->failTransaction = true;
  EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x22, &send, 1,
                                        &received, 1),
            I2CResult::kHardwareFailure);
}

TEST(VMXI2CTest, ConcurrentTransactionsAndCloseDoNotOverlap) {
  I2CFixture fixture;
  ASSERT_EQ(fixture.manager.Initialize(HAL_I2C_kMXP), I2CResult::kOk);
  fixture.hardware->blockTransaction = true;
  uint8_t byte = 0;

  std::thread first{[&] {
    EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x20, &byte, 1,
                                          nullptr, 0),
              I2CResult::kOk);
  }};
  {
    std::unique_lock lock{fixture.hardware->mutex};
    ASSERT_TRUE(fixture.hardware->condition.wait_for(
        lock, std::chrono::seconds{1},
        [&] { return fixture.hardware->transactionEntered; }));
  }

  std::thread second{[&] {
    EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x21, &byte, 1,
                                          nullptr, 0),
              I2CResult::kOk);
  }};
  std::this_thread::sleep_for(std::chrono::milliseconds{10});
  {
    std::scoped_lock lock{fixture.hardware->mutex};
    fixture.hardware->releaseTransaction = true;
  }
  fixture.hardware->condition.notify_all();
  first.join();
  second.join();
  EXPECT_EQ(fixture.hardware->maximumActiveTransactions, 1);

  fixture.hardware->blockTransaction = true;
  {
    std::scoped_lock lock{fixture.hardware->mutex};
    fixture.hardware->transactionEntered = false;
    fixture.hardware->releaseTransaction = false;
  }
  std::thread inFlight{[&] {
    EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x22, &byte, 1,
                                          nullptr, 0),
              I2CResult::kOk);
  }};
  {
    std::unique_lock lock{fixture.hardware->mutex};
    ASSERT_TRUE(fixture.hardware->condition.wait_for(
        lock, std::chrono::seconds{1},
        [&] { return fixture.hardware->transactionEntered; }));
  }
  std::thread close{[&] {
    EXPECT_EQ(fixture.manager.Close(HAL_I2C_kMXP), I2CResult::kOk);
  }};
  std::this_thread::sleep_for(std::chrono::milliseconds{10});
  {
    std::scoped_lock lock{fixture.hardware->mutex};
    fixture.hardware->releaseTransaction = true;
  }
  fixture.hardware->condition.notify_all();
  inFlight.join();
  close.join();
  EXPECT_EQ(fixture.hardware->backendsDestroyed, 1);
}

TEST(VMXI2CTest, MultipleDeviceAddressesShareOneBus) {
  I2CFixture fixture;
  ASSERT_EQ(fixture.manager.Initialize(HAL_I2C_kMXP), I2CResult::kOk);
  uint8_t value = 0;
  EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x20, nullptr, 0,
                                        &value, 1),
            I2CResult::kOk);
  EXPECT_EQ(fixture.manager.Transaction(HAL_I2C_kMXP, 0x68, nullptr, 0,
                                        &value, 1),
            I2CResult::kOk);
  EXPECT_EQ(fixture.manager.GetReferenceCount(HAL_I2C_kMXP), 1);
}

TEST(VMXI2CTest, CommDioPhysicalReservationConflictsWithI2C) {
  auto hardware = std::make_shared<FakeI2CHardware>();
  DigitalChannelRegistry registry;
  ASSERT_TRUE(registry.Reserve(26, DigitalChannelOwner::kDIO,
                                "logical CommDIO 22")
                  .reserved);
  I2CManager manager{
      [hardware] {
        return std::unique_ptr<I2CBackend>{
            std::make_unique<FakeI2CBackend>(hardware)};
      },
      registry};
  EXPECT_EQ(manager.Initialize(HAL_I2C_kMXP), I2CResult::kResourceConflict);
}

}  // namespace
}  // namespace hal::vmx
