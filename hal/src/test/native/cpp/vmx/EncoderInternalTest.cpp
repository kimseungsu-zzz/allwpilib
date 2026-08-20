// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "../../../../main/native/vmx/EncoderInternal.h"

namespace hal::vmx {
namespace {

struct FakeEncoderHardware {
  int32_t count = 0;
  bool forward = true;
  uint16_t periodMicroseconds = 0;
  bool failReads = false;
  bool failReset = false;
  int resets = 0;
  VMXEncoderEdge edge = VMXEncoderEdge::k4X;
};

class FakeEncoderBackend final : public EncoderBackend {
 public:
  explicit FakeEncoderBackend(std::shared_ptr<FakeEncoderHardware> hardware)
      : m_hardware{std::move(hardware)} {}

  bool GetCount(int32_t& count) noexcept override {
    count = m_hardware->count;
    return !m_hardware->failReads;
  }

  bool GetDirection(bool& forward) noexcept override {
    forward = m_hardware->forward;
    return !m_hardware->failReads;
  }

  bool Reset() noexcept override {
    if (m_hardware->failReset) {
      return false;
    }
    m_hardware->count = 0;
    ++m_hardware->resets;
    return true;
  }

  bool GetPeriodMicroseconds(uint16_t& period) noexcept override {
    period = m_hardware->periodMicroseconds;
    return !m_hardware->failReads;
  }

 private:
  std::shared_ptr<FakeEncoderHardware> m_hardware;
};

struct EncoderFixture {
  EncoderFixture()
      : manager{
            [this](int32_t channelA, int32_t channelB, VMXEncoderEdge edge) {
              lastChannelA = channelA;
              lastChannelB = channelB;
              hardware->edge = edge;
              if (failFactory) {
                return std::unique_ptr<EncoderBackend>{};
              }
              return std::unique_ptr<EncoderBackend>{
                  std::make_unique<FakeEncoderBackend>(hardware)};
            },
            [this](HAL_Handle sourceA, HAL_Handle sourceB) {
              ++claims;
              return claimResult == EncoderResult::kOk
                         ? EncoderSourceClaim{EncoderResult::kOk,
                                              sourceA + 10, sourceB + 10}
                         : EncoderSourceClaim{claimResult, -1, -1};
            },
            [this](HAL_Handle, HAL_Handle, int32_t, int32_t) { ++releases; },
            [this] { return now; }} {}

  EncoderAllocationResult Allocate(
      HAL_EncoderEncodingType type = HAL_Encoder_k4X,
      bool reverseDirection = false) {
    return manager.Allocate(1, 2, reverseDirection, type);
  }

  std::shared_ptr<FakeEncoderHardware> hardware =
      std::make_shared<FakeEncoderHardware>();
  std::chrono::steady_clock::time_point now{};
  EncoderResult claimResult = EncoderResult::kOk;
  bool failFactory = false;
  int claims = 0;
  int releases = 0;
  int32_t lastChannelA = -1;
  int32_t lastChannelB = -1;
  EncoderManager manager;
};

TEST(VMXEncoderTest, SelectsHardwareEdgeAndScalesCounts) {
  for (auto type : {HAL_Encoder_k1X, HAL_Encoder_k2X, HAL_Encoder_k4X}) {
    EncoderFixture fixture;
    auto allocation = fixture.Allocate(type);
    ASSERT_EQ(allocation.result, EncoderResult::kOk);
    fixture.hardware->count = 8;

    EXPECT_EQ(fixture.manager.GetRaw(allocation.handle).second, 8);
    EXPECT_EQ(fixture.manager.Get(allocation.handle).second,
              8 / EncodingScale(type));
    EXPECT_EQ(fixture.manager.GetEncodingScale(allocation.handle).second,
              EncodingScale(type));
    EXPECT_DOUBLE_EQ(
        fixture.manager.GetDecodingScale(allocation.handle).second,
        DecodingScale(type));
    EXPECT_EQ(fixture.lastChannelA, 11);
    EXPECT_EQ(fixture.lastChannelB, 12);
  }
}

TEST(VMXEncoderTest, AllocationIsTransactional) {
  EncoderFixture fixture;
  fixture.claimResult = EncoderResult::kAlreadyAllocated;
  EXPECT_EQ(fixture.Allocate().result, EncoderResult::kAlreadyAllocated);
  EXPECT_EQ(fixture.releases, 0);

  fixture.claimResult = EncoderResult::kOk;
  fixture.failFactory = true;
  EXPECT_EQ(fixture.Allocate().result, EncoderResult::kHardwareFailure);
  EXPECT_EQ(fixture.releases, 1);

  fixture.failFactory = false;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, EncoderResult::kOk);
  fixture.manager.Free(allocation.handle);
  EXPECT_EQ(fixture.releases, 2);
}

TEST(VMXEncoderTest, RejectsInvalidEncodingBeforeClaimingSources) {
  EncoderFixture fixture;
  auto allocation = fixture.Allocate(
      static_cast<HAL_EncoderEncodingType>(99));
  EXPECT_EQ(allocation.result, EncoderResult::kInvalidEncoding);
  EXPECT_EQ(fixture.claims, 0);
}

TEST(VMXEncoderTest, ReverseDirectionIsConsistent) {
  EncoderFixture fixture;
  auto allocation = fixture.Allocate(HAL_Encoder_k4X, true);
  ASSERT_EQ(allocation.result, EncoderResult::kOk);
  fixture.hardware->count = 20;
  fixture.hardware->forward = true;
  fixture.hardware->periodMicroseconds = 2500;
  ASSERT_EQ(fixture.manager.SetDistancePerPulse(allocation.handle, 0.5),
            EncoderResult::kOk);

  EXPECT_EQ(fixture.manager.GetRaw(allocation.handle).second, -20);
  EXPECT_EQ(fixture.manager.Get(allocation.handle).second, -5);
  EXPECT_FALSE(fixture.manager.GetDirection(allocation.handle).second);
  EXPECT_DOUBLE_EQ(fixture.manager.GetDistance(allocation.handle).second,
                   -2.5);
  EXPECT_DOUBLE_EQ(fixture.manager.GetRate(allocation.handle).second, -200.0);
}

TEST(VMXEncoderTest, ConvertsHardwarePeriodFromMicrosecondsToSeconds) {
  EncoderFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, EncoderResult::kOk);
  fixture.hardware->periodMicroseconds = 1234;
  EXPECT_DOUBLE_EQ(fixture.manager.GetPeriod(allocation.handle).second,
                   0.001234);
}

TEST(VMXEncoderTest, StoppedUsesMonotonicCountChanges) {
  EncoderFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, EncoderResult::kOk);
  ASSERT_EQ(fixture.manager.SetMaxPeriod(allocation.handle, 0.1),
            EncoderResult::kOk);

  EXPECT_FALSE(fixture.manager.GetStopped(allocation.handle).second);
  fixture.now += std::chrono::milliseconds{101};
  EXPECT_TRUE(fixture.manager.GetStopped(allocation.handle).second);
  fixture.hardware->count = 1;
  EXPECT_FALSE(fixture.manager.GetStopped(allocation.handle).second);
}

TEST(VMXEncoderTest, ResetAndHardwareFailuresAreReported) {
  EncoderFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, EncoderResult::kOk);
  fixture.hardware->count = 42;
  EXPECT_EQ(fixture.manager.Reset(allocation.handle), EncoderResult::kOk);
  EXPECT_EQ(fixture.hardware->resets, 1);
  EXPECT_EQ(fixture.hardware->count, 0);

  fixture.hardware->failReads = true;
  EXPECT_EQ(fixture.manager.GetRaw(allocation.handle).first,
            EncoderResult::kHardwareFailure);
  fixture.hardware->failReset = true;
  EXPECT_EQ(fixture.manager.Reset(allocation.handle),
            EncoderResult::kHardwareFailure);
}

TEST(VMXEncoderTest, SamplesToAverageIsExplicitlyUnsupported) {
  EncoderFixture fixture;
  auto allocation = fixture.Allocate();
  ASSERT_EQ(allocation.result, EncoderResult::kOk);
  EXPECT_EQ(fixture.manager.SetSamplesToAverage(allocation.handle, 0),
            EncoderResult::kOutOfRange);
  EXPECT_EQ(fixture.manager.SetSamplesToAverage(allocation.handle, 4),
            EncoderResult::kUnsupported);
  EXPECT_EQ(fixture.manager.GetSamplesToAverage(allocation.handle).first,
            EncoderResult::kUnsupported);
}

TEST(VMXEncoderTest, InvalidHandlesAreRejected) {
  EncoderFixture fixture;
  EXPECT_EQ(fixture.manager.GetRaw(HAL_kInvalidHandle).first,
            EncoderResult::kInvalidHandle);
  EXPECT_EQ(fixture.manager.Reset(HAL_kInvalidHandle),
            EncoderResult::kInvalidHandle);
}

}  // namespace
}  // namespace hal::vmx
