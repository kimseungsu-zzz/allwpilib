// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include "DIOInternal.h"
#include "VMXConstants.h"
#include "hal/DutyCycle.h"
#include "hal/Types.h"
#include "hal/handles/LimitedClassedHandleResource.h"

namespace hal::vmx {

constexpr int32_t kNumVMXDutyCycles = 8;
constexpr int32_t kDutyCycleOutputScaleFactor = 40'000'000 - 1;

enum class DutyCycleResult {
  kOk,
  kInvalidHandle,
  kInvalidSource,
  kUnsupportedSource,
  kAlreadyAllocated,
  kNoResources,
  kHardwareFailure,
  kOutOfRange,
};

class DutyCycleBackend {
 public:
  virtual ~DutyCycleBackend() = default;
  virtual bool GetTiming(uint32_t& periodMicroseconds,
                         uint32_t& highMicroseconds) noexcept = 0;
};

using DutyCycleBackendFactory =
    std::function<std::unique_ptr<DutyCycleBackend>(int32_t physicalChannel)>;

struct DutyCycleSourceClaim {
  DutyCycleResult result = DutyCycleResult::kInvalidSource;
  int32_t physicalChannel = -1;
};

using DutyCycleSourceClaimer =
    std::function<DutyCycleSourceClaim(HAL_Handle source)>;
using DutyCycleSourceReleaser =
    std::function<void(HAL_Handle source, int32_t physicalChannel)>;

class DutyCyclePort final {
 public:
  bool Initialize(HAL_Handle source, int32_t physicalChannel,
                  DutyCycleBackendFactory factory,
                  DutyCycleSourceReleaser releaser) noexcept {
    std::scoped_lock lock{m_mutex};
    m_source = source;
    m_physicalChannel = physicalChannel;
    m_releaser = std::move(releaser);
    try {
      m_backend = factory ? factory(physicalChannel) : nullptr;
    } catch (...) {
      m_backend.reset();
    }
    return m_backend != nullptr;
  }

  DutyCycleResult GetTiming(uint32_t& periodMicroseconds,
                            uint32_t& highMicroseconds) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend ||
        !m_backend->GetTiming(periodMicroseconds, highMicroseconds)) {
      periodMicroseconds = 0;
      highMicroseconds = 0;
      return DutyCycleResult::kHardwareFailure;
    }
    if (highMicroseconds > periodMicroseconds) {
      return DutyCycleResult::kHardwareFailure;
    }
    return DutyCycleResult::kOk;
  }

  void Close() noexcept {
    std::scoped_lock lock{m_mutex};
    m_backend.reset();
    if (m_releaser && m_source != HAL_kInvalidHandle) {
      try {
        m_releaser(m_source, m_physicalChannel);
      } catch (...) {
      }
    }
    m_source = HAL_kInvalidHandle;
    m_physicalChannel = -1;
    m_releaser = {};
  }

 private:
  mutable std::mutex m_mutex;
  HAL_Handle m_source = HAL_kInvalidHandle;
  int32_t m_physicalChannel = -1;
  DutyCycleSourceReleaser m_releaser;
  std::unique_ptr<DutyCycleBackend> m_backend;
};

struct DutyCycleAllocationResult {
  HAL_DutyCycleHandle handle = HAL_kInvalidHandle;
  DutyCycleResult result = DutyCycleResult::kHardwareFailure;
};

class DutyCycleHandleResource final
    : public hal::LimitedClassedHandleResource<HAL_DutyCycleHandle,
                                                DutyCyclePort,
                                                kNumVMXDutyCycles,
                                                HAL_HandleEnum::DutyCycle> {
 public:
  DutyCycleHandleResource() { m_version = 0; }
};

class DutyCycleManager final {
 public:
  DutyCycleManager(DutyCycleBackendFactory factory,
                   DutyCycleSourceClaimer claimer,
                   DutyCycleSourceReleaser releaser)
      : m_factory{std::move(factory)},
        m_claimer{std::move(claimer)},
        m_releaser{std::move(releaser)} {}

  DutyCycleAllocationResult Allocate(HAL_Handle source) {
    std::scoped_lock lock{m_allocationMutex};
    DutyCycleSourceClaim claim;
    try {
      claim = m_claimer ? m_claimer(source)
                        : DutyCycleSourceClaim{
                              DutyCycleResult::kHardwareFailure, -1};
    } catch (...) {
      return {HAL_kInvalidHandle, DutyCycleResult::kHardwareFailure};
    }
    if (claim.result != DutyCycleResult::kOk) {
      return {HAL_kInvalidHandle, claim.result};
    }
    auto newPort = std::make_shared<DutyCyclePort>();
    HAL_DutyCycleHandle handle = m_handles.Allocate(newPort);
    if (handle == HAL_kInvalidHandle) {
      ReleaseClaim(source, claim.physicalChannel);
      return {HAL_kInvalidHandle, DutyCycleResult::kNoResources};
    }
    auto port = m_handles.Get(handle);
    if (!port || !port->Initialize(source, claim.physicalChannel, m_factory,
                                   m_releaser)) {
      m_handles.Free(handle);
      ReleaseClaim(source, claim.physicalChannel);
      return {HAL_kInvalidHandle, DutyCycleResult::kHardwareFailure};
    }
    return {handle, DutyCycleResult::kOk};
  }

  void Free(HAL_DutyCycleHandle handle) noexcept {
    std::scoped_lock lock{m_allocationMutex};
    auto port = m_handles.Get(handle);
    if (!port) {
      return;
    }
    port->Close();
    m_handles.Free(handle);
  }

  std::pair<DutyCycleResult, uint64_t> GetPeriodAndHigh(
      HAL_DutyCycleHandle handle) noexcept {
    auto port = m_handles.Get(handle);
    if (!port) {
      return {DutyCycleResult::kInvalidHandle, 0};
    }
    uint32_t period = 0;
    uint32_t high = 0;
    auto result = port->GetTiming(period, high);
    if (result != DutyCycleResult::kOk) {
      return {result, 0};
    }
    return {DutyCycleResult::kOk,
            (static_cast<uint64_t>(period) << 32) | high};
  }

  std::pair<DutyCycleResult, int32_t> GetFrequency(
      HAL_DutyCycleHandle handle) noexcept {
    auto timing = GetPeriodAndHigh(handle);
    if (timing.first != DutyCycleResult::kOk) {
      return {timing.first, 0};
    }
    uint32_t period = static_cast<uint32_t>(timing.second >> 32);
    if (period == 0) {
      return {DutyCycleResult::kOk, 0};
    }
    auto frequency = static_cast<uint64_t>(std::llround(
        1'000'000.0 / static_cast<double>(period)));
    return {DutyCycleResult::kOk,
            static_cast<int32_t>(std::min<uint64_t>(frequency,
                                                    INT32_MAX))};
  }

  std::pair<DutyCycleResult, double> GetOutput(
      HAL_DutyCycleHandle handle) noexcept {
    auto timing = GetPeriodAndHigh(handle);
    if (timing.first != DutyCycleResult::kOk) {
      return {timing.first, 0.0};
    }
    uint32_t period = static_cast<uint32_t>(timing.second >> 32);
    uint32_t high = static_cast<uint32_t>(timing.second & 0xffffffffu);
    return {DutyCycleResult::kOk,
            period == 0 ? 0.0
                        : static_cast<double>(high) /
                              static_cast<double>(period)};
  }

  std::pair<DutyCycleResult, int32_t> GetHighTime(
      HAL_DutyCycleHandle handle) noexcept {
    auto timing = GetPeriodAndHigh(handle);
    if (timing.first != DutyCycleResult::kOk) {
      return {timing.first, 0};
    }
    uint32_t high = static_cast<uint32_t>(timing.second & 0xffffffffu);
    if (high > static_cast<uint32_t>(INT32_MAX / 1000)) {
      return {DutyCycleResult::kOutOfRange, 0};
    }
    return {DutyCycleResult::kOk, static_cast<int32_t>(high * 1000u)};
  }

  std::pair<DutyCycleResult, int32_t> GetFPGAIndex(
      HAL_DutyCycleHandle handle) noexcept {
    auto port = m_handles.Get(handle);
    if (!port) {
      return {DutyCycleResult::kInvalidHandle, -1};
    }
    return {DutyCycleResult::kOk, m_handles.GetIndex(handle)};
  }

 private:
  void ReleaseClaim(HAL_Handle source, int32_t physicalChannel) noexcept {
    try {
      if (m_releaser) {
        m_releaser(source, physicalChannel);
      }
    } catch (...) {
    }
  }

  DutyCycleBackendFactory m_factory;
  DutyCycleSourceClaimer m_claimer;
  DutyCycleSourceReleaser m_releaser;
  std::mutex m_allocationMutex;
  DutyCycleHandleResource m_handles;
};

DutyCycleManager& GetDutyCycleManager();

}  // namespace hal::vmx
