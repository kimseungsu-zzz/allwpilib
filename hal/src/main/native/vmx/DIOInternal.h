// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>

#include "VMXConstants.h"
#include "DigitalChannelRegistry.h"
#include "hal/Types.h"
#include "hal/handles/DigitalHandleResource.h"

namespace hal::vmx {

enum class DIOResult {
  kOk,
  kOutOfRange,
  kAlreadyAllocated,
  kInvalidHandle,
  kInputChannel,
  kHardwareFailure,
  kRollbackFailure,
};

class DIOBackend {
 public:
  virtual ~DIOBackend() = default;
  virtual bool Set(bool value) noexcept = 0;
  virtual bool Get(bool& value) noexcept = 0;
};

using DIOBackendFactory =
    std::function<std::unique_ptr<DIOBackend>(int32_t channel, bool input)>;

class DIOPort final {
 public:
  bool Initialize(int32_t channel, bool input, std::string_view location,
                  DIOBackendFactory factory) noexcept {
    std::scoped_lock lock{m_mutex};
    m_channel = channel;
    m_input = input;
    m_previousAllocation = location;
    m_factory = std::move(factory);
    m_backend = CreateBackend(input);
    m_faulted = !m_backend;
    return !m_faulted;
  }

  DIOResult Set(bool value) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted || !m_backend) {
      return DIOResult::kHardwareFailure;
    }
    if (m_input) {
      return DIOResult::kInputChannel;
    }
    return m_backend->Set(value) ? DIOResult::kOk : DIOResult::kHardwareFailure;
  }

  std::pair<DIOResult, bool> Get() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted || !m_backend) {
      return {DIOResult::kHardwareFailure, false};
    }
    bool value = false;
    if (!m_backend->Get(value)) {
      return {DIOResult::kHardwareFailure, false};
    }
    return {DIOResult::kOk, value};
  }

  DIOResult SetDirection(bool input) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted || !m_backend) {
      return DIOResult::kHardwareFailure;
    }
    if (input == m_input) {
      return DIOResult::kOk;
    }

    bool previousInput = m_input;
    m_backend.reset();

    auto replacement = CreateBackend(input);
    if (replacement) {
      m_backend = std::move(replacement);
      m_input = input;
      return DIOResult::kOk;
    }

    auto rollback = CreateBackend(previousInput);
    if (rollback) {
      m_backend = std::move(rollback);
      return DIOResult::kHardwareFailure;
    }

    m_faulted = true;
    return DIOResult::kRollbackFailure;
  }

  std::pair<DIOResult, bool> GetDirection() const noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_faulted || !m_backend) {
      return {DIOResult::kHardwareFailure, false};
    }
    return {DIOResult::kOk, m_input};
  }

  void Close() noexcept {
    std::scoped_lock lock{m_mutex};
    m_faulted = true;
    m_backend.reset();
  }

  int32_t GetChannel() const noexcept { return m_channel; }

  std::string GetPreviousAllocation() const {
    std::scoped_lock lock{m_mutex};
    return m_previousAllocation;
  }

 private:
  std::unique_ptr<DIOBackend> CreateBackend(bool input) noexcept {
    try {
      return m_factory ? m_factory(m_channel, input) : nullptr;
    } catch (...) {
      return nullptr;
    }
  }

  mutable std::mutex m_mutex;
  int32_t m_channel = -1;
  bool m_input = true;
  bool m_faulted = true;
  std::string m_previousAllocation;
  DIOBackendFactory m_factory;
  std::unique_ptr<DIOBackend> m_backend;
};

struct DIOAllocationResult {
  HAL_DigitalHandle handle = HAL_kInvalidHandle;
  DIOResult result = DIOResult::kHardwareFailure;
  std::string previousAllocation;
};

class DIOHandleResource final
    : public hal::DigitalHandleResource<HAL_DigitalHandle, DIOPort,
                                        kNumDIOChannels> {
 public:
  DIOHandleResource() { m_version = 0; }
};

class DIOManager final {
 public:
  explicit DIOManager(
      DIOBackendFactory factory,
      DigitalChannelRegistry& registry = GetDigitalChannelRegistry())
      : m_factory{std::move(factory)}, m_registry{registry} {}

  DIOManager(const DIOManager&) = delete;
  DIOManager& operator=(const DIOManager&) = delete;

  DIOAllocationResult Allocate(int32_t channel, bool input,
                               std::string_view location) {
    std::scoped_lock allocationLock{m_allocationMutex};
    if (!IsDIOChannelValid(channel)) {
      return {HAL_kInvalidHandle, DIOResult::kOutOfRange, {}};
    }

    auto reservation =
        m_registry.Reserve(channel, DigitalChannelOwner::kDIO, location);
    if (!reservation.reserved) {
      return {HAL_kInvalidHandle, DIOResult::kAlreadyAllocated,
              std::move(reservation.previousAllocation)};
    }

    HAL_DigitalHandle handle = HAL_kInvalidHandle;
    int32_t status = 0;
    auto port =
        m_handles.Allocate(channel, HAL_HandleEnum::DIO, &handle, &status);
    if (status != 0) {
      m_registry.Release(channel, DigitalChannelOwner::kDIO);
      return {HAL_kInvalidHandle,
              status == RESOURCE_IS_ALLOCATED ? DIOResult::kAlreadyAllocated
                                              : DIOResult::kOutOfRange,
              port ? port->GetPreviousAllocation() : std::string{}};
    }

    if (!port->Initialize(channel, input, location, m_factory)) {
      m_handles.Free(handle, HAL_HandleEnum::DIO);
      m_registry.Release(channel, DigitalChannelOwner::kDIO);
      return {HAL_kInvalidHandle, DIOResult::kHardwareFailure, {}};
    }
    return {handle, DIOResult::kOk, {}};
  }

  void Free(HAL_DigitalHandle handle) noexcept {
    std::scoped_lock allocationLock{m_allocationMutex};
    auto port = m_handles.Get(handle, HAL_HandleEnum::DIO);
    if (!port) {
      return;
    }
    int32_t channel = port->GetChannel();
    port->Close();
    m_handles.Free(handle, HAL_HandleEnum::DIO);
    m_registry.Release(channel, DigitalChannelOwner::kDIO);
  }

  DIOResult Set(HAL_DigitalHandle handle, bool value) noexcept {
    auto port = Get(handle);
    return port ? port->Set(value) : DIOResult::kInvalidHandle;
  }

  std::pair<DIOResult, bool> GetValue(HAL_DigitalHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->Get() : std::pair{DIOResult::kInvalidHandle, false};
  }

  DIOResult SetDirection(HAL_DigitalHandle handle, bool input) noexcept {
    auto port = Get(handle);
    return port ? port->SetDirection(input) : DIOResult::kInvalidHandle;
  }

  std::pair<DIOResult, bool> GetDirection(HAL_DigitalHandle handle) noexcept {
    auto port = Get(handle);
    return port ? port->GetDirection()
                : std::pair{DIOResult::kInvalidHandle, false};
  }

 private:
  std::shared_ptr<DIOPort> Get(HAL_DigitalHandle handle) noexcept {
    return m_handles.Get(handle, HAL_HandleEnum::DIO);
  }

  DIOBackendFactory m_factory;
  DigitalChannelRegistry& m_registry;
  // Serializes channel allocation with driver destruction so a new owner
  // cannot activate a channel while the previous VMX resource is closing.
  std::mutex m_allocationMutex;
  DIOHandleResource m_handles;
};

}  // namespace hal::vmx
