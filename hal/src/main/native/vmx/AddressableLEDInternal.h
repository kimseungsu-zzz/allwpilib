// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "DigitalChannelRegistry.h"
#include "VMXChannelCapabilities.h"
#include "VMXConstants.h"
#include "hal/AddressableLED.h"
#include "hal/AddressableLEDTypes.h"
#include "hal/Types.h"
#include "hal/handles/IndexedHandleResource.h"

namespace hal::vmx {

constexpr int32_t kNumVMXAddressableLEDs = 1;

struct AddressableLEDConfiguration {
  int32_t length = 1;
  HAL_AddressableLEDColorOrder colorOrder = HAL_ALED_GRB;
  int32_t highTime0Nanoseconds = 300;
  int32_t lowTime0Nanoseconds = 950;
  int32_t highTime1Nanoseconds = 600;
  int32_t lowTime1Nanoseconds = 650;
  int32_t targetFrequencyHz = 800000;
  uint64_t resetWaitMicroseconds = 80;
};

/** Converts a WPILib timing tuple to the one-frequency VMX representation.
 *
 * VMX exposes one target frequency and one high-time per symbol. It does not
 * expose independent low-time fields, so timing requests with different bit
 * periods are rejected instead of silently dropping the low-time values.
 */
inline bool ConvertAddressableLEDBitTiming(int32_t highTime0Nanoseconds,
                                           int32_t lowTime0Nanoseconds,
                                           int32_t highTime1Nanoseconds,
                                           int32_t lowTime1Nanoseconds,
                                           int32_t& targetFrequencyHz) noexcept {
  targetFrequencyHz = 0;
  if (highTime0Nanoseconds < 0 || lowTime0Nanoseconds < 0 ||
      highTime1Nanoseconds < 0 || lowTime1Nanoseconds < 0) {
    return false;
  }
  const int64_t period0 = static_cast<int64_t>(highTime0Nanoseconds) +
                          static_cast<int64_t>(lowTime0Nanoseconds);
  const int64_t period1 = static_cast<int64_t>(highTime1Nanoseconds) +
                          static_cast<int64_t>(lowTime1Nanoseconds);
  if (period0 <= 0 || period0 != period1 || highTime0Nanoseconds >= period0 ||
      highTime1Nanoseconds >= period1) {
    return false;
  }
  const auto frequency =
      std::llround(1.0e9 / static_cast<double>(period0));
  if (frequency <= 0 || frequency > (std::numeric_limits<int32_t>::max)() ||
      std::llround(1.0e9 / static_cast<double>(frequency)) != period0) {
    return false;
  }
  targetFrequencyHz = static_cast<int32_t>(frequency);
  return true;
}

class AddressableLEDBackend {
 public:
  virtual ~AddressableLEDBackend() = default;

  virtual bool Configure(int32_t physicalChannel,
                         const AddressableLEDConfiguration& configuration,
                         const std::vector<HAL_AddressableLEDData>& data,
                         bool running) noexcept = 0;
  virtual bool Write(const std::vector<HAL_AddressableLEDData>& data) noexcept = 0;
  virtual bool Render() noexcept = 0;
  virtual bool Start() noexcept = 0;
  virtual bool Stop() noexcept = 0;
  virtual void Close() noexcept = 0;
};

using AddressableLEDBackendFactory =
    std::function<std::unique_ptr<AddressableLEDBackend>()>;
using AddressableLEDPWMSuspendFunction =
    std::function<bool(HAL_DigitalHandle)>;

enum class AddressableLEDResult {
  kOk,
  kInvalidHandle,
  kInvalidChannel,
  kAlreadyAllocated,
  kUnsupportedCapability,
  kInvalidParameter,
  kHardwareFailure,
};

class AddressableLEDPort final {
 public:
  AddressableLEDResult Initialize(
      int32_t physicalChannel, std::string_view allocationLocation,
      AddressableLEDBackendFactory factory) noexcept {
    std::scoped_lock lock{m_mutex};
    if (physicalChannel < 0 || !factory) {
      return AddressableLEDResult::kInvalidChannel;
    }
    try {
      m_backend = factory();
    } catch (...) {
      m_backend.reset();
    }
    if (!m_backend) {
      return AddressableLEDResult::kHardwareFailure;
    }
    m_physicalChannel = physicalChannel;
    m_previousAllocation = allocationLocation;
    m_data.assign(static_cast<size_t>(m_configuration.length), {});
    if (!m_backend->Configure(m_physicalChannel, m_configuration, m_data,
                              false)) {
      m_backend->Close();
      m_backend.reset();
      return AddressableLEDResult::kHardwareFailure;
    }
    return AddressableLEDResult::kOk;
  }

  AddressableLEDResult SetColorOrder(
      HAL_AddressableLEDColorOrder colorOrder) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return AddressableLEDResult::kInvalidHandle;
    }
    if (colorOrder < HAL_ALED_RGB || colorOrder > HAL_ALED_GRB) {
      return AddressableLEDResult::kInvalidParameter;
    }
    auto configuration = m_configuration;
    configuration.colorOrder = colorOrder;
    return ReconfigureLocked(configuration, m_data);
  }

  AddressableLEDResult SetOutputPort(int32_t physicalChannel) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return AddressableLEDResult::kInvalidHandle;
    }
    if (physicalChannel < 0) {
      return AddressableLEDResult::kInvalidChannel;
    }
    const auto previous = m_physicalChannel;
    m_physicalChannel = physicalChannel;
    auto result = ReconfigureLocked(m_configuration, m_data);
    if (result != AddressableLEDResult::kOk) {
      m_physicalChannel = previous;
    }
    return result;
  }

  AddressableLEDResult SetLength(int32_t length) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return AddressableLEDResult::kInvalidHandle;
    }
    if (length < 0 || length > HAL_kAddressableLEDMaxLength) {
      return AddressableLEDResult::kInvalidParameter;
    }
    auto configuration = m_configuration;
    configuration.length = length;
    auto data = m_data;
    data.resize(static_cast<size_t>(length));
    return ReconfigureLocked(configuration, data);
  }

  AddressableLEDResult Write(const HAL_AddressableLEDData* data,
                             int32_t length) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return AddressableLEDResult::kInvalidHandle;
    }
    if (length < 0 || length > m_configuration.length) {
      return AddressableLEDResult::kInvalidParameter;
    }
    if (length > 0 && data == nullptr) {
      return AddressableLEDResult::kInvalidParameter;
    }
    auto previous = m_data;
    if (length > 0) {
      std::copy_n(data, length, m_data.begin());
    }
    if (!m_backend->Write(m_data)) {
      m_data = std::move(previous);
      return AddressableLEDResult::kHardwareFailure;
    }
    if (m_running && !m_backend->Render()) {
      m_data = std::move(previous);
      return AddressableLEDResult::kHardwareFailure;
    }
    return AddressableLEDResult::kOk;
  }

  AddressableLEDResult SetBitTiming(int32_t highTime0Nanoseconds,
                                    int32_t lowTime0Nanoseconds,
                                    int32_t highTime1Nanoseconds,
                                    int32_t lowTime1Nanoseconds) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return AddressableLEDResult::kInvalidHandle;
    }
    int32_t targetFrequencyHz = 0;
    if (!ConvertAddressableLEDBitTiming(
            highTime0Nanoseconds, lowTime0Nanoseconds, highTime1Nanoseconds,
            lowTime1Nanoseconds, targetFrequencyHz)) {
      return AddressableLEDResult::kInvalidParameter;
    }
    auto configuration = m_configuration;
    configuration.highTime0Nanoseconds = highTime0Nanoseconds;
    configuration.lowTime0Nanoseconds = lowTime0Nanoseconds;
    configuration.highTime1Nanoseconds = highTime1Nanoseconds;
    configuration.lowTime1Nanoseconds = lowTime1Nanoseconds;
    configuration.targetFrequencyHz = targetFrequencyHz;
    return ReconfigureLocked(configuration, m_data);
  }

  AddressableLEDResult SetSyncTime(int32_t syncTimeMicroseconds) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return AddressableLEDResult::kInvalidHandle;
    }
    if (syncTimeMicroseconds < 0) {
      return AddressableLEDResult::kInvalidParameter;
    }
    auto configuration = m_configuration;
    configuration.resetWaitMicroseconds =
        static_cast<uint64_t>(syncTimeMicroseconds);
    return ReconfigureLocked(configuration, m_data);
  }

  AddressableLEDResult Start() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return AddressableLEDResult::kInvalidHandle;
    }
    if (m_running) {
      return AddressableLEDResult::kOk;
    }
    if (!m_backend->Start() || !m_backend->Write(m_data) ||
        !m_backend->Render()) {
      static_cast<void>(m_backend->Stop());
      return AddressableLEDResult::kHardwareFailure;
    }
    m_running = true;
    return AddressableLEDResult::kOk;
  }

  AddressableLEDResult Stop() noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_backend) {
      return AddressableLEDResult::kInvalidHandle;
    }
    if (!m_running) {
      return AddressableLEDResult::kOk;
    }
    if (!m_backend->Stop()) {
      return AddressableLEDResult::kHardwareFailure;
    }
    m_running = false;
    return AddressableLEDResult::kOk;
  }

  void Close() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_backend) {
      static_cast<void>(m_backend->Stop());
      m_backend->Close();
    }
    m_backend.reset();
    m_running = false;
  }

  int32_t GetPhysicalChannel() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_physicalChannel;
  }

  int32_t GetLength() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_configuration.length;
  }

  bool IsRunning() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_running;
  }

  const AddressableLEDConfiguration& GetConfigurationUnsafe() const noexcept {
    return m_configuration;
  }

 private:
  AddressableLEDResult ReconfigureLocked(
      const AddressableLEDConfiguration& configuration,
      const std::vector<HAL_AddressableLEDData>& data) noexcept {
    const auto previousConfiguration = m_configuration;
    const auto previousData = m_data;
    const auto wasRunning = m_running;
    if (!m_backend->Configure(m_physicalChannel, configuration, data,
                              wasRunning)) {
      static_cast<void>(m_backend->Configure(m_physicalChannel,
                                             previousConfiguration,
                                             previousData, wasRunning));
      return AddressableLEDResult::kHardwareFailure;
    }
    m_configuration = configuration;
    m_data = data;
    if (wasRunning) {
      if (!m_backend->Start() || !m_backend->Write(m_data) ||
          !m_backend->Render()) {
        static_cast<void>(m_backend->Stop());
        const bool restored =
            m_backend->Configure(m_physicalChannel, previousConfiguration,
                                 previousData, false) &&
            m_backend->Start() && m_backend->Write(previousData) &&
            m_backend->Render();
        m_configuration = previousConfiguration;
        m_data = previousData;
        m_running = restored;
        return AddressableLEDResult::kHardwareFailure;
      }
    }
    return AddressableLEDResult::kOk;
  }

  mutable std::mutex m_mutex;
  int32_t m_physicalChannel = -1;
  bool m_running = false;
  AddressableLEDConfiguration m_configuration;
  std::vector<HAL_AddressableLEDData> m_data;
  std::string m_previousAllocation;
  std::unique_ptr<AddressableLEDBackend> m_backend;
};

class AddressableLEDHandleResource final
    : public hal::IndexedHandleResource<HAL_AddressableLEDHandle,
                                        AddressableLEDPort,
                                        kNumVMXAddressableLEDs,
                                        HAL_HandleEnum::AddressableLED> {};

class AddressableLEDManager final {
 public:
  explicit AddressableLEDManager(
      AddressableLEDBackendFactory factory,
      DigitalChannelRegistry& registry = GetDigitalChannelRegistry(),
      const VMXCapabilityProvider* capabilities = nullptr,
      AddressableLEDPWMSuspendFunction suspendPWM = {})
      : m_factory{std::move(factory)},
        m_registry{registry},
        m_capabilities{capabilities},
        m_suspendPWM{std::move(suspendPWM)} {}

  AddressableLEDResult Initialize(HAL_DigitalHandle outputPort,
                                  std::string_view allocationLocation,
                                  HAL_AddressableLEDHandle& handle) noexcept;
  AddressableLEDResult SetOutputPort(HAL_AddressableLEDHandle handle,
                                     HAL_DigitalHandle outputPort) noexcept;
  AddressableLEDResult SetColorOrder(HAL_AddressableLEDHandle handle,
                                     HAL_AddressableLEDColorOrder order) noexcept;
  AddressableLEDResult SetLength(HAL_AddressableLEDHandle handle,
                                 int32_t length) noexcept;
  AddressableLEDResult Write(HAL_AddressableLEDHandle handle,
                             const HAL_AddressableLEDData* data,
                             int32_t length) noexcept;
  AddressableLEDResult SetBitTiming(HAL_AddressableLEDHandle handle,
                                    int32_t highTime0Nanoseconds,
                                    int32_t lowTime0Nanoseconds,
                                    int32_t highTime1Nanoseconds,
                                    int32_t lowTime1Nanoseconds) noexcept;
  AddressableLEDResult SetSyncTime(HAL_AddressableLEDHandle handle,
                                   int32_t syncTimeMicroseconds) noexcept;
  AddressableLEDResult Start(HAL_AddressableLEDHandle handle) noexcept;
  AddressableLEDResult Stop(HAL_AddressableLEDHandle handle) noexcept;
  void Free(HAL_AddressableLEDHandle handle) noexcept;
  void Shutdown() noexcept;

 private:
  std::shared_ptr<AddressableLEDPort> Get(
      HAL_AddressableLEDHandle handle) noexcept {
    return m_handles.Get(handle);
  }

  bool ValidateOutputPort(HAL_DigitalHandle outputPort,
                          int32_t& logicalChannel,
                          int32_t& physicalChannel) const noexcept;

  AddressableLEDBackendFactory m_factory;
  DigitalChannelRegistry& m_registry;
  const VMXCapabilityProvider* m_capabilities;
  AddressableLEDPWMSuspendFunction m_suspendPWM;
  AddressableLEDHandleResource m_handles;
  std::mutex m_allocationMutex;
};

AddressableLEDManager& GetAddressableLEDManager();

}  // namespace hal::vmx
