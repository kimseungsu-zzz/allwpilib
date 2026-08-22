// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/DIO.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string_view>

#include "VMXPi.h"

#include "DIOInternal.h"
#include "HALInitializer.h"
#include "HALInternal.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"
#include "hal/handles/LimitedClassedHandleResource.h"

namespace hal::vmx {
namespace {

class DriverDIOBackend final : public DIOBackend {
 public:
  DriverDIOBackend(int32_t channel, bool input,
                   std::shared_ptr<VMXPi> context)
      : m_context{std::move(context)}, m_input{input} {
    if (!m_context || !m_context->IsOpen()) {
      return;
    }
    DIOConfig config;
    VMXChannelCapability capability;
    if (input) {
      config.SetInputMode(DIOConfig::InputMode::PULLUP);
      capability = VMXChannelCapability::DigitalInput;
    } else {
      config = DIOConfig{DIOConfig::PUSHPULL};
      capability = VMXChannelCapability::DigitalOutput;
    }
    VMXErrorCode error;
    m_initialized = m_context->io.ActivateSinglechannelResource(
        ::VMXChannelInfo(channel, capability), &config, m_resourceHandle,
        &error);
  }

  ~DriverDIOBackend() override {
    if (m_initialized) {
      VMXErrorCode error;
      m_context->io.DeallocateResource(m_resourceHandle, &error);
    }
  }

  bool IsInitialized() const noexcept { return m_initialized; }

  bool Set(bool value) noexcept override {
    try {
      if (!m_initialized || m_input) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.DIO_Set(m_resourceHandle, value, &error);
    } catch (...) {
      return false;
    }
  }

  bool Get(bool& value) noexcept override {
    try {
      if (!m_initialized) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.DIO_Get(m_resourceHandle, value, &error);
    } catch (...) {
      value = false;
      return false;
    }
  }

  bool Pulse(uint32_t microseconds) noexcept override {
    try {
      if (!m_initialized || m_input) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.DIO_Pulse(m_resourceHandle, true, microseconds,
                                     &error);
    } catch (...) {
      return false;
    }
  }

  bool IsPulsing(bool& isPulsing) noexcept override {
    try {
      if (!m_initialized || m_input) {
        return false;
      }
      VMXErrorCode error;
      return m_context->io.DIO_IsPulsing(m_resourceHandle, isPulsing, &error);
    } catch (...) {
      isPulsing = false;
      return false;
    }
  }

 private:
  std::shared_ptr<VMXPi> m_context;
  VMXResourceHandle m_resourceHandle = 0;
  bool m_input = true;
  bool m_initialized = false;
};

std::unique_ptr<DIOBackend> CreateDIOBackend(int32_t channel, bool input) {
  auto context = GetRuntimeContext();
  if (!context) {
    return nullptr;
  }

  auto backend =
      std::make_unique<DriverDIOBackend>(channel, input, std::move(context));
  if (!backend->IsInitialized()) {
    return nullptr;
  }
  return backend;
}

void SetHardwareError(int32_t* status, std::string_view message) {
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(status, message);
}

void SetUnsupported(int32_t* status, std::string_view feature) {
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(status,
                    std::string{"VMX HAL does not yet support "} +
                        std::string{feature});
}

constexpr int32_t kDigitalPWMGeneratorCount = kNumDIOChannels;
constexpr int32_t kDigitalPWMMaxDutyCycle = 10000;

class DigitalPWMGenerator final {
 public:
  DigitalPWMGenerator(std::shared_ptr<VMXPi> context,
                      DigitalChannelRegistry& registry, int32_t rate)
      : m_context{std::move(context)}, m_registry{registry}, m_rate{rate} {}

  ~DigitalPWMGenerator() { Close(); }

  bool SetRate(double rate, int32_t* status) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!std::isfinite(rate) || rate < 0.6 || rate > 19000.0) {
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status, "VMX DigitalPWM rate must be in [0.6, 19000] Hz");
      return false;
    }
    m_rate = static_cast<int32_t>(std::lround(rate));
    return ReconfigureLocked(status);
  }

  bool SetDutyCycle(double dutyCycle, int32_t* status) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!std::isfinite(dutyCycle)) {
      *status = PARAMETER_OUT_OF_RANGE;
      return false;
    }
    if (m_channel < 0 || !m_active) {
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, "VMX DigitalPWM output channel is not configured");
      return false;
    }
    m_dutyCycle = std::clamp(dutyCycle, 0.0, 1.0);
    return WriteDutyLocked(status);
  }

  bool SetOutputChannel(int32_t channel, int32_t* status) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!IsDIOChannelValid(channel)) {
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastErrorIndexOutOfRange(status, "Invalid VMX DigitalPWM channel",
                                       0, kNumDIOChannels - 1, channel);
      return false;
    }
    const auto physical = ToVMXDigitalChannel(channel);
    if (!GetVMXCapabilityProvider().SupportsPhysical(
            physical, VMXCapability::kPWMGenerator)) {
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status,
                        "VMX channel does not support PWMGenerator output");
      return false;
    }
    if (m_channel == channel && m_active) {
      *status = HAL_SUCCESS;
      return true;
    }
    return ActivateLocked(channel, physical, status);
  }

  bool Close() noexcept {
    std::scoped_lock lock{m_mutex};
    bool success = true;
    if (m_active && m_context && m_context->IsOpen()) {
      VMXErrorCode error;
      if (!m_context->io.DeallocateResource(m_resourceHandle, &error)) {
        success = false;
      }
    }
    if (m_physicalChannel >= 0) {
      m_registry.Release(m_physicalChannel, DigitalChannelOwner::kPWM);
    }
    m_resourceHandle = 0;
    m_active = false;
    m_channel = -1;
    m_physicalChannel = -1;
    return success;
  }

 private:
  bool WriteDutyLocked(int32_t* status) noexcept {
    if (!m_context || !m_active) {
      *status = INCOMPATIBLE_STATE;
      return false;
    }
    try {
      VMXErrorCode error;
      const auto duty = static_cast<uint16_t>(std::lround(
          m_dutyCycle * static_cast<double>(kDigitalPWMMaxDutyCycle)));
      if (!m_context->io.PWMGenerator_SetDutyCycle(m_resourceHandle, 0, duty,
                                                    &error)) {
        *status = INCOMPATIBLE_STATE;
        hal::SetLastError(status, "VMX DigitalPWM duty-cycle write failed");
        return false;
      }
      *status = HAL_SUCCESS;
      return true;
    } catch (...) {
      *status = INCOMPATIBLE_STATE;
      return false;
    }
  }

  bool ActivateLocked(int32_t channel, int32_t physical,
                      int32_t* status) noexcept {
    const auto oldPhysical = m_physicalChannel;
    auto reservation = m_registry.Reserve(physical, DigitalChannelOwner::kPWM,
                                           "VMX DigitalPWM");
    if (!reservation.reserved) {
      *status = RESOURCE_IS_ALLOCATED;
      hal::SetLastErrorPreviouslyAllocated(
          status, "PWM or DIO", physical, reservation.previousAllocation);
      return false;
    }

    if (m_active && m_context && m_context->IsOpen()) {
      VMXErrorCode error;
      if (!m_context->io.DeallocateResource(m_resourceHandle, &error)) {
        m_registry.Release(physical, DigitalChannelOwner::kPWM);
        *status = INCOMPATIBLE_STATE;
        return false;
      }
      m_active = false;
      m_resourceHandle = 0;
    }
    if (oldPhysical >= 0 && oldPhysical != physical) {
      m_registry.Release(oldPhysical, DigitalChannelOwner::kPWM);
      m_channel = -1;
      m_physicalChannel = -1;
    }
    try {
      if (!m_context || !m_context->IsOpen()) {
        m_registry.Release(physical, DigitalChannelOwner::kPWM);
        *status = INCOMPATIBLE_STATE;
        return false;
      }
      PWMGeneratorConfig config{m_rate};
      config.SetMaxDutyCycleValue(kDigitalPWMMaxDutyCycle);
      VMXErrorCode error;
      if (!m_context->io.ActivateSinglechannelResource(
              ::VMXChannelInfo(physical,
                               VMXChannelCapability::PWMGeneratorOutput),
              &config, m_resourceHandle, &error)) {
        m_registry.Release(physical, DigitalChannelOwner::kPWM);
        *status = INCOMPATIBLE_STATE;
        hal::SetLastError(status, "VMX DigitalPWM resource activation failed");
        return false;
      }
      m_channel = channel;
      m_physicalChannel = physical;
      m_active = true;
      if (!WriteDutyLocked(status)) {
        m_context->io.DeallocateResource(m_resourceHandle, &error);
        m_registry.Release(physical, DigitalChannelOwner::kPWM);
        m_resourceHandle = 0;
        m_active = false;
        m_channel = -1;
        m_physicalChannel = -1;
        return false;
      }
      *status = HAL_SUCCESS;
      return true;
    } catch (...) {
      m_registry.Release(physical, DigitalChannelOwner::kPWM);
      *status = INCOMPATIBLE_STATE;
      return false;
    }
  }

  bool ReconfigureLocked(int32_t* status) noexcept {
    if (m_channel < 0) {
      *status = HAL_SUCCESS;
      return true;
    }
    const auto channel = m_channel;
    const auto physical = m_physicalChannel;
    if (m_active && m_context && m_context->IsOpen()) {
      VMXErrorCode error;
      if (!m_context->io.DeallocateResource(m_resourceHandle, &error)) {
        *status = INCOMPATIBLE_STATE;
        hal::SetLastError(status,
                          "VMX DigitalPWM reconfiguration could not release "
                          "the active generator");
        return false;
      }
      m_resourceHandle = 0;
    }
    m_registry.Release(physical, DigitalChannelOwner::kPWM);
    m_channel = -1;
    m_physicalChannel = -1;
    m_active = false;
    return ActivateLocked(channel, physical, status);
  }

  std::mutex m_mutex;
  std::shared_ptr<VMXPi> m_context;
  DigitalChannelRegistry& m_registry;
  VMXResourceHandle m_resourceHandle = 0;
  int32_t m_channel = -1;
  int32_t m_physicalChannel = -1;
  int32_t m_rate = 1000;
  double m_dutyCycle = 0.0;
  bool m_active = false;
};

using DigitalPWMHandleResource =
    hal::LimitedClassedHandleResource<HAL_DigitalPWMHandle,
                                      DigitalPWMGenerator,
                                      kDigitalPWMGeneratorCount,
                                      HAL_HandleEnum::DigitalPWM>;

class DigitalPWMManager final {
 public:
  HAL_DigitalPWMHandle Allocate(int32_t* status) {
    std::scoped_lock lock{m_mutex};
    auto context = GetRuntimeContext();
    if (!context) {
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, "VMX runtime is not initialized");
      return HAL_kInvalidHandle;
    }
    auto generator = std::make_shared<DigitalPWMGenerator>(
        std::move(context), GetDigitalChannelRegistry(),
        static_cast<int32_t>(std::lround(m_rate)));
    auto handle = m_handles.Allocate(generator);
    if (handle == HAL_kInvalidHandle) {
      *status = NO_AVAILABLE_RESOURCES;
      return HAL_kInvalidHandle;
    }
    Register(handle, generator);
    *status = HAL_SUCCESS;
    return handle;
  }

  std::shared_ptr<DigitalPWMGenerator> Get(HAL_DigitalPWMHandle handle) {
    return m_handles.Get(handle);
  }

  void Free(HAL_DigitalPWMHandle handle) {
    auto generator = m_handles.Get(handle);
    if (generator) {
      generator->Close();
    }
    m_handles.Free(handle);
  }

  bool SetRate(double rate, int32_t* status) {
    std::scoped_lock lock{m_mutex};
    if (!std::isfinite(rate) || rate < 0.6 || rate > 19000.0) {
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status, "VMX DigitalPWM rate must be in [0.6, 19000] Hz");
      return false;
    }
    m_rate = rate;
    bool success = true;
    for (auto& weak : m_generators) {
      if (auto generator = weak.lock()) {
        success = generator->SetRate(rate, status) && success;
      }
    }
    *status = success ? HAL_SUCCESS : INCOMPATIBLE_STATE;
    return success;
  }

  bool Route(HAL_DigitalPWMHandle handle, int32_t channel, int32_t* status) {
    auto generator = m_handles.Get(handle);
    if (!generator) {
      *status = HAL_HANDLE_ERROR;
      return false;
    }
    const auto result = generator->SetOutputChannel(channel, status);
    return result;
  }

  void Register(HAL_DigitalPWMHandle handle,
                const std::shared_ptr<DigitalPWMGenerator>& generator) {
    const auto index = m_handles.GetIndex(handle);
    if (index >= 0 && index < kDigitalPWMGeneratorCount) {
      m_generators[index] = generator;
    }
  }

 private:
  std::mutex m_mutex;
  double m_rate = 1000.0;
  DigitalPWMHandleResource m_handles;
  std::array<std::weak_ptr<DigitalPWMGenerator>,
             kDigitalPWMGeneratorCount>
      m_generators;
};

DigitalPWMManager& GetDigitalPWMManager() {
  static DigitalPWMManager manager;
  return manager;
}

bool CheckHandleForUnsupported(HAL_DigitalHandle handle, int32_t* status) {
  auto [result, input] = GetDIOManager().GetDirection(handle);
  static_cast<void>(input);
  if (result == DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }
  if (result != DIOResult::kOk) {
    SetHardwareError(status, "VMX DIO resource is faulted");
    return false;
  }
  return true;
}

}  // namespace

DIOManager& GetDIOManager() {
  static DIOManager manager{CreateDIOBackend, GetDigitalChannelRegistry(),
                            &GetVMXCapabilityProvider()};
  return manager;
}
}  // namespace hal::vmx

extern "C" {

HAL_DigitalHandle HAL_InitializeDIOPort(HAL_PortHandle portHandle,
                                        HAL_Bool input,
                                        const char* allocationLocation,
                                        int32_t* status) {
  hal::init::CheckInit();
  if (!hal::vmx::IsRuntimeInitialized()) {
    hal::vmx::SetHardwareError(status, "VMX HAL runtime is not initialized");
    return HAL_kInvalidHandle;
  }

  int16_t channel = hal::getPortHandleChannel(portHandle);
  if (!HAL_CheckDIOChannel(channel)) {
    *status = RESOURCE_OUT_OF_RANGE;
    hal::SetLastErrorIndexOutOfRange(status, "Invalid Index for VMX DIO", 0,
                                     hal::vmx::kNumDIOChannels - 1, channel);
    return HAL_kInvalidHandle;
  }

  auto result = hal::vmx::GetDIOManager().Allocate(
      channel, input != 0, allocationLocation ? allocationLocation : "");
  if (result.result == hal::vmx::DIOResult::kAlreadyAllocated) {
    *status = RESOURCE_IS_ALLOCATED;
    hal::SetLastErrorPreviouslyAllocated(status, "DIO", channel,
                                         result.previousAllocation);
    return HAL_kInvalidHandle;
  }
  if (result.result != hal::vmx::DIOResult::kOk) {
    if (result.result == hal::vmx::DIOResult::kUnsupportedCapability) {
      hal::vmx::SetHardwareError(
          status, "VMX DIO channel does not support the requested direction");
      return HAL_kInvalidHandle;
    }
    hal::vmx::SetHardwareError(
        status, "Failed to activate the VMX DIO hardware resource");
    return HAL_kInvalidHandle;
  }

  *status = HAL_SUCCESS;
  return result.handle;
}

HAL_Bool HAL_CheckDIOChannel(int32_t channel) {
  return hal::vmx::IsDIOChannelValid(channel);
}

void HAL_FreeDIOPort(HAL_DigitalHandle dioPortHandle) {
  hal::vmx::GetDIOManager().Free(dioPortHandle);
}

void HAL_SetDIOSimDevice(HAL_DigitalHandle handle,
                         HAL_SimDeviceHandle device) {
  static_cast<void>(handle);
  static_cast<void>(device);
}

void HAL_SetDIO(HAL_DigitalHandle dioPortHandle, HAL_Bool value,
                int32_t* status) {
  auto result = hal::vmx::GetDIOManager().Set(dioPortHandle, value != 0);
  switch (result) {
    case hal::vmx::DIOResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case hal::vmx::DIOResult::kInvalidHandle:
      *status = HAL_HANDLE_ERROR;
      return;
    case hal::vmx::DIOResult::kInputChannel:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status, "Cannot set output of an input channel");
      return;
    default:
      hal::vmx::SetHardwareError(status, "Failed to write VMX DIO resource");
      return;
  }
}

HAL_Bool HAL_GetDIO(HAL_DigitalHandle dioPortHandle, int32_t* status) {
  auto [result, value] = hal::vmx::GetDIOManager().GetValue(dioPortHandle);
  if (result == hal::vmx::DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }
  if (result != hal::vmx::DIOResult::kOk) {
    hal::vmx::SetHardwareError(status, "Failed to read VMX DIO resource");
    return false;
  }
  *status = HAL_SUCCESS;
  return value;
}

void HAL_SetDIODirection(HAL_DigitalHandle dioPortHandle, HAL_Bool input,
                         int32_t* status) {
  auto result =
      hal::vmx::GetDIOManager().SetDirection(dioPortHandle, input != 0);
  if (result == hal::vmx::DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  if (result == hal::vmx::DIOResult::kRollbackFailure) {
    hal::vmx::SetHardwareError(
        status, "VMX DIO direction change and rollback both failed; resource "
                "is faulted");
    return;
  }
  if (result != hal::vmx::DIOResult::kOk) {
    if (result == hal::vmx::DIOResult::kUnsupportedCapability) {
      hal::vmx::SetHardwareError(
          status, "VMX DIO channel does not support the requested direction");
      return;
    }
    hal::vmx::SetHardwareError(
        status, "VMX DIO direction change failed; previous direction restored");
    return;
  }
  *status = HAL_SUCCESS;
}

HAL_Bool HAL_GetDIODirection(HAL_DigitalHandle dioPortHandle,
                             int32_t* status) {
  auto [result, input] =
      hal::vmx::GetDIOManager().GetDirection(dioPortHandle);
  if (result == hal::vmx::DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }
  if (result != hal::vmx::DIOResult::kOk) {
    hal::vmx::SetHardwareError(status, "VMX DIO resource is faulted");
    return false;
  }
  *status = HAL_SUCCESS;
  return input;
}

HAL_DigitalPWMHandle HAL_AllocateDigitalPWM(int32_t* status) {
  hal::init::CheckInit();
  return hal::vmx::GetDigitalPWMManager().Allocate(status);
}

void HAL_FreeDigitalPWM(HAL_DigitalPWMHandle pwmGenerator) {
  hal::vmx::GetDigitalPWMManager().Free(pwmGenerator);
}

void HAL_SetDigitalPWMRate(double rate, int32_t* status) {
  hal::vmx::GetDigitalPWMManager().SetRate(rate, status);
}

void HAL_SetDigitalPWMDutyCycle(HAL_DigitalPWMHandle pwmGenerator,
                                double dutyCycle, int32_t* status) {
  auto generator = hal::vmx::GetDigitalPWMManager().Get(pwmGenerator);
  if (!generator) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  generator->SetDutyCycle(dutyCycle, status);
}

void HAL_SetDigitalPWMPPS(HAL_DigitalPWMHandle pwmGenerator, double dutyCycle,
                          int32_t* status) {
  static_cast<void>(dutyCycle);
  if (!hal::vmx::GetDigitalPWMManager().Get(pwmGenerator)) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  hal::vmx::SetUnsupported(status, "Digital PWM PPS mode");
}

void HAL_SetDigitalPWMOutputChannel(HAL_DigitalPWMHandle pwmGenerator,
                                    int32_t channel, int32_t* status) {
  hal::vmx::GetDigitalPWMManager().Route(pwmGenerator, channel, status);
}

void HAL_Pulse(HAL_DigitalHandle dioPortHandle, double pulseLengthSeconds,
               int32_t* status) {
  if (!std::isfinite(pulseLengthSeconds) || pulseLengthSeconds <= 0.0 ||
      pulseLengthSeconds >
          static_cast<double>(std::numeric_limits<uint32_t>::max()) / 1.0e6) {
    *status = PARAMETER_OUT_OF_RANGE;
    hal::SetLastError(status, "VMX DIO pulse length is out of range");
    return;
  }
  auto microseconds =
      static_cast<uint32_t>(std::llround(pulseLengthSeconds * 1.0e6));
  if (microseconds == 0) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  auto result = hal::vmx::GetDIOManager().Pulse(dioPortHandle, microseconds);
  if (result == hal::vmx::DIOResult::kOk) {
    *status = HAL_SUCCESS;
  } else if (result == hal::vmx::DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
  } else if (result == hal::vmx::DIOResult::kInputChannel) {
    *status = PARAMETER_OUT_OF_RANGE;
    hal::SetLastError(status, "Cannot pulse a VMX DIO input channel");
  } else {
    hal::vmx::SetHardwareError(status, "Failed to start VMX DIO pulse");
  }
}

void HAL_PulseMultiple(uint32_t channelMask, double pulseLengthSeconds,
                       int32_t* status) {
  if ((channelMask >> hal::vmx::kNumDIOChannels) != 0 ||
      !std::isfinite(pulseLengthSeconds) || pulseLengthSeconds <= 0.0 ||
      pulseLengthSeconds >
          static_cast<double>(std::numeric_limits<uint32_t>::max()) / 1.0e6) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  auto microseconds =
      static_cast<uint32_t>(std::llround(pulseLengthSeconds * 1.0e6));
  if (microseconds == 0) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  auto result =
      hal::vmx::GetDIOManager().PulseMultiple(channelMask, microseconds);
  if (result == hal::vmx::DIOResult::kOk) {
    *status = HAL_SUCCESS;
  } else if (result == hal::vmx::DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
  } else if (result == hal::vmx::DIOResult::kInputChannel) {
    *status = PARAMETER_OUT_OF_RANGE;
  } else {
    hal::vmx::SetHardwareError(status,
                               "Failed to start multiple VMX DIO pulses");
  }
}

HAL_Bool HAL_IsPulsing(HAL_DigitalHandle dioPortHandle, int32_t* status) {
  auto [result, pulsing] =
      hal::vmx::GetDIOManager().IsPulsing(dioPortHandle);
  if (result == hal::vmx::DIOResult::kOk) {
    *status = HAL_SUCCESS;
    return pulsing;
  }
  if (result == hal::vmx::DIOResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
  } else if (result == hal::vmx::DIOResult::kInputChannel) {
    *status = PARAMETER_OUT_OF_RANGE;
  } else {
    hal::vmx::SetHardwareError(status, "Failed to read VMX DIO pulse state");
  }
  return false;
}

HAL_Bool HAL_IsAnyPulsing(int32_t* status) {
  auto [result, pulsing] = hal::vmx::GetDIOManager().IsAnyPulsing();
  if (result == hal::vmx::DIOResult::kOk) {
    *status = HAL_SUCCESS;
    return pulsing;
  }
  hal::vmx::SetHardwareError(status, "Failed to read VMX DIO pulse state");
  return false;
}

void HAL_SetFilterSelect(HAL_DigitalHandle dioPortHandle, int32_t filterIndex,
                         int32_t* status) {
  static_cast<void>(filterIndex);
  if (hal::vmx::CheckHandleForUnsupported(dioPortHandle, status)) {
    hal::vmx::SetUnsupported(status, "DIO glitch filters");
  }
}

int32_t HAL_GetFilterSelect(HAL_DigitalHandle dioPortHandle,
                            int32_t* status) {
  if (hal::vmx::CheckHandleForUnsupported(dioPortHandle, status)) {
    hal::vmx::SetUnsupported(status, "DIO glitch filters");
  }
  return 0;
}

void HAL_SetFilterPeriod(int32_t filterIndex, int64_t value,
                         int32_t* status) {
  static_cast<void>(filterIndex);
  static_cast<void>(value);
  hal::vmx::SetUnsupported(status, "DIO glitch filters");
}

int64_t HAL_GetFilterPeriod(int32_t filterIndex, int32_t* status) {
  static_cast<void>(filterIndex);
  hal::vmx::SetUnsupported(status, "DIO glitch filters");
  return 0;
}

}  // extern "C"
