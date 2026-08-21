// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AddressableLEDInternal.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string_view>

#include "VMXPi.h"

#include "HALInitializer.h"
#include "HALInternal.h"
#include "VMXRuntime.h"
#include "PWMInternal.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"

namespace hal::vmx {
namespace {

LEDArray_OneWireConfig::PixelFormat ToVMXPixelFormat(
    HAL_AddressableLEDColorOrder order) noexcept {
  switch (order) {
    case HAL_ALED_RGB:
      return LEDArray_OneWireConfig::PixelFormat::RGB;
    case HAL_ALED_RBG:
      return LEDArray_OneWireConfig::PixelFormat::RBG;
    case HAL_ALED_BGR:
      return LEDArray_OneWireConfig::PixelFormat::BGR;
    case HAL_ALED_BRG:
      return LEDArray_OneWireConfig::PixelFormat::BRG;
    case HAL_ALED_GBR:
      return LEDArray_OneWireConfig::PixelFormat::GBR;
    case HAL_ALED_GRB:
      return LEDArray_OneWireConfig::PixelFormat::GRB;
  }
  return LEDArray_OneWireConfig::PixelFormat::GRB;
}

class DriverAddressableLEDBackend final : public AddressableLEDBackend {
 public:
  explicit DriverAddressableLEDBackend(std::shared_ptr<VMXPi> context)
      : m_context{std::move(context)} {}

  ~DriverAddressableLEDBackend() override { Close(); }

  bool Configure(int32_t physicalChannel,
                 const AddressableLEDConfiguration& configuration,
                 const std::vector<HAL_AddressableLEDData>& data,
                 bool running) noexcept override {
    try {
      Destroy();
      if (!m_context || !m_context->IsOpen() || physicalChannel < 0 ||
          configuration.length < 0 ||
          configuration.length != static_cast<int32_t>(data.size()) ||
          configuration.targetFrequencyHz <= 0 ||
          configuration.highTime0Nanoseconds < 0 ||
          configuration.highTime1Nanoseconds < 0 ||
          configuration.highTime0Nanoseconds >=
              (std::numeric_limits<int32_t>::max)() ||
          configuration.highTime1Nanoseconds >=
              (std::numeric_limits<int32_t>::max)()) {
        return false;
      }

      LEDArray_OneWireConfig sdkConfig{configuration.length,
                                       configuration.targetFrequencyHz};
      sdkConfig.pixel_format = ToVMXPixelFormat(configuration.colorOrder);
      sdkConfig.zero_symbol_high_time_ns =
          static_cast<uint64_t>(configuration.highTime0Nanoseconds);
      sdkConfig.one_symbol_high_time_ns =
          static_cast<uint64_t>(configuration.highTime1Nanoseconds);
      sdkConfig.reset_wait_time_us = configuration.resetWaitMicroseconds;

      VMXErrorCode error = 0;
      if (!m_context->io.ActivateSinglechannelResource(
              ::VMXChannelInfo(
                  static_cast<VMXChannelIndex>(physicalChannel),
                  VMXChannelCapability::LEDArray_OneWire),
              &sdkConfig, m_resourceHandle, &error)) {
        return false;
      }
      m_resourceActive = true;
      if (!m_context->io.LEDArrayBuffer_Create(configuration.length,
                                               m_buffer, &error)) {
        Destroy();
        return false;
      }
      for (int32_t i = 0; i < configuration.length; ++i) {
        const auto& pixel = data[static_cast<size_t>(i)];
        if (!m_context->io.LEDArrayBuffer_SetRGBValue(
                m_buffer, i, pixel.r, pixel.g, pixel.b, &error)) {
          Destroy();
          return false;
        }
      }
      if (!m_context->io.LEDArray_SetBuffer(m_resourceHandle, m_buffer,
                                            &error)) {
        Destroy();
        return false;
      }
      m_physicalChannel = physicalChannel;
      m_configuration = configuration;
      m_data = data;
      m_started = running;
      return true;
    } catch (...) {
      Destroy();
      return false;
    }
  }

  bool Write(const std::vector<HAL_AddressableLEDData>& data) noexcept override {
    try {
      if (!m_resourceActive || data.size() != m_data.size()) {
        return false;
      }
      VMXErrorCode error = 0;
      for (size_t i = 0; i < data.size(); ++i) {
        const auto& pixel = data[i];
        if (!m_context->io.LEDArrayBuffer_SetRGBValue(
                m_buffer, static_cast<int>(i), pixel.r, pixel.g, pixel.b,
                &error)) {
          return false;
        }
      }
      if (!m_context->io.LEDArray_SetBuffer(m_resourceHandle, m_buffer,
                                            &error)) {
        return false;
      }
      m_data = data;
      return true;
    } catch (...) {
      return false;
    }
  }

  bool Render() noexcept override {
    try {
      if (!m_resourceActive) {
        return false;
      }
      VMXErrorCode error = 0;
      return m_context->io.LEDArray_Render(m_resourceHandle, &error);
    } catch (...) {
      return false;
    }
  }

  bool Start() noexcept override {
    if (m_resourceActive) {
      m_started = true;
      return true;
    }
    return Configure(m_physicalChannel, m_configuration, m_data, true);
  }

  bool Stop() noexcept override {
    try {
      Destroy();
      m_started = false;
      return true;
    } catch (...) {
      return false;
    }
  }

  void Close() noexcept override {
    try {
      Destroy();
    } catch (...) {
    }
    m_context.reset();
  }

 private:
  void Destroy() noexcept {
    if (!m_context) {
      return;
    }
    VMXErrorCode error = 0;
    if (m_buffer != nullptr) {
      static_cast<void>(m_context->io.LEDArrayBuffer_Delete(m_buffer, &error));
      m_buffer = nullptr;
    }
    if (m_resourceActive) {
      static_cast<void>(
          m_context->io.DeallocateResource(m_resourceHandle, &error));
      m_resourceActive = false;
    }
  }

  std::shared_ptr<VMXPi> m_context;
  VMXResourceHandle m_resourceHandle = 0;
  LEDArrayBufferHandle m_buffer = nullptr;
  int32_t m_physicalChannel = -1;
  bool m_resourceActive = false;
  bool m_started = false;
  AddressableLEDConfiguration m_configuration;
  std::vector<HAL_AddressableLEDData> m_data;
};

std::unique_ptr<AddressableLEDBackend> CreateAddressableLEDBackend() {
  auto context = GetRuntimeContext();
  if (!context) {
    return nullptr;
  }
  return std::make_unique<DriverAddressableLEDBackend>(std::move(context));
}

void SetAddressableLEDStatus(AddressableLEDResult result, int32_t* status,
                             std::string_view message) {
  if (!status) {
    return;
  }
  switch (result) {
    case AddressableLEDResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case AddressableLEDResult::kInvalidHandle:
      *status = HAL_HANDLE_ERROR;
      return;
    case AddressableLEDResult::kInvalidChannel:
      *status = HAL_LED_CHANNEL_ERROR;
      hal::SetLastError(status, message);
      return;
    case AddressableLEDResult::kAlreadyAllocated:
      *status = RESOURCE_IS_ALLOCATED;
      hal::SetLastError(status, message);
      return;
    case AddressableLEDResult::kUnsupportedCapability:
      *status = HAL_LED_CHANNEL_ERROR;
      hal::SetLastError(status, message);
      return;
    case AddressableLEDResult::kInvalidParameter:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status, message);
      return;
    case AddressableLEDResult::kHardwareFailure:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
  }
}

}  // namespace

bool AddressableLEDManager::ValidateOutputPort(
    HAL_DigitalHandle outputPort, int32_t& logicalChannel,
    int32_t& physicalChannel) const noexcept {
  logicalChannel = -1;
  physicalChannel = -1;
  if (hal::getHandleType(outputPort) != HAL_HandleEnum::PWM) {
    return false;
  }
  logicalChannel = hal::getHandleIndex(outputPort);
  if (!IsPWMChannelValid(logicalChannel)) {
    return false;
  }
  physicalChannel = ToVMXDigitalChannel(logicalChannel);
  if (m_capabilities &&
      !m_capabilities->SupportsPhysical(physicalChannel,
                                        VMXCapability::kAddressableLED)) {
    return false;
  }
  return true;
}

AddressableLEDResult AddressableLEDManager::Initialize(
    HAL_DigitalHandle outputPort, std::string_view allocationLocation,
    HAL_AddressableLEDHandle& handle) noexcept {
  handle = HAL_kInvalidHandle;
  std::scoped_lock allocationLock{m_allocationMutex};
  int32_t logicalChannel = -1;
  int32_t physicalChannel = -1;
  if (!ValidateOutputPort(outputPort, logicalChannel, physicalChannel)) {
    return AddressableLEDResult::kInvalidChannel;
  }
  const auto reservation = m_registry.Reserve(
      physicalChannel, DigitalChannelOwner::kAddressableLED,
      allocationLocation);
  if (reservation.reserved) {
    // A VMX addressable LED must be backed by the PWM handle that WPILib
    // created immediately before this call. A free physical channel means the
    // caller bypassed that lifecycle and is rejected below.
    m_registry.Release(physicalChannel, DigitalChannelOwner::kAddressableLED);
    return AddressableLEDResult::kAlreadyAllocated;
  }
  if (reservation.previousOwner != DigitalChannelOwner::kPWM ||
      !m_suspendPWM || !m_suspendPWM(outputPort) ||
      !m_registry.Transfer(physicalChannel, DigitalChannelOwner::kPWM,
                           DigitalChannelOwner::kAddressableLED,
                           allocationLocation)) {
    return reservation.previousOwner == DigitalChannelOwner::kNone
               ? AddressableLEDResult::kInvalidChannel
               : AddressableLEDResult::kAlreadyAllocated;
  }

  int32_t status = HAL_SUCCESS;
  auto port = m_handles.Allocate(0, &handle, &status);
  if (!port || status != HAL_SUCCESS ||
      port->Initialize(physicalChannel, allocationLocation, m_factory) !=
          AddressableLEDResult::kOk) {
    m_registry.Transfer(physicalChannel, DigitalChannelOwner::kAddressableLED,
                        DigitalChannelOwner::kPWM, "restored PWM output");
    if (port) {
      m_handles.Free(handle);
    }
    handle = HAL_kInvalidHandle;
    return AddressableLEDResult::kHardwareFailure;
  }
  return AddressableLEDResult::kOk;
}

AddressableLEDResult AddressableLEDManager::SetOutputPort(
    HAL_AddressableLEDHandle handle, HAL_DigitalHandle outputPort) noexcept {
  std::scoped_lock allocationLock{m_allocationMutex};
  auto port = Get(handle);
  if (!port) {
    return AddressableLEDResult::kInvalidHandle;
  }
  int32_t logicalChannel = -1;
  int32_t physicalChannel = -1;
  if (!ValidateOutputPort(outputPort, logicalChannel, physicalChannel)) {
    return AddressableLEDResult::kInvalidChannel;
  }
  const auto oldPhysical = port->GetPhysicalChannel();
  if (oldPhysical == physicalChannel) {
    return AddressableLEDResult::kOk;
  }
  const auto reservation = m_registry.Reserve(
      physicalChannel, DigitalChannelOwner::kAddressableLED, "AddressableLED");
  if (reservation.reserved || reservation.previousOwner !=
                                 DigitalChannelOwner::kPWM ||
      !m_suspendPWM || !m_suspendPWM(outputPort) ||
      !m_registry.Transfer(physicalChannel, DigitalChannelOwner::kPWM,
                           DigitalChannelOwner::kAddressableLED,
                           "AddressableLED")) {
    if (reservation.reserved) {
      m_registry.Release(physicalChannel, DigitalChannelOwner::kAddressableLED);
    }
    return AddressableLEDResult::kAlreadyAllocated;
  }
  if (port->SetOutputPort(physicalChannel) != AddressableLEDResult::kOk) {
    m_registry.Transfer(physicalChannel, DigitalChannelOwner::kAddressableLED,
                        DigitalChannelOwner::kPWM, "restored PWM output");
    return AddressableLEDResult::kHardwareFailure;
  }
  m_registry.Transfer(oldPhysical, DigitalChannelOwner::kAddressableLED,
                      DigitalChannelOwner::kPWM, "restored PWM output");
  return AddressableLEDResult::kOk;
}

AddressableLEDResult AddressableLEDManager::SetColorOrder(
    HAL_AddressableLEDHandle handle, HAL_AddressableLEDColorOrder order) noexcept {
  auto port = Get(handle);
  return port ? port->SetColorOrder(order) : AddressableLEDResult::kInvalidHandle;
}

AddressableLEDResult AddressableLEDManager::SetLength(
    HAL_AddressableLEDHandle handle, int32_t length) noexcept {
  auto port = Get(handle);
  return port ? port->SetLength(length) : AddressableLEDResult::kInvalidHandle;
}

AddressableLEDResult AddressableLEDManager::Write(
    HAL_AddressableLEDHandle handle, const HAL_AddressableLEDData* data,
    int32_t length) noexcept {
  auto port = Get(handle);
  return port ? port->Write(data, length) : AddressableLEDResult::kInvalidHandle;
}

AddressableLEDResult AddressableLEDManager::SetBitTiming(
    HAL_AddressableLEDHandle handle, int32_t highTime0Nanoseconds,
    int32_t lowTime0Nanoseconds, int32_t highTime1Nanoseconds,
    int32_t lowTime1Nanoseconds) noexcept {
  auto port = Get(handle);
  return port ? port->SetBitTiming(highTime0Nanoseconds, lowTime0Nanoseconds,
                                  highTime1Nanoseconds, lowTime1Nanoseconds)
              : AddressableLEDResult::kInvalidHandle;
}

AddressableLEDResult AddressableLEDManager::SetSyncTime(
    HAL_AddressableLEDHandle handle, int32_t syncTimeMicroseconds) noexcept {
  auto port = Get(handle);
  return port ? port->SetSyncTime(syncTimeMicroseconds)
              : AddressableLEDResult::kInvalidHandle;
}

AddressableLEDResult AddressableLEDManager::Start(
    HAL_AddressableLEDHandle handle) noexcept {
  auto port = Get(handle);
  return port ? port->Start() : AddressableLEDResult::kInvalidHandle;
}

AddressableLEDResult AddressableLEDManager::Stop(
    HAL_AddressableLEDHandle handle) noexcept {
  auto port = Get(handle);
  return port ? port->Stop() : AddressableLEDResult::kInvalidHandle;
}

void AddressableLEDManager::Free(HAL_AddressableLEDHandle handle) noexcept {
  std::scoped_lock allocationLock{m_allocationMutex};
  auto port = Get(handle);
  if (!port) {
    return;
  }
  const auto physicalChannel = port->GetPhysicalChannel();
  port->Close();
  m_handles.Free(handle);
  m_registry.Transfer(physicalChannel, DigitalChannelOwner::kAddressableLED,
                      DigitalChannelOwner::kPWM, "restored PWM output");
}

void AddressableLEDManager::Shutdown() noexcept {
  std::scoped_lock allocationLock{m_allocationMutex};
  // The VMX implementation supports one addressable LED. Constructing the
  // typed handle directly avoids exposing any new public shutdown ABI.
  const auto typedHandle = static_cast<HAL_AddressableLEDHandle>(
      hal::createHandle(0, HAL_HandleEnum::AddressableLED, 0));
  auto port = Get(typedHandle);
  if (port) {
    const auto physicalChannel = port->GetPhysicalChannel();
    port->Close();
    m_handles.Free(typedHandle);
    m_registry.Transfer(physicalChannel, DigitalChannelOwner::kAddressableLED,
                        DigitalChannelOwner::kPWM, "restored PWM output");
  }
}

AddressableLEDManager& GetAddressableLEDManager() {
  static AddressableLEDManager manager{
      CreateAddressableLEDBackend, GetDigitalChannelRegistry(),
      &GetVMXCapabilityProvider(),
      [](HAL_DigitalHandle handle) {
        return GetPWMManager().Disable(handle) == PWMResult::kOk;
      }};
  return manager;
}

}  // namespace hal::vmx

extern "C" {

HAL_AddressableLEDHandle HAL_InitializeAddressableLED(
    HAL_DigitalHandle outputPort, int32_t* status) {
  hal::init::CheckInit();
  HAL_AddressableLEDHandle handle = HAL_kInvalidHandle;
  const auto result = hal::vmx::GetAddressableLEDManager().Initialize(
      outputPort, "AddressableLED", handle);
  hal::vmx::SetAddressableLEDStatus(
      result, status, "VMX addressable LED requires a PWM-capable output");
  return result == hal::vmx::AddressableLEDResult::kOk ? handle
                                                       : HAL_kInvalidHandle;
}

void HAL_FreeAddressableLED(HAL_AddressableLEDHandle handle) {
  hal::vmx::GetAddressableLEDManager().Free(handle);
}

void HAL_SetAddressableLEDColorOrder(HAL_AddressableLEDHandle handle,
                                     HAL_AddressableLEDColorOrder colorOrder,
                                     int32_t* status) {
  hal::vmx::SetAddressableLEDStatus(
      hal::vmx::GetAddressableLEDManager().SetColorOrder(handle, colorOrder),
      status, "Invalid VMX addressable LED color order");
}

void HAL_SetAddressableLEDOutputPort(HAL_AddressableLEDHandle handle,
                                     HAL_DigitalHandle outputPort,
                                     int32_t* status) {
  hal::vmx::SetAddressableLEDStatus(
      hal::vmx::GetAddressableLEDManager().SetOutputPort(handle, outputPort),
      status, "VMX addressable LED output port is unavailable");
}

void HAL_SetAddressableLEDLength(HAL_AddressableLEDHandle handle,
                                 int32_t length, int32_t* status) {
  hal::vmx::SetAddressableLEDStatus(
      hal::vmx::GetAddressableLEDManager().SetLength(handle, length), status,
      "Invalid VMX addressable LED length");
}

void HAL_WriteAddressableLEDData(
    HAL_AddressableLEDHandle handle, const struct HAL_AddressableLEDData* data,
    int32_t length, int32_t* status) {
  hal::vmx::SetAddressableLEDStatus(
      hal::vmx::GetAddressableLEDManager().Write(handle, data, length), status,
      "Invalid VMX addressable LED data buffer");
}

void HAL_SetAddressableLEDBitTiming(HAL_AddressableLEDHandle handle,
                                    int32_t highTime0NanoSeconds,
                                    int32_t lowTime0NanoSeconds,
                                    int32_t highTime1NanoSeconds,
                                    int32_t lowTime1NanoSeconds,
                                    int32_t* status) {
  hal::vmx::SetAddressableLEDStatus(
      hal::vmx::GetAddressableLEDManager().SetBitTiming(
          handle, highTime0NanoSeconds, lowTime0NanoSeconds,
          highTime1NanoSeconds, lowTime1NanoSeconds),
      status, "VMX cannot represent the requested addressable LED timing");
}

void HAL_SetAddressableLEDSyncTime(HAL_AddressableLEDHandle handle,
                                   int32_t syncTimeMicroSeconds,
                                   int32_t* status) {
  hal::vmx::SetAddressableLEDStatus(
      hal::vmx::GetAddressableLEDManager().SetSyncTime(handle,
                                                       syncTimeMicroSeconds),
      status, "Invalid VMX addressable LED sync time");
}

void HAL_StartAddressableLEDOutput(HAL_AddressableLEDHandle handle,
                                   int32_t* status) {
  hal::vmx::SetAddressableLEDStatus(
      hal::vmx::GetAddressableLEDManager().Start(handle), status,
      "Failed to start VMX addressable LED output");
}

void HAL_StopAddressableLEDOutput(HAL_AddressableLEDHandle handle,
                                  int32_t* status) {
  hal::vmx::SetAddressableLEDStatus(
      hal::vmx::GetAddressableLEDManager().Stop(handle), status,
      "Failed to stop VMX addressable LED output");
}

}  // extern "C"
