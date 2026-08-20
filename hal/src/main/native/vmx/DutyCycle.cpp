// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/DutyCycle.h"

#include <memory>
#include <string_view>

#include "DIOInternal.h"
#include "HALInitializer.h"
#include "HALInternal.h"
#include "DutyCycleInternal.h"
#include "VMXPi.h"
#include "VMXDigitalSource.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"

namespace hal::vmx {
namespace {

class DriverDutyCycleBackend final : public DutyCycleBackend {
 public:
  DriverDutyCycleBackend(int32_t physicalChannel,
                         std::shared_ptr<VMXPi> context)
      : m_context{std::move(context)} {
    if (!m_context || !m_context->IsOpen() || physicalChannel < 0 ||
        physicalChannel >= 12) {
      return;
    }
    PWMCaptureConfig config;
    // Keep the SDK's documented timeout behavior: a stalled signal returns
    // zero period/high-time values instead of stale capture data.
    config.SetTimeout(PWMCaptureConfig::PWMCaptureTimeout::x2);
    VMXErrorCode error;
    m_initialized = m_context->io.ActivateSinglechannelResource(
        ::VMXChannelInfo(static_cast<VMXChannelIndex>(physicalChannel),
                         VMXChannelCapability::PWMCaptureInput),
        &config, m_resourceHandle, &error);
  }

  ~DriverDutyCycleBackend() override {
    if (m_initialized) {
      VMXErrorCode error;
      m_context->io.DeallocateResource(m_resourceHandle, &error);
    }
  }

  bool GetTiming(uint32_t& periodMicroseconds,
                 uint32_t& highMicroseconds) noexcept override {
    periodMicroseconds = 0;
    highMicroseconds = 0;
    if (!m_initialized) {
      return false;
    }
    VMXErrorCode error;
    return m_context->io.PWMCapture_GetCount(
        m_resourceHandle, periodMicroseconds, highMicroseconds, &error);
  }

  bool IsInitialized() const noexcept { return m_initialized; }

 private:
  std::shared_ptr<VMXPi> m_context;
  VMXResourceHandle m_resourceHandle = 0;
  bool m_initialized = false;
};

std::unique_ptr<DutyCycleBackend> CreateDutyCycleBackend(
    int32_t physicalChannel) {
  auto context = GetRuntimeContext();
  if (!context) {
    return nullptr;
  }
  auto backend = std::make_unique<DriverDutyCycleBackend>(
      physicalChannel, std::move(context));
  return backend->IsInitialized() ? std::move(backend) : nullptr;
}

DutyCycleSourceClaim ClaimDutyCycleSource(HAL_Handle source) {
  auto decoded = DecodeVMXDigitalSource(source, HAL_Trigger_kInWindow);
  if (decoded == VMXDigitalSourceResult::kUnsupportedAnalogTrigger) {
    return {DutyCycleResult::kUnsupportedSource, -1};
  }
  if (decoded != VMXDigitalSourceResult::kOk) {
    return {DutyCycleResult::kInvalidSource, -1};
  }
  auto claim = GetDIOManager().ClaimResourceSource(
      source, DigitalChannelOwner::kDutyCycle, "VMX DutyCycle source");
  switch (claim.result) {
    case DIOResult::kOk:
      return {DutyCycleResult::kOk, claim.channelA};
    case DIOResult::kAlreadyAllocated:
      return {DutyCycleResult::kAlreadyAllocated, claim.channelA};
    case DIOResult::kUnsupportedCapability:
      return {DutyCycleResult::kUnsupportedSource, claim.channelA};
    case DIOResult::kInvalidHandle:
      return {DutyCycleResult::kInvalidSource, claim.channelA};
    default:
      return {DutyCycleResult::kHardwareFailure, claim.channelA};
  }
}

void ReleaseDutyCycleSource(HAL_Handle source, int32_t physicalChannel) {
  GetDIOManager().ReleaseResourceSource(
      source, physicalChannel, DigitalChannelOwner::kDutyCycle);
}

void SetDutyCycleResult(DutyCycleResult result, int32_t* status,
                        std::string_view message) {
  switch (result) {
    case DutyCycleResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case DutyCycleResult::kInvalidHandle:
    case DutyCycleResult::kInvalidSource:
      *status = HAL_HANDLE_ERROR;
      return;
    case DutyCycleResult::kUnsupportedSource:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
    case DutyCycleResult::kAlreadyAllocated:
      *status = RESOURCE_IS_ALLOCATED;
      return;
    case DutyCycleResult::kNoResources:
      *status = NO_AVAILABLE_RESOURCES;
      return;
    case DutyCycleResult::kOutOfRange:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status, message);
      return;
    case DutyCycleResult::kHardwareFailure:
    default:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
  }
}

}  // namespace

DutyCycleManager& GetDutyCycleManager() {
  static DutyCycleManager manager{CreateDutyCycleBackend,
                                  ClaimDutyCycleSource,
                                  ReleaseDutyCycleSource};
  return manager;
}

}  // namespace hal::vmx

extern "C" {

HAL_DutyCycleHandle HAL_InitializeDutyCycle(HAL_Handle digitalSourceHandle,
                                            HAL_AnalogTriggerType triggerType,
                                            int32_t* status) {
  hal::init::CheckInit();
  static_cast<void>(triggerType);
  if (!hal::vmx::IsRuntimeInitialized()) {
    *status = INCOMPATIBLE_STATE;
    hal::SetLastError(status, "VMX HAL runtime is not initialized");
    return HAL_kInvalidHandle;
  }
  auto allocation =
      hal::vmx::GetDutyCycleManager().Allocate(digitalSourceHandle);
  hal::vmx::SetDutyCycleResult(
      allocation.result, status,
      "VMX DutyCycle requires a FlexDIO input with PWMCapture capability");
  return allocation.result == hal::vmx::DutyCycleResult::kOk
             ? allocation.handle
             : HAL_kInvalidHandle;
}

void HAL_FreeDutyCycle(HAL_DutyCycleHandle dutyCycleHandle) {
  hal::vmx::GetDutyCycleManager().Free(dutyCycleHandle);
}

void HAL_SetDutyCycleSimDevice(HAL_DutyCycleHandle handle,
                               HAL_SimDeviceHandle device) {
  static_cast<void>(handle);
  static_cast<void>(device);
}

int32_t HAL_GetDutyCycleFrequency(HAL_DutyCycleHandle dutyCycleHandle,
                                  int32_t* status) {
  auto result =
      hal::vmx::GetDutyCycleManager().GetFrequency(dutyCycleHandle);
  hal::vmx::SetDutyCycleResult(result.first, status,
                               "Failed to read VMX DutyCycle frequency");
  return result.first == hal::vmx::DutyCycleResult::kOk ? result.second : 0;
}

double HAL_GetDutyCycleOutput(HAL_DutyCycleHandle dutyCycleHandle,
                              int32_t* status) {
  auto result = hal::vmx::GetDutyCycleManager().GetOutput(dutyCycleHandle);
  hal::vmx::SetDutyCycleResult(result.first, status,
                               "Failed to read VMX DutyCycle output");
  return result.first == hal::vmx::DutyCycleResult::kOk ? result.second : 0.0;
}

int32_t HAL_GetDutyCycleHighTime(HAL_DutyCycleHandle dutyCycleHandle,
                                 int32_t* status) {
  auto result = hal::vmx::GetDutyCycleManager().GetHighTime(dutyCycleHandle);
  hal::vmx::SetDutyCycleResult(result.first, status,
                               "VMX DutyCycle high time is out of range");
  return result.first == hal::vmx::DutyCycleResult::kOk ? result.second : 0;
}

int32_t HAL_GetDutyCycleOutputScaleFactor(HAL_DutyCycleHandle dutyCycleHandle,
                                          int32_t* status) {
  auto result =
      hal::vmx::GetDutyCycleManager().GetFPGAIndex(dutyCycleHandle);
  hal::vmx::SetDutyCycleResult(result.first, status,
                               "Invalid VMX DutyCycle handle");
  return result.first == hal::vmx::DutyCycleResult::kOk
             ? hal::vmx::kDutyCycleOutputScaleFactor
             : 0;
}

int32_t HAL_GetDutyCycleFPGAIndex(HAL_DutyCycleHandle dutyCycleHandle,
                                  int32_t* status) {
  auto result =
      hal::vmx::GetDutyCycleManager().GetFPGAIndex(dutyCycleHandle);
  hal::vmx::SetDutyCycleResult(result.first, status,
                               "Invalid VMX DutyCycle handle");
  return result.first == hal::vmx::DutyCycleResult::kOk ? result.second : -1;
}

}  // extern "C"
