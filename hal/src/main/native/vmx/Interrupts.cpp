// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/Interrupts.h"

#include <memory>
#include <limits>
#include <string_view>

#include "DIOInternal.h"
#include "HALInitializer.h"
#include "HALInternal.h"
#include "InterruptInternal.h"
#include "VMXChannelCapabilities.h"
#include "VMXDigitalSource.h"
#include "VMXPi.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"

namespace hal::vmx {
namespace {

class DriverInterruptBackend final : public InterruptBackend {
 public:
  DriverInterruptBackend(int32_t channel, VMXInterruptEdge edge,
                         InterruptCallbackState* state,
                         std::shared_ptr<VMXPi> context)
      : m_context{std::move(context)} {
    if (!m_context || !m_context->IsOpen() || !state) {
      return;
    }
    InterruptConfig config{ToSdkEdge(edge), &OnInterrupt, state, false};
    ::VMXChannelInfo channelInfo(
        static_cast<VMXChannelIndex>(channel),
        VMXChannelCapability::InterruptInput);
    VMXErrorCode error;
    m_initialized = m_context->io.ActivateSinglechannelResource(
        channelInfo, &config, m_resourceHandle, &error);
    if (m_initialized &&
        !m_context->io.Interrupt_SetEnabled(m_resourceHandle, true, &error)) {
      m_context->io.DeactivateResource(m_resourceHandle, &error);
      m_context->io.DeallocateResource(m_resourceHandle, &error);
      m_resourceHandle = 0;
      m_initialized = false;
    }
  }

  ~DriverInterruptBackend() override {
    if (!m_initialized) {
      return;
    }
    VMXErrorCode error;
    m_context->io.Interrupt_SetEnabled(m_resourceHandle, false, &error);
    m_context->io.DeactivateResource(m_resourceHandle, &error);
    m_context->io.DeallocateResource(m_resourceHandle, &error);
  }

  bool SetEnabled(bool enabled) noexcept override {
    if (!m_initialized) {
      return false;
    }
    VMXErrorCode error;
    return m_context->io.Interrupt_SetEnabled(m_resourceHandle, enabled,
                                              &error);
  }

  bool GetEnabled(bool& enabled) noexcept override {
    enabled = false;
    if (!m_initialized) {
      return false;
    }
    VMXErrorCode error;
    return m_context->io.Interrupt_GetEnabled(m_resourceHandle, enabled,
                                              &error);
  }

  bool ReadTimestamp(bool rising, uint64_t& timestampUs) noexcept override {
    timestampUs = 0;
    if (!m_initialized) {
      return false;
    }
    VMXErrorCode error;
    return rising
               ? m_context->io.Interrupt_GetLastRisingEdgeTimestampMicroseconds(
                     m_resourceHandle, timestampUs, &error)
               : m_context->io.Interrupt_GetLastFallingEdgeTimestampMicroseconds(
                     m_resourceHandle, timestampUs, &error);
  }

  VMXResourceHandle GetResourceHandle() const noexcept override {
    return m_initialized ? m_resourceHandle : 0;
  }

  bool IsInitialized() const noexcept { return m_initialized; }

 private:
  static InterruptConfig::InterruptEdge ToSdkEdge(
      VMXInterruptEdge edge) noexcept {
    switch (edge) {
      case VMXInterruptEdge::kFalling:
        return InterruptConfig::FALLING;
      case VMXInterruptEdge::kBoth:
        return InterruptConfig::BOTH;
      case VMXInterruptEdge::kRising:
      default:
        return InterruptConfig::RISING;
    }
  }

  static void OnInterrupt(uint32_t, InterruptEdgeType edge, void* param,
                          uint64_t timestampUs) noexcept {
    auto* state = static_cast<InterruptCallbackState*>(param);
    if (state) {
      state->OnHardwareEvent(edge == RISING_EDGE_INTERRUPT, timestampUs);
    }
  }

  std::shared_ptr<VMXPi> m_context;
  VMXResourceHandle m_resourceHandle = 0;
  bool m_initialized = false;
};

std::unique_ptr<InterruptBackend> CreateInterruptBackend(
    int32_t channel, VMXInterruptEdge edge, InterruptCallbackState* state) {
  auto context = GetRuntimeContext();
  if (!context) {
    return nullptr;
  }
  auto backend = std::make_unique<DriverInterruptBackend>(
      channel, edge, state, std::move(context));
  return backend->IsInitialized() ? std::move(backend) : nullptr;
}

InterruptManager& GetInterruptManager() {
  static InterruptManager manager{CreateInterruptBackend};
  return manager;
}

InterruptResult ValidateInterruptSource(HAL_Handle sourceHandle,
                                         HAL_AnalogTriggerType triggerType,
                                         int32_t& channel) {
  auto decoded = DecodeVMXDigitalSource(sourceHandle, triggerType);
  if (decoded == VMXDigitalSourceResult::kUnsupportedAnalogTrigger) {
    return InterruptResult::kUnsupportedSource;
  }
  if (decoded != VMXDigitalSourceResult::kOk) {
    return InterruptResult::kInvalidSource;
  }
  auto validation = GetDIOManager().ValidateInputSource(sourceHandle);
  channel = validation.second;
  switch (validation.first) {
    case DIOResult::kOk:
      if (IsRuntimeInitialized() &&
          !GetVMXCapabilityProvider().SupportsPhysical(
              channel, VMXCapability::kInterruptInput)) {
        return InterruptResult::kUnsupportedSource;
      }
      return InterruptResult::kOk;
    case DIOResult::kAlreadyAllocated:
      return InterruptResult::kAlreadyAllocated;
    case DIOResult::kInvalidHandle:
      return InterruptResult::kInvalidSource;
    case DIOResult::kOutputChannel:
      return InterruptResult::kInvalidSource;
    default:
      return InterruptResult::kHardwareFailure;
  }
}

void SetInterruptResult(InterruptResult result, int32_t* status,
                        std::string_view message) {
  switch (result) {
    case InterruptResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case InterruptResult::kInvalidHandle:
    case InterruptResult::kInvalidSource:
      *status = HAL_HANDLE_ERROR;
      return;
    case InterruptResult::kUnsupportedSource:
    case InterruptResult::kUnconfigured:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
    case InterruptResult::kOutOfRange:
      *status = PARAMETER_OUT_OF_RANGE;
      return;
    case InterruptResult::kAlreadyAllocated:
      *status = RESOURCE_IS_ALLOCATED;
      return;
    case InterruptResult::kNoResources:
      *status = NO_AVAILABLE_RESOURCES;
      return;
    case InterruptResult::kHardwareFailure:
    default:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
  }
}

template <typename T>
T ReturnInterruptValue(std::pair<InterruptResult, T> result, int32_t* status,
                       std::string_view message, T fallback = T{}) {
  SetInterruptResult(result.first, status, message);
  return result.first == InterruptResult::kOk ? result.second : fallback;
}

}  // namespace

InterruptBackendFactory GetInterruptBackendFactory() {
  return CreateInterruptBackend;
}
}  // namespace hal::vmx

extern "C" {

HAL_InterruptHandle HAL_InitializeInterrupts(int32_t* status) {
  hal::init::CheckInit();
  if (!hal::vmx::IsRuntimeInitialized()) {
    *status = INCOMPATIBLE_STATE;
    hal::SetLastError(status, "VMX HAL runtime is not initialized");
    return HAL_kInvalidHandle;
  }
  auto allocation = hal::vmx::GetInterruptManager().Allocate(nullptr);
  hal::vmx::SetInterruptResult(
      allocation.result, status, "Failed to allocate VMX interrupt handle");
  return allocation.result == hal::vmx::InterruptResult::kOk
             ? allocation.handle
             : HAL_kInvalidHandle;
}

void HAL_CleanInterrupts(HAL_InterruptHandle interruptHandle) {
  hal::vmx::GetInterruptManager().Free(interruptHandle);
}

void HAL_RequestInterrupts(HAL_InterruptHandle interruptHandle,
                           HAL_Handle digitalSourceHandle,
                           HAL_AnalogTriggerType analogTriggerType,
                           int32_t* status) {
  if (!hal::vmx::GetInterruptManager().IsValid(interruptHandle)) {
    hal::vmx::SetInterruptResult(hal::vmx::InterruptResult::kInvalidHandle,
                                 status, "Invalid VMX interrupt handle");
    return;
  }
  int32_t channel = -1;
  auto sourceResult = hal::vmx::ValidateInterruptSource(
      digitalSourceHandle, analogTriggerType, channel);
  if (sourceResult != hal::vmx::InterruptResult::kOk) {
    hal::vmx::SetInterruptResult(
        sourceResult, status,
        sourceResult == hal::vmx::InterruptResult::kUnsupportedSource
            ? "VMX Interrupt AnalogTrigger sources are not supported"
            : "VMX Interrupt source must be an input DIO handle");
    return;
  }
  hal::vmx::SetInterruptResult(
      hal::vmx::GetInterruptManager().RequestSource(
          interruptHandle, digitalSourceHandle, channel),
      status, "Failed to activate VMX Interrupt resource");
}

void HAL_SetInterruptUpSourceEdge(HAL_InterruptHandle interruptHandle,
                                  HAL_Bool risingEdge, HAL_Bool fallingEdge,
                                  int32_t* status) {
  hal::vmx::SetInterruptResult(
      hal::vmx::GetInterruptManager().SetEdges(
          interruptHandle, risingEdge != 0, fallingEdge != 0),
      status, "VMX Interrupt requires rising, falling, or both edges");
}

int64_t HAL_WaitForInterrupt(HAL_InterruptHandle interruptHandle,
                             double timeout, HAL_Bool ignorePrevious,
                             int32_t* status) {
  auto result = hal::vmx::GetInterruptManager().Wait(
      interruptHandle, timeout, ignorePrevious != 0,
      std::numeric_limits<uint64_t>::max());
  hal::vmx::SetInterruptResult(result.result, status,
                               "Failed to wait for VMX Interrupt");
  return result.result == hal::vmx::InterruptResult::kOk ? result.mask : 0;
}

int64_t HAL_WaitForMultipleInterrupts(HAL_InterruptHandle interruptHandle,
                                      int64_t mask, double timeout,
                                      HAL_Bool ignorePrevious,
                                      int32_t* status) {
  auto result = hal::vmx::GetInterruptManager().Wait(
      interruptHandle, timeout, ignorePrevious != 0,
      static_cast<uint64_t>(mask));
  hal::vmx::SetInterruptResult(result.result, status,
                               "Failed to wait for VMX Interrupt mask");
  return result.result == hal::vmx::InterruptResult::kOk ? result.mask : 0;
}

int64_t HAL_ReadInterruptRisingTimestamp(HAL_InterruptHandle interruptHandle,
                                         int32_t* status) {
  return hal::vmx::ReturnInterruptValue(
      hal::vmx::GetInterruptManager().ReadTimestamp(interruptHandle, true),
      status, "Failed to read VMX rising interrupt timestamp");
}

int64_t HAL_ReadInterruptFallingTimestamp(HAL_InterruptHandle interruptHandle,
                                          int32_t* status) {
  return hal::vmx::ReturnInterruptValue(
      hal::vmx::GetInterruptManager().ReadTimestamp(interruptHandle, false),
      status, "Failed to read VMX falling interrupt timestamp");
}

void HAL_ReleaseWaitingInterrupt(HAL_InterruptHandle interruptHandle,
                                 int32_t* status) {
  hal::vmx::SetInterruptResult(
      hal::vmx::GetInterruptManager().ReleaseWaiting(interruptHandle), status,
      "Failed to release VMX Interrupt waiters");
}

}  // extern "C"
