// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/Counter.h"

#include <memory>
#include <chrono>
#include <string_view>

#include "CounterInternal.h"
#include "DIOInternal.h"
#include "VMXChannelCapabilities.h"
#include "HALInitializer.h"
#include "HALInternal.h"
#include "VMXDigitalSource.h"
#include "VMXPi.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"

namespace hal::vmx {
namespace {

class DriverCounterBackend final : public CounterBackend {
 public:
  DriverCounterBackend(int32_t channelUp, int32_t channelDown,
                       bool upRising, bool upFalling, bool downRising,
                       bool downFalling, std::shared_ptr<VMXPi> context)
      : m_context{std::move(context)} {
    if (!m_context || !m_context->IsOpen()) {
      return;
    }
    InputCaptureConfig config;
    config.SetCounterClockSource(InputCaptureConfig::INTERNAL);
    config.SetCounterDirection(InputCaptureConfig::DIRECTION_UP);
    config.SetCaptureChannelSource(InputCaptureConfig::CH1,
                                   InputCaptureConfig::CAPTURE_SIGNAL_A);
    config.SetCaptureChannelActiveEdge(
        InputCaptureConfig::CH1, ToActiveEdge(upRising, upFalling));
    config.SetCaptureChannelActiveEdge(
        InputCaptureConfig::CH2, ToActiveEdge(downRising, downFalling));
    config.SetVirtualCounterMode(
        channelUp == channelDown
            ? InputCaptureConfig::VC_MODE_DISABLED
            : InputCaptureConfig::VC_MODE_DUAL_INPUT_UPDOWN);
    VMXErrorCode error;
    if (channelUp == channelDown) {
      m_singleChannel = true;
      m_initialized = m_context->io.ActivateSinglechannelResource(
          ::VMXChannelInfo(channelUp,
                           VMXChannelCapability::InputCaptureInput),
          &config, m_resourceHandle, &error);
    } else {
      config.SetCaptureChannelSource(InputCaptureConfig::CH2,
                                     InputCaptureConfig::CAPTURE_SIGNAL_B);
      m_initialized = m_context->io.ActivateDualchannelResource(
          ::VMXChannelInfo(channelUp,
                           VMXChannelCapability::InputCaptureInput),
          ::VMXChannelInfo(channelDown,
                           VMXChannelCapability::InputCaptureInput2),
          &config, m_resourceHandle, &error);
    }
  }

  ~DriverCounterBackend() override {
    if (m_initialized) {
      VMXErrorCode error;
      m_context->io.DeallocateResource(m_resourceHandle, &error);
    }
  }

  bool GetChannelCounts(uint32_t& channel1,
                        uint32_t& channel2) noexcept override {
    if (!m_initialized) {
      return false;
    }
    VMXErrorCode error;
    if (m_singleChannel) {
      int32_t count = 0;
      auto ok = m_context->io.InputCapture_GetCount(m_resourceHandle, count,
                                                    &error);
      channel1 = ok && count >= 0 ? static_cast<uint32_t>(count) : 0;
      channel2 = 0;
      return ok;
    }
    return m_context->io.InputCapture_GetChannelCounts(
        m_resourceHandle, channel1, channel2, &error);
  }

  bool GetInputStatus(bool& forward, bool& active) noexcept override {
    if (!m_initialized) {
      return false;
    }
    VMXErrorCode error;
    return m_context->io.InputCapture_InputStatus(m_resourceHandle, forward,
                                                  active, &error);
  }

  bool Reset() noexcept override {
    if (!m_initialized) {
      return false;
    }
    VMXErrorCode error;
    return m_context->io.InputCapture_Reset(m_resourceHandle, &error);
  }

  bool IsInitialized() const noexcept { return m_initialized; }

 private:
  static InputCaptureConfig::CaptureChannelActiveEdge ToActiveEdge(
      bool rising, bool falling) noexcept {
    if (rising && falling) {
      return InputCaptureConfig::ACTIVE_BOTH;
    }
    if (falling) {
      return InputCaptureConfig::ACTIVE_FALLING;
    }
    return InputCaptureConfig::ACTIVE_RISING;
  }

  std::shared_ptr<VMXPi> m_context;
  VMXResourceHandle m_resourceHandle = 0;
  bool m_initialized = false;
  bool m_singleChannel = false;
};

std::unique_ptr<CounterBackend> CreateCounterBackend(
    int32_t channelUp, int32_t channelDown, bool upRising, bool upFalling,
    bool downRising, bool downFalling) {
  auto context = GetRuntimeContext();
  if (!context) {
    return nullptr;
  }
  auto backend = std::make_unique<DriverCounterBackend>(
      channelUp, channelDown, upRising, upFalling, downRising, downFalling,
      std::move(context));
  return backend->IsInitialized() ? std::move(backend) : nullptr;
}

CounterSourceClaim ClaimCounterSources(HAL_Handle sourceUp,
                                       HAL_Handle sourceDown) {
  DIOResourceClaimResult claim;
  if (sourceUp == sourceDown) {
    claim = GetDIOManager().ClaimResourceSource(
        sourceUp, DigitalChannelOwner::kCounter,
        "VMX Counter InputCapture source");
    claim.channelB = claim.channelA;
  } else {
    claim = GetDIOManager().ClaimResourceSources(
        sourceUp, sourceDown, DigitalChannelOwner::kCounter,
        "VMX Counter InputCapture source");
  }
  if (claim.result == DIOResult::kOk &&
      sourceUp != sourceDown &&
      !IsVMXCounterPair(claim.channelA, claim.channelB)) {
    GetDIOManager().ReleaseResourceSources(
        sourceUp, sourceDown, claim.channelA, claim.channelB,
        DigitalChannelOwner::kCounter);
    return {CounterResult::kUnsupportedSource, claim.channelA,
            claim.channelB};
  }
  switch (claim.result) {
    case DIOResult::kOk:
      return {CounterResult::kOk, claim.channelA, claim.channelB};
    case DIOResult::kAlreadyAllocated:
      return {CounterResult::kAlreadyAllocated, claim.channelA,
              claim.channelB};
    case DIOResult::kInvalidHandle:
      return {CounterResult::kInvalidSource, claim.channelA,
              claim.channelB};
    default:
      return {CounterResult::kHardwareFailure, claim.channelA,
              claim.channelB};
  }
}

void ReleaseCounterSources(HAL_Handle sourceUp, HAL_Handle sourceDown,
                           int32_t channelUp, int32_t channelDown) {
  GetDIOManager().ReleaseResourceSources(
      sourceUp, sourceDown, channelUp, channelDown,
      DigitalChannelOwner::kCounter);
}

CounterManager& GetCounterManager() {
  static CounterManager manager{CreateCounterBackend, ClaimCounterSources,
                                ReleaseCounterSources,
                                std::chrono::steady_clock::now,
                                [](HAL_Counter_Mode mode, int32_t channelUp,
                                   int32_t channelDown, bool upRising,
                                   bool upFalling, bool downRising,
                                   bool downFalling) {
                                  // The VMX InputCapture adapter uses the
                                  // same hardware primitive for all supported
                                  // counter modes; the mode controls source
                                  // validation and single/dual routing.
                                  static_cast<void>(mode);
                                  return CreateCounterBackend(
                                      channelUp, channelDown, upRising,
                                      upFalling, downRising, downFalling);
                                }};
  return manager;
}

void SetCounterResult(CounterResult result, int32_t* status,
                      std::string_view message) {
  switch (result) {
    case CounterResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case CounterResult::kInvalidHandle:
    case CounterResult::kInvalidSource:
      *status = HAL_HANDLE_ERROR;
      return;
    case CounterResult::kUnsupportedSource:
    case CounterResult::kUnsupportedMode:
    case CounterResult::kUnsupported:
    case CounterResult::kUnconfigured:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
    case CounterResult::kOutOfRange:
      *status = PARAMETER_OUT_OF_RANGE;
      return;
    case CounterResult::kAlreadyAllocated:
      *status = RESOURCE_IS_ALLOCATED;
      return;
    case CounterResult::kNoResources:
      *status = NO_AVAILABLE_RESOURCES;
      return;
    default:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
  }
}

template <typename T>
T ReturnCounterValue(std::pair<CounterResult, T> result, int32_t* status,
                     std::string_view message, T fallback = T{}) {
  SetCounterResult(result.first, status, message);
  return result.first == CounterResult::kOk ? result.second : fallback;
}

CounterResult ValidateCounterSource(HAL_Handle sourceHandle,
                                    HAL_AnalogTriggerType analogTriggerType,
                                    int32_t& channel) {
  auto decoded = DecodeVMXDigitalSource(sourceHandle, analogTriggerType);
  if (decoded == VMXDigitalSourceResult::kUnsupportedAnalogTrigger) {
    return CounterResult::kUnsupportedSource;
  }
  if (decoded != VMXDigitalSourceResult::kOk) {
    return CounterResult::kInvalidSource;
  }
  auto validation = GetDIOManager().ValidateInputSource(sourceHandle);
  channel = validation.second;
  switch (validation.first) {
    case DIOResult::kOk:
      return CounterResult::kOk;
    case DIOResult::kAlreadyAllocated:
      return CounterResult::kAlreadyAllocated;
    case DIOResult::kOutputChannel:
      return CounterResult::kInvalidSource;
    case DIOResult::kInvalidHandle:
      return CounterResult::kInvalidSource;
    default:
      return CounterResult::kHardwareFailure;
  }
}

}  // namespace
}  // namespace hal::vmx

extern "C" {

HAL_CounterHandle HAL_InitializeCounter(HAL_Counter_Mode mode, int32_t* index,
                                        int32_t* status) {
  hal::init::CheckInit();
  if (!hal::vmx::IsRuntimeInitialized()) {
    *status = INCOMPATIBLE_STATE;
    hal::SetLastError(status, "VMX HAL runtime is not initialized");
    return HAL_kInvalidHandle;
  }
  auto allocation = hal::vmx::GetCounterManager().Allocate(mode, index);
  hal::vmx::SetCounterResult(
      allocation.result, status,
      "VMX Counter mode is not supported by the InputCapture adapter");
  return allocation.result == hal::vmx::CounterResult::kOk
             ? allocation.handle
             : HAL_kInvalidHandle;
}

void HAL_FreeCounter(HAL_CounterHandle counterHandle) {
  hal::vmx::GetCounterManager().Free(counterHandle);
}

void HAL_SetCounterAverageSize(HAL_CounterHandle counterHandle, int32_t size,
                               int32_t* status) {
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetAverageSize(counterHandle, size),
      status, "VMX Counter average size is not supported");
}

void HAL_SetCounterUpSource(HAL_CounterHandle counterHandle,
                            HAL_Handle digitalSourceHandle,
                            HAL_AnalogTriggerType analogTriggerType,
                            int32_t* status) {
  int32_t channel = -1;
  auto sourceResult = hal::vmx::ValidateCounterSource(
      digitalSourceHandle, analogTriggerType, channel);
  if (sourceResult != hal::vmx::CounterResult::kOk) {
    hal::vmx::SetCounterResult(
        sourceResult, status,
        sourceResult == hal::vmx::CounterResult::kUnsupportedSource
            ? "VMX Counter AnalogTrigger sources are not supported"
            : "VMX Counter up source must be an input DIO handle");
    return;
  }
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetUpSource(counterHandle,
                                                digitalSourceHandle, channel),
      status, "Failed to configure VMX Counter up source");
}

void HAL_SetCounterUpSourceEdge(HAL_CounterHandle counterHandle,
                                HAL_Bool risingEdge, HAL_Bool fallingEdge,
                                int32_t* status) {
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetUpSourceEdge(
          counterHandle, risingEdge != 0, fallingEdge != 0),
      status, "Failed to configure VMX Counter up edge");
}

void HAL_ClearCounterUpSource(HAL_CounterHandle counterHandle,
                              int32_t* status) {
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().ClearUpSource(counterHandle), status,
      "Failed to clear VMX Counter up source");
}

void HAL_SetCounterDownSource(HAL_CounterHandle counterHandle,
                              HAL_Handle digitalSourceHandle,
                              HAL_AnalogTriggerType analogTriggerType,
                              int32_t* status) {
  int32_t channel = -1;
  auto sourceResult = hal::vmx::ValidateCounterSource(
      digitalSourceHandle, analogTriggerType, channel);
  if (sourceResult != hal::vmx::CounterResult::kOk) {
    hal::vmx::SetCounterResult(
        sourceResult, status,
        sourceResult == hal::vmx::CounterResult::kUnsupportedSource
            ? "VMX Counter AnalogTrigger sources are not supported"
            : "VMX Counter down source must be an input DIO handle");
    return;
  }
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetDownSource(counterHandle,
                                                  digitalSourceHandle, channel),
      status, "Failed to configure VMX Counter down source");
}

void HAL_SetCounterDownSourceEdge(HAL_CounterHandle counterHandle,
                                  HAL_Bool risingEdge, HAL_Bool fallingEdge,
                                  int32_t* status) {
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetDownSourceEdge(
          counterHandle, risingEdge != 0, fallingEdge != 0),
      status, "Failed to configure VMX Counter down edge");
}

void HAL_ClearCounterDownSource(HAL_CounterHandle counterHandle,
                                int32_t* status) {
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().ClearDownSource(counterHandle), status,
      "Failed to clear VMX Counter down source");
}

void HAL_SetCounterUpDownMode(HAL_CounterHandle counterHandle,
                              int32_t* status) {
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetTwoPulseMode(counterHandle),
      status, "VMX Counter TwoPulse mode is the active mode");
}

void HAL_SetCounterExternalDirectionMode(HAL_CounterHandle counterHandle,
                                         int32_t* status) {
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetExternalDirectionMode(counterHandle),
      status, "VMX Counter ExternalDirection requires a supported channel pair");
}

void HAL_SetCounterSemiPeriodMode(HAL_CounterHandle counterHandle,
                                  HAL_Bool highSemiPeriod, int32_t* status) {
  static_cast<void>(highSemiPeriod);
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetSemiPeriodMode(counterHandle,
                                                      highSemiPeriod != 0),
      status, "Failed to activate VMX Counter SemiPeriod input capture");
}

void HAL_SetCounterPulseLengthMode(HAL_CounterHandle counterHandle,
                                   double threshold, int32_t* status) {
  static_cast<void>(threshold);
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetUnsupportedMode(counterHandle), status,
      "VMX Counter PulseLength mode is deferred to the timing/Interrupt milestone");
}

int32_t HAL_GetCounterSamplesToAverage(HAL_CounterHandle counterHandle,
                                       int32_t* status) {
  return hal::vmx::ReturnCounterValue(
      hal::vmx::GetCounterManager().GetSamplesToAverage(counterHandle), status,
      "VMX Counter samples-to-average is not supported");
}

void HAL_SetCounterSamplesToAverage(HAL_CounterHandle counterHandle,
                                    int32_t samplesToAverage, int32_t* status) {
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetSamplesToAverage(
          counterHandle, samplesToAverage),
      status, "VMX Counter samples-to-average is not supported");
}

void HAL_ResetCounter(HAL_CounterHandle counterHandle, int32_t* status) {
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().Reset(counterHandle), status,
      "Failed to reset VMX Counter InputCapture resource");
}

int32_t HAL_GetCounter(HAL_CounterHandle counterHandle, int32_t* status) {
  return hal::vmx::ReturnCounterValue(
      hal::vmx::GetCounterManager().Get(counterHandle), status,
      "Failed to read VMX Counter InputCapture count");
}

double HAL_GetCounterPeriod(HAL_CounterHandle counterHandle, int32_t* status) {
  return hal::vmx::ReturnCounterValue(
      hal::vmx::GetCounterManager().GetPeriod(counterHandle), status,
      "Failed to read VMX Counter period");
}

void HAL_SetCounterMaxPeriod(HAL_CounterHandle counterHandle, double maxPeriod,
                             int32_t* status) {
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetMaxPeriod(counterHandle, maxPeriod),
      status, "Failed to set VMX Counter maximum period");
}

void HAL_SetCounterUpdateWhenEmpty(HAL_CounterHandle counterHandle,
                                   HAL_Bool enabled, int32_t* status) {
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetUpdateWhenEmpty(counterHandle,
                                                       enabled != 0),
      status, "VMX Counter update-when-empty is not supported");
}

HAL_Bool HAL_GetCounterStopped(HAL_CounterHandle counterHandle,
                               int32_t* status) {
  return hal::vmx::ReturnCounterValue(
      hal::vmx::GetCounterManager().GetStopped(counterHandle), status,
      "Failed to read VMX Counter stopped state");
}

HAL_Bool HAL_GetCounterDirection(HAL_CounterHandle counterHandle,
                                 int32_t* status) {
  return hal::vmx::ReturnCounterValue(
      hal::vmx::GetCounterManager().GetDirection(counterHandle), status,
      "Failed to read VMX Counter direction");
}

void HAL_SetCounterReverseDirection(HAL_CounterHandle counterHandle,
                                    HAL_Bool reverseDirection,
                                    int32_t* status) {
  hal::vmx::SetCounterResult(
      hal::vmx::GetCounterManager().SetReverseDirection(
          counterHandle, reverseDirection != 0),
      status, "Failed to set VMX Counter reverse direction");
}

}  // extern "C"
