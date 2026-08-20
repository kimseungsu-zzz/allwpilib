// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/Encoder.h"

#include <memory>
#include <string_view>

#include "VMXPi.h"

#include "DIOInternal.h"
#include "EncoderInternal.h"
#include "HALInitializer.h"
#include "HALInternal.h"
#include "VMXDigitalSource.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"
#include "hal/handles/HandlesInternal.h"

namespace hal::vmx {
namespace {

class DriverEncoderBackend final : public EncoderBackend {
 public:
  DriverEncoderBackend(int32_t channelA, int32_t channelB,
                       VMXEncoderEdge edge, std::shared_ptr<VMXPi> context)
      : m_context{std::move(context)} {
    if (!m_context || !m_context->IsOpen()) {
      return;
    }
    EncoderConfig::EncoderEdge hardwareEdge;
    switch (edge) {
      case VMXEncoderEdge::k1X:
        hardwareEdge = EncoderConfig::EncoderEdge::x1;
        break;
      case VMXEncoderEdge::k2X:
        hardwareEdge = EncoderConfig::EncoderEdge::x2;
        break;
      default:
        hardwareEdge = EncoderConfig::EncoderEdge::x4;
        break;
    }
    EncoderConfig config{hardwareEdge};
    VMXChannelInfo channels[2] = {
        VMXChannelInfo(channelA, VMXChannelCapability::EncoderAInput),
        VMXChannelInfo(channelB, VMXChannelCapability::EncoderBInput)};
    VMXErrorCode error;
    m_initialized = m_context->io.ActivateDualchannelResource(
        channels[0], channels[1], &config, m_resourceHandle, &error);
  }

  ~DriverEncoderBackend() override {
    if (m_initialized) {
      VMXErrorCode error;
      m_context->io.DeallocateResource(m_resourceHandle, &error);
    }
  }

  bool IsInitialized() const noexcept { return m_initialized; }

  bool GetCount(int32_t& count) noexcept override {
    count = 0;
    if (!m_initialized) {
      return false;
    }
    VMXErrorCode error;
    return m_context->io.Encoder_GetCount(m_resourceHandle, count, &error);
  }

  bool GetDirection(bool& forward) noexcept override {
    forward = false;
    if (!m_initialized) {
      return false;
    }
    VMXIO::EncoderDirection direction;
    VMXErrorCode error;
    if (!m_context->io.Encoder_GetDirection(m_resourceHandle, direction,
                                             &error)) {
      return false;
    }
    forward = direction == VMXIO::EncoderForward;
    return true;
  }

  bool Reset() noexcept override {
    if (!m_initialized) {
      return false;
    }
    VMXErrorCode error;
    return m_context->io.Encoder_Reset(m_resourceHandle, &error);
  }

  bool GetPeriodMicroseconds(uint16_t& period) noexcept override {
    period = 0;
    if (!m_initialized) {
      return false;
    }
    VMXErrorCode error;
    return m_context->io.Encoder_GetLastPulsePeriodMicroseconds(
        m_resourceHandle, period, &error);
  }

 private:
  std::shared_ptr<VMXPi> m_context;
  VMXResourceHandle m_resourceHandle = 0;
  bool m_initialized = false;
};

std::unique_ptr<EncoderBackend> CreateEncoderBackend(int32_t channelA,
                                                     int32_t channelB,
                                                     VMXEncoderEdge edge) {
  auto context = GetRuntimeContext();
  if (!context) {
    return nullptr;
  }
  auto backend = std::make_unique<DriverEncoderBackend>(
      channelA, channelB, edge, std::move(context));
  return backend->IsInitialized() ? std::move(backend) : nullptr;
}

EncoderSourceClaim ClaimEncoderSources(HAL_Handle sourceA,
                                       HAL_Handle sourceB) {
  auto sourceAResult = DecodeVMXDigitalSource(sourceA, HAL_Trigger_kInWindow);
  auto sourceBResult = DecodeVMXDigitalSource(sourceB, HAL_Trigger_kInWindow);
  if (sourceAResult == VMXDigitalSourceResult::kUnsupportedAnalogTrigger ||
      sourceBResult == VMXDigitalSourceResult::kUnsupportedAnalogTrigger) {
    return {EncoderResult::kUnsupportedSource, -1, -1};
  }
  if (sourceAResult != VMXDigitalSourceResult::kOk ||
      sourceBResult != VMXDigitalSourceResult::kOk) {
    return {EncoderResult::kInvalidSource, -1, -1};
  }
  auto claim = GetDIOManager().ClaimEncoderSources(sourceA, sourceB,
                                                   "VMX Encoder source");
  switch (claim.result) {
    case DIOResult::kOk:
      return {EncoderResult::kOk, claim.channelA, claim.channelB};
    case DIOResult::kAlreadyAllocated:
      return {EncoderResult::kAlreadyAllocated, claim.channelA,
              claim.channelB};
    case DIOResult::kInvalidHandle:
      return {EncoderResult::kInvalidSource, claim.channelA, claim.channelB};
    default:
      return {EncoderResult::kHardwareFailure, claim.channelA, claim.channelB};
  }
}

void ReleaseEncoderSources(HAL_Handle sourceA, HAL_Handle sourceB,
                           int32_t channelA, int32_t channelB) {
  GetDIOManager().ReleaseEncoderSources(sourceA, sourceB, channelA, channelB);
}

EncoderManager& GetEncoderManager() {
  static EncoderManager manager{CreateEncoderBackend, ClaimEncoderSources,
                                ReleaseEncoderSources};
  return manager;
}

void SetEncoderResult(EncoderResult result, int32_t* status,
                      std::string_view hardwareMessage) {
  switch (result) {
    case EncoderResult::kOk:
      *status = HAL_SUCCESS;
      break;
    case EncoderResult::kInvalidHandle:
    case EncoderResult::kInvalidSource:
      *status = HAL_HANDLE_ERROR;
      break;
    case EncoderResult::kUnsupportedSource:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(
          status, "VMX Encoder does not yet support AnalogTrigger sources");
      break;
    case EncoderResult::kInvalidEncoding:
    case EncoderResult::kOutOfRange:
      *status = PARAMETER_OUT_OF_RANGE;
      break;
    case EncoderResult::kAlreadyAllocated:
      *status = RESOURCE_IS_ALLOCATED;
      break;
    case EncoderResult::kNoResources:
      *status = NO_AVAILABLE_RESOURCES;
      break;
    case EncoderResult::kUnsupported:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, hardwareMessage);
      break;
    default:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, hardwareMessage);
      break;
  }
}

template <typename T>
T ReturnEncoderValue(std::pair<EncoderResult, T> result, int32_t* status,
                     std::string_view message, T fallback = T{}) {
  SetEncoderResult(result.first, status, message);
  return result.first == EncoderResult::kOk ? result.second : fallback;
}

}  // namespace
}  // namespace hal::vmx

extern "C" {

HAL_EncoderHandle HAL_InitializeEncoder(
    HAL_Handle digitalSourceHandleA, HAL_AnalogTriggerType analogTriggerTypeA,
    HAL_Handle digitalSourceHandleB, HAL_AnalogTriggerType analogTriggerTypeB,
    HAL_Bool reverseDirection, HAL_EncoderEncodingType encodingType,
    int32_t* status) {
  hal::init::CheckInit();
  static_cast<void>(analogTriggerTypeA);
  static_cast<void>(analogTriggerTypeB);
  if (!hal::vmx::IsRuntimeInitialized()) {
    *status = INCOMPATIBLE_STATE;
    hal::SetLastError(status, "VMX HAL runtime is not initialized");
    return HAL_kInvalidHandle;
  }
  auto allocation = hal::vmx::GetEncoderManager().Allocate(
      digitalSourceHandleA, digitalSourceHandleB, reverseDirection != 0,
      encodingType);
  hal::vmx::SetEncoderResult(
      allocation.result, status,
      "Failed to activate the VMX Encoder dual-channel resource");
  return allocation.result == hal::vmx::EncoderResult::kOk
             ? allocation.handle
             : HAL_kInvalidHandle;
}

void HAL_FreeEncoder(HAL_EncoderHandle encoderHandle) {
  hal::vmx::GetEncoderManager().Free(encoderHandle);
}

void HAL_SetEncoderSimDevice(HAL_EncoderHandle handle,
                             HAL_SimDeviceHandle device) {
  static_cast<void>(handle);
  static_cast<void>(device);
}

int32_t HAL_GetEncoder(HAL_EncoderHandle handle, int32_t* status) {
  return hal::vmx::ReturnEncoderValue(
      hal::vmx::GetEncoderManager().Get(handle), status,
      "Failed to read VMX Encoder count");
}

int32_t HAL_GetEncoderRaw(HAL_EncoderHandle handle, int32_t* status) {
  return hal::vmx::ReturnEncoderValue(
      hal::vmx::GetEncoderManager().GetRaw(handle), status,
      "Failed to read raw VMX Encoder count");
}

int32_t HAL_GetEncoderEncodingScale(HAL_EncoderHandle handle,
                                    int32_t* status) {
  return hal::vmx::ReturnEncoderValue(
      hal::vmx::GetEncoderManager().GetEncodingScale(handle), status,
      "Failed to read VMX Encoder encoding scale");
}

void HAL_ResetEncoder(HAL_EncoderHandle handle, int32_t* status) {
  hal::vmx::SetEncoderResult(hal::vmx::GetEncoderManager().Reset(handle), status,
                             "Failed to reset VMX Encoder");
}

double HAL_GetEncoderPeriod(HAL_EncoderHandle handle, int32_t* status) {
  return hal::vmx::ReturnEncoderValue(
      hal::vmx::GetEncoderManager().GetPeriod(handle), status,
      "Failed to read VMX Encoder period");
}

void HAL_SetEncoderMaxPeriod(HAL_EncoderHandle handle, double maxPeriod,
                             int32_t* status) {
  hal::vmx::SetEncoderResult(
      hal::vmx::GetEncoderManager().SetMaxPeriod(handle, maxPeriod), status,
      "Failed to set VMX Encoder max period");
}

HAL_Bool HAL_GetEncoderStopped(HAL_EncoderHandle handle, int32_t* status) {
  return hal::vmx::ReturnEncoderValue(
      hal::vmx::GetEncoderManager().GetStopped(handle), status,
      "Failed to determine VMX Encoder stopped state");
}

HAL_Bool HAL_GetEncoderDirection(HAL_EncoderHandle handle, int32_t* status) {
  return hal::vmx::ReturnEncoderValue(
      hal::vmx::GetEncoderManager().GetDirection(handle), status,
      "Failed to read VMX Encoder direction");
}

double HAL_GetEncoderDistance(HAL_EncoderHandle handle, int32_t* status) {
  return hal::vmx::ReturnEncoderValue(
      hal::vmx::GetEncoderManager().GetDistance(handle), status,
      "Failed to read VMX Encoder distance");
}

double HAL_GetEncoderRate(HAL_EncoderHandle handle, int32_t* status) {
  return hal::vmx::ReturnEncoderValue(
      hal::vmx::GetEncoderManager().GetRate(handle), status,
      "Failed to read VMX Encoder rate");
}

void HAL_SetEncoderMinRate(HAL_EncoderHandle handle, double minRate,
                           int32_t* status) {
  hal::vmx::SetEncoderResult(
      hal::vmx::GetEncoderManager().SetMinRate(handle, minRate), status,
      "Failed to set VMX Encoder minimum rate");
}

void HAL_SetEncoderDistancePerPulse(HAL_EncoderHandle handle,
                                    double distancePerPulse, int32_t* status) {
  hal::vmx::SetEncoderResult(
      hal::vmx::GetEncoderManager().SetDistancePerPulse(handle,
                                                        distancePerPulse),
      status, "Failed to set VMX Encoder distance per pulse");
}

double HAL_GetEncoderDistancePerPulse(HAL_EncoderHandle handle,
                                      int32_t* status) {
  return hal::vmx::ReturnEncoderValue(
      hal::vmx::GetEncoderManager().GetDistancePerPulse(handle), status,
      "Failed to read VMX Encoder distance per pulse");
}

void HAL_SetEncoderReverseDirection(HAL_EncoderHandle handle,
                                    HAL_Bool reverseDirection,
                                    int32_t* status) {
  hal::vmx::SetEncoderResult(
      hal::vmx::GetEncoderManager().SetReverseDirection(
          handle, reverseDirection != 0),
      status, "Failed to set VMX Encoder reverse direction");
}

void HAL_SetEncoderSamplesToAverage(HAL_EncoderHandle handle,
                                    int32_t samplesToAverage,
                                    int32_t* status) {
  hal::vmx::SetEncoderResult(
      hal::vmx::GetEncoderManager().SetSamplesToAverage(handle,
                                                        samplesToAverage),
      status,
      "VMX Encoder hardware has a fixed approximately 50 us period averaging "
      "window and no configurable sample-depth API");
}

int32_t HAL_GetEncoderSamplesToAverage(HAL_EncoderHandle handle,
                                       int32_t* status) {
  return hal::vmx::ReturnEncoderValue(
      hal::vmx::GetEncoderManager().GetSamplesToAverage(handle), status,
      "VMX Encoder does not expose a configurable samples-to-average value");
}

double HAL_GetEncoderDecodingScaleFactor(HAL_EncoderHandle handle,
                                         int32_t* status) {
  return hal::vmx::ReturnEncoderValue(
      hal::vmx::GetEncoderManager().GetDecodingScale(handle), status,
      "Failed to read VMX Encoder decoding scale");
}

HAL_EncoderEncodingType HAL_GetEncoderEncodingType(HAL_EncoderHandle handle,
                                                   int32_t* status) {
  return hal::vmx::ReturnEncoderValue(
      hal::vmx::GetEncoderManager().GetEncodingType(handle), status,
      "Failed to read VMX Encoder encoding type", HAL_Encoder_k4X);
}

void HAL_SetEncoderIndexSource(HAL_EncoderHandle encoderHandle,
                               HAL_Handle digitalSourceHandle,
                               HAL_AnalogTriggerType analogTriggerType,
                               HAL_EncoderIndexingType type, int32_t* status) {
  static_cast<void>(digitalSourceHandle);
  static_cast<void>(analogTriggerType);
  static_cast<void>(type);
  auto validation =
      hal::vmx::GetEncoderManager().GetEncodingScale(encoderHandle);
  if (validation.first == hal::vmx::EncoderResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(
      status,
      "VMX Encoder index source is deferred until Interrupt HAL support");
}

int32_t HAL_GetEncoderFPGAIndex(HAL_EncoderHandle encoderHandle,
                                int32_t* status) {
  auto validation =
      hal::vmx::GetEncoderManager().GetEncodingScale(encoderHandle);
  if (validation.first == hal::vmx::EncoderResult::kInvalidHandle) {
    *status = HAL_HANDLE_ERROR;
    return -1;
  }
  *status = INCOMPATIBLE_STATE;
  hal::SetLastError(status, "VMX Encoder has no FPGA index");
  return -1;
}

}  // extern "C"
