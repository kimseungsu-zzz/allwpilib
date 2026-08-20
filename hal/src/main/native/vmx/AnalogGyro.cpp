// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AnalogGyroInternal.h"

#include <string_view>

#include "HALInitializer.h"
#include "HALInternal.h"
#include "hal/Errors.h"

namespace hal::vmx {

AnalogGyroResult AnalogGyroManager::Initialize(
    HAL_AnalogInputHandle analogHandle, std::string_view allocationLocation,
    HAL_GyroHandle& handle) noexcept {
  handle = HAL_kInvalidHandle;
  auto input = m_inputManager ? m_inputManager->AcquirePort(analogHandle)
                              : GetAnalogInputManager().AcquirePort(analogHandle);
  if (!input) {
    return AnalogGyroResult::kInvalidHandle;
  }
  const auto channel = input->GetLogicalChannel();
  if (!IsAnalogAccumulatorChannelValid(channel)) {
    return AnalogGyroResult::kInvalidChannel;
  }

  std::scoped_lock allocationLock{m_allocationMutex};
  int32_t status = HAL_SUCCESS;
  auto gyro = m_handles.Allocate(channel, &handle, &status);
  if (status != HAL_SUCCESS || !gyro) {
    handle = HAL_kInvalidHandle;
    return status == RESOURCE_IS_ALLOCATED
               ? AnalogGyroResult::kAlreadyAllocated
               : AnalogGyroResult::kInvalidChannel;
  }
  gyro->SetWait(m_wait);
  auto result = gyro->Attach(std::move(input), allocationLocation);
  if (result != AnalogGyroResult::kOk) {
    m_handles.Free(handle);
    handle = HAL_kInvalidHandle;
  }
  return result;
}

AnalogGyroResult AnalogGyroManager::Setup(HAL_GyroHandle handle) noexcept {
  auto gyro = Get(handle);
  return gyro ? gyro->Setup() : AnalogGyroResult::kInvalidHandle;
}

AnalogGyroResult AnalogGyroManager::SetParameters(
    HAL_GyroHandle handle, double voltsPerDegreePerSecond, double offset,
    int32_t center) noexcept {
  auto gyro = Get(handle);
  return gyro ? gyro->SetParameters(voltsPerDegreePerSecond, offset, center)
              : AnalogGyroResult::kInvalidHandle;
}

AnalogGyroResult AnalogGyroManager::SetSensitivity(
    HAL_GyroHandle handle, double voltsPerDegreePerSecond) noexcept {
  auto gyro = Get(handle);
  return gyro ? gyro->SetSensitivity(voltsPerDegreePerSecond)
              : AnalogGyroResult::kInvalidHandle;
}

AnalogGyroResult AnalogGyroManager::Reset(HAL_GyroHandle handle) noexcept {
  auto gyro = Get(handle);
  return gyro ? gyro->Reset() : AnalogGyroResult::kInvalidHandle;
}

AnalogGyroResult AnalogGyroManager::Calibrate(HAL_GyroHandle handle) noexcept {
  auto gyro = Get(handle);
  return gyro ? gyro->Calibrate() : AnalogGyroResult::kInvalidHandle;
}

AnalogGyroResult AnalogGyroManager::SetDeadband(HAL_GyroHandle handle,
                                                double volts) noexcept {
  auto gyro = Get(handle);
  return gyro ? gyro->SetDeadband(volts) : AnalogGyroResult::kInvalidHandle;
}

std::pair<AnalogGyroResult, double> AnalogGyroManager::GetAngle(
    HAL_GyroHandle handle) noexcept {
  auto gyro = Get(handle);
  return gyro ? gyro->GetAngle()
              : std::pair{AnalogGyroResult::kInvalidHandle, 0.0};
}

std::pair<AnalogGyroResult, double> AnalogGyroManager::GetRate(
    HAL_GyroHandle handle) noexcept {
  auto gyro = Get(handle);
  return gyro ? gyro->GetRate()
              : std::pair{AnalogGyroResult::kInvalidHandle, 0.0};
}

std::pair<AnalogGyroResult, double> AnalogGyroManager::GetOffset(
    HAL_GyroHandle handle) noexcept {
  auto gyro = Get(handle);
  return gyro ? gyro->GetOffset()
              : std::pair{AnalogGyroResult::kInvalidHandle, 0.0};
}

std::pair<AnalogGyroResult, int32_t> AnalogGyroManager::GetCenter(
    HAL_GyroHandle handle) noexcept {
  auto gyro = Get(handle);
  return gyro ? gyro->GetCenter()
              : std::pair{AnalogGyroResult::kInvalidHandle, 0};
}

void AnalogGyroManager::Free(HAL_GyroHandle handle) noexcept {
  auto gyro = Get(handle);
  if (gyro) {
    gyro->Close();
  }
  m_handles.Free(handle);
}

AnalogGyroManager& GetAnalogGyroManager() {
  static AnalogGyroManager manager;
  return manager;
}

namespace {

void SetAnalogGyroResult(AnalogGyroResult result, int32_t* status,
                         std::string_view message) {
  if (status == nullptr) {
    return;
  }
  switch (result) {
    case AnalogGyroResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case AnalogGyroResult::kInvalidHandle:
      *status = HAL_HANDLE_ERROR;
      return;
    case AnalogGyroResult::kInvalidChannel:
      *status = HAL_INVALID_ACCUMULATOR_CHANNEL;
      hal::SetLastError(status, message);
      return;
    case AnalogGyroResult::kAlreadyAllocated:
      *status = RESOURCE_IS_ALLOCATED;
      hal::SetLastError(status, message);
      return;
    case AnalogGyroResult::kInvalidParameter:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status, message);
      return;
    case AnalogGyroResult::kZeroCount:
    case AnalogGyroResult::kHardwareFailure:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
  }
}

}  // namespace
}  // namespace hal::vmx

extern "C" {

HAL_GyroHandle HAL_InitializeAnalogGyro(HAL_AnalogInputHandle analogHandle,
                                        const char* allocationLocation,
                                        int32_t* status) {
  hal::init::CheckInit();
  HAL_GyroHandle handle = HAL_kInvalidHandle;
  const auto result = hal::vmx::GetAnalogGyroManager().Initialize(
      analogHandle, allocationLocation ? allocationLocation : "", handle);
  hal::vmx::SetAnalogGyroResult(
      result, status, "VMX AnalogGyro requires a valid accumulator channel");
  return result == hal::vmx::AnalogGyroResult::kOk ? handle
                                                   : HAL_kInvalidHandle;
}

void HAL_SetupAnalogGyro(HAL_GyroHandle handle, int32_t* status) {
  hal::vmx::SetAnalogGyroResult(
      hal::vmx::GetAnalogGyroManager().Setup(handle), status,
      "Failed to configure VMX AnalogGyro fixed-rate accumulator");
}

void HAL_FreeAnalogGyro(HAL_GyroHandle handle) {
  hal::vmx::GetAnalogGyroManager().Free(handle);
}

void HAL_SetAnalogGyroParameters(HAL_GyroHandle handle,
                                 double voltsPerDegreePerSecond, double offset,
                                 int32_t center, int32_t* status) {
  hal::vmx::SetAnalogGyroResult(
      hal::vmx::GetAnalogGyroManager().SetParameters(
          handle, voltsPerDegreePerSecond, offset, center),
      status, "Invalid VMX AnalogGyro parameters or center update failed");
}

void HAL_SetAnalogGyroVoltsPerDegreePerSecond(
    HAL_GyroHandle handle, double voltsPerDegreePerSecond, int32_t* status) {
  hal::vmx::SetAnalogGyroResult(
      hal::vmx::GetAnalogGyroManager().SetSensitivity(
          handle, voltsPerDegreePerSecond),
      status, "Invalid VMX AnalogGyro sensitivity");
}

void HAL_ResetAnalogGyro(HAL_GyroHandle handle, int32_t* status) {
  hal::vmx::SetAnalogGyroResult(
      hal::vmx::GetAnalogGyroManager().Reset(handle), status,
      "Failed to reset VMX AnalogGyro accumulator");
}

void HAL_CalibrateAnalogGyro(HAL_GyroHandle handle, int32_t* status) {
  hal::vmx::SetAnalogGyroResult(
      hal::vmx::GetAnalogGyroManager().Calibrate(handle), status,
      "Failed to calibrate VMX AnalogGyro (no samples or hardware failure)");
}

void HAL_SetAnalogGyroDeadband(HAL_GyroHandle handle, double volts,
                               int32_t* status) {
  hal::vmx::SetAnalogGyroResult(
      hal::vmx::GetAnalogGyroManager().SetDeadband(handle, volts), status,
      "Invalid VMX AnalogGyro deadband or accumulator update failed");
}

double HAL_GetAnalogGyroAngle(HAL_GyroHandle handle, int32_t* status) {
  auto [result, angle] = hal::vmx::GetAnalogGyroManager().GetAngle(handle);
  hal::vmx::SetAnalogGyroResult(result, status,
                                "Failed to read VMX AnalogGyro angle");
  return result == hal::vmx::AnalogGyroResult::kOk ? angle : 0.0;
}

double HAL_GetAnalogGyroRate(HAL_GyroHandle handle, int32_t* status) {
  auto [result, rate] = hal::vmx::GetAnalogGyroManager().GetRate(handle);
  hal::vmx::SetAnalogGyroResult(result, status,
                                "Failed to read VMX AnalogGyro rate");
  return result == hal::vmx::AnalogGyroResult::kOk ? rate : 0.0;
}

double HAL_GetAnalogGyroOffset(HAL_GyroHandle handle, int32_t* status) {
  auto [result, offset] = hal::vmx::GetAnalogGyroManager().GetOffset(handle);
  hal::vmx::SetAnalogGyroResult(result, status,
                                "Failed to read VMX AnalogGyro offset");
  return result == hal::vmx::AnalogGyroResult::kOk ? offset : 0.0;
}

int32_t HAL_GetAnalogGyroCenter(HAL_GyroHandle handle, int32_t* status) {
  auto [result, center] = hal::vmx::GetAnalogGyroManager().GetCenter(handle);
  hal::vmx::SetAnalogGyroResult(result, status,
                                "Failed to read VMX AnalogGyro center");
  return result == hal::vmx::AnalogGyroResult::kOk ? center : 0;
}

}  // extern "C"
