// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/Notifier.h"

#include <thread>

#include "HALInitializer.h"
#include "HALInternal.h"
#include "NotifierInternal.h"
#include "VMXPi.h"
#include "VMXTimeInternal.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"
#include "hal/Threads.h"

namespace hal::vmx {
namespace {

bool SetWorkerPriority(std::thread::native_handle_type nativeHandle,
                       bool realTime, int32_t priority,
                       int32_t& priorityStatus) noexcept {
  auto nativeCopy = nativeHandle;
  priorityStatus = HAL_SUCCESS;
  return HAL_SetThreadPriority(&nativeCopy, realTime, priority,
                               &priorityStatus) != 0;
}

void OnVMXNotifierTimer(void* parameter, uint64_t timestamp) noexcept {
  auto* scheduler = static_cast<NotifierScheduler*>(parameter);
  if (scheduler) {
    scheduler->HardwareWake(timestamp);
  }
}

bool ArmVMXNotifierTimer(uint64_t triggerTime, void* parameter) noexcept {
  auto context = GetRuntimeContext();
  if (!context) {
    return false;
  }
  try {
    return context->getTime().RegisterTimerNotificationAbsolute(
        &OnVMXNotifierTimer, triggerTime, parameter);
  } catch (...) {
    return false;
  }
}

void DisarmVMXNotifierTimer() noexcept {
  auto context = GetRuntimeContext();
  if (!context) {
    return;
  }
  try {
    context->getTime().DeregisterTimerNotification(&OnVMXNotifierTimer);
  } catch (...) {
  }
}

NotifierManager& CreateNotifierManager() {
  static NotifierManager manager{
      [] { return GetTimeMicroseconds(nullptr); }, SetWorkerPriority,
      ArmVMXNotifierTimer, DisarmVMXNotifierTimer};
  return manager;
}

void SetNotifierResult(NotifierResult result, int32_t* status,
                       std::string_view message) {
  switch (result) {
    case NotifierResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case NotifierResult::kInvalidHandle:
      *status = HAL_HANDLE_ERROR;
      return;
    case NotifierResult::kNoResources:
      *status = NO_AVAILABLE_RESOURCES;
      return;
    case NotifierResult::kStopped:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, "VMX Notifier is stopped");
      return;
    case NotifierResult::kPriorityRange:
      *status = HAL_THREAD_PRIORITY_RANGE_ERROR;
      return;
    case NotifierResult::kPriorityFailure:
      *status = HAL_THREAD_PRIORITY_ERROR;
      hal::SetLastError(status, message);
      return;
    case NotifierResult::kNullParameter:
      *status = NULL_PARAMETER;
      return;
    case NotifierResult::kHardwareFailure:
    default:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
  }
}

}  // namespace

NotifierManager& GetNotifierManager() { return CreateNotifierManager(); }

void ShutdownNotifiers() noexcept { GetNotifierManager().Shutdown(); }

}  // namespace hal::vmx

extern "C" {

HAL_NotifierHandle HAL_InitializeNotifier(int32_t* status) {
  hal::init::CheckInit();
  if (!hal::vmx::IsRuntimeInitialized()) {
    *status = INCOMPATIBLE_STATE;
    hal::SetLastError(status, "VMX HAL runtime is not initialized");
    return HAL_kInvalidHandle;
  }
  auto allocation = hal::vmx::GetNotifierManager().Allocate();
  hal::vmx::SetNotifierResult(
      allocation.result, status, "Failed to start VMX Notifier scheduler");
  return allocation.result == hal::vmx::NotifierResult::kOk
             ? allocation.handle
             : HAL_kInvalidHandle;
}

HAL_Bool HAL_SetNotifierThreadPriority(HAL_Bool realTime, int32_t priority,
                                       int32_t* status) {
  auto result = hal::vmx::GetNotifierManager().SetPriority(
      realTime != 0, priority);
  hal::vmx::SetNotifierResult(result, status,
                              "Failed to set VMX Notifier thread priority");
  return result == hal::vmx::NotifierResult::kOk;
}

void HAL_SetNotifierName(HAL_NotifierHandle notifierHandle, const char* name,
                         int32_t* status) {
  auto result =
      hal::vmx::GetNotifierManager().SetName(notifierHandle, name);
  hal::vmx::SetNotifierResult(result, status,
                              "Failed to set VMX Notifier name");
}

void HAL_StopNotifier(HAL_NotifierHandle notifierHandle, int32_t* status) {
  hal::vmx::SetNotifierResult(
      hal::vmx::GetNotifierManager().Stop(notifierHandle), status,
      "Failed to stop VMX Notifier");
}

void HAL_CleanNotifier(HAL_NotifierHandle notifierHandle) {
  hal::vmx::GetNotifierManager().Free(notifierHandle);
}

void HAL_UpdateNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             uint64_t triggerTime, int32_t* status) {
  hal::vmx::SetNotifierResult(
      hal::vmx::GetNotifierManager().Update(notifierHandle, triggerTime),
      status, "Failed to update VMX Notifier alarm");
}

void HAL_CancelNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             int32_t* status) {
  hal::vmx::SetNotifierResult(
      hal::vmx::GetNotifierManager().Cancel(notifierHandle), status,
      "Failed to cancel VMX Notifier alarm");
}

uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle,
                                  int32_t* status) {
  auto result = hal::vmx::GetNotifierManager().Wait(notifierHandle);
  hal::vmx::SetNotifierResult(result.first, status,
                              "Failed to wait for VMX Notifier alarm");
  return result.first == hal::vmx::NotifierResult::kOk ? result.second : 0;
}

}  // extern "C"
