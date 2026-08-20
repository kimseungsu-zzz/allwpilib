// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/Threads.h"

#include <pthread.h>
#include <sched.h>

#include "hal/Errors.h"

namespace hal::init {
void InitializeThreads() {}
}  // namespace hal::init

extern "C" {

int32_t HAL_GetThreadPriority(NativeThreadHandle handle, HAL_Bool* isRealTime,
                              int32_t* status) {
  if (handle == nullptr || isRealTime == nullptr || status == nullptr) {
    if (status) {
      *status = NULL_PARAMETER;
    }
    return 0;
  }

  sched_param parameters{};
  int policy = 0;
  const auto result = pthread_getschedparam(
      *reinterpret_cast<const pthread_t*>(handle), &policy, &parameters);
  if (result != 0) {
    *status = HAL_THREAD_PRIORITY_ERROR;
    return 0;
  }

  *status = HAL_SUCCESS;
  if (policy == SCHED_FIFO || policy == SCHED_RR) {
    *isRealTime = true;
    return parameters.sched_priority;
  }

  *isRealTime = false;
  return 0;
}

int32_t HAL_GetCurrentThreadPriority(HAL_Bool* isRealTime, int32_t* status) {
  auto thread = pthread_self();
  return HAL_GetThreadPriority(&thread, isRealTime, status);
}

HAL_Bool HAL_SetThreadPriority(NativeThreadHandle handle, HAL_Bool realTime,
                               int32_t priority, int32_t* status) {
  if (handle == nullptr || status == nullptr) {
    if (status) {
      *status = NULL_PARAMETER;
    }
    return false;
  }

  const int policy = realTime ? SCHED_FIFO : SCHED_OTHER;
  if (realTime &&
      (priority < sched_get_priority_min(policy) ||
       priority > sched_get_priority_max(policy))) {
    *status = HAL_THREAD_PRIORITY_RANGE_ERROR;
    return false;
  }

  sched_param parameters{};
  int currentPolicy = 0;
  if (pthread_getschedparam(*reinterpret_cast<const pthread_t*>(handle),
                            &currentPolicy, &parameters) != 0) {
    *status = HAL_THREAD_PRIORITY_ERROR;
    return false;
  }
  parameters.sched_priority = realTime ? priority : 0;

  if (pthread_setschedparam(*reinterpret_cast<const pthread_t*>(handle),
                            policy, &parameters) != 0) {
    *status = HAL_THREAD_PRIORITY_ERROR;
    return false;
  }

  *status = HAL_SUCCESS;
  return true;
}

HAL_Bool HAL_SetCurrentThreadPriority(HAL_Bool realTime, int32_t priority,
                                      int32_t* status) {
  auto thread = pthread_self();
  return HAL_SetThreadPriority(&thread, realTime, priority, status);
}

}  // extern "C"
