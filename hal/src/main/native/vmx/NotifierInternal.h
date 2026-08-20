// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "hal/Notifier.h"
#include "hal/Errors.h"
#include "hal/Types.h"
#include "hal/handles/UnlimitedHandleResource.h"

namespace hal::vmx {

enum class NotifierResult {
  kOk,
  kInvalidHandle,
  kNoResources,
  kStopped,
  kPriorityRange,
  kPriorityFailure,
  kNullParameter,
  kHardwareFailure,
};

struct NotifierSnapshot {
  bool alarmActive = false;
  bool stopped = false;
  uint64_t triggerTime = std::numeric_limits<uint64_t>::max();
  uint64_t generation = 0;
};

// VMX time is an unsigned, wrapping microsecond domain.  Comparing the
// signed half-range of the subtraction keeps alarm ordering correct across a
// uint64_t rollover (alarms are necessarily less than half a wrap apart).
inline bool IsTimeReached(uint64_t currentTime, uint64_t triggerTime) noexcept {
  return static_cast<int64_t>(currentTime - triggerTime) >= 0;
}

class NotifierState final {
 public:
  NotifierResult Update(uint64_t triggerTime) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_stopped) {
      return NotifierResult::kStopped;
    }
    ++m_generation;
    m_triggerTime = triggerTime;
    m_alarmActive = triggerTime != std::numeric_limits<uint64_t>::max();
    m_fired = false;
    m_firedTime = 0;
    m_cv.notify_all();
    return NotifierResult::kOk;
  }

  NotifierResult Cancel() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_stopped) {
      return NotifierResult::kOk;
    }
    ++m_generation;
    m_alarmActive = false;
    m_triggerTime = std::numeric_limits<uint64_t>::max();
    // Cancellation deliberately does not notify the state condition variable.
    // HAL_CancelNotifierAlarm() must not release a waiter by itself.
    return NotifierResult::kOk;
  }

  void Stop() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_stopped) {
      return;
    }
    ++m_generation;
    m_stopped = true;
    m_alarmActive = false;
    m_triggerTime = std::numeric_limits<uint64_t>::max();
    m_fired = false;
    m_firedTime = 0;
    m_cv.notify_all();
  }

  NotifierResult SetName(const char* name) noexcept {
    if (name == nullptr) {
      return NotifierResult::kNullParameter;
    }
    std::scoped_lock lock{m_mutex};
    if (m_stopped) {
      return NotifierResult::kStopped;
    }
    try {
      m_name = name;
    } catch (...) {
      return NotifierResult::kHardwareFailure;
    }
    return NotifierResult::kOk;
  }

  std::string GetName() const {
    std::scoped_lock lock{m_mutex};
    return m_name;
  }

  NotifierSnapshot Snapshot() const noexcept {
    std::scoped_lock lock{m_mutex};
    return {m_alarmActive, m_stopped, m_triggerTime, m_generation};
  }

  bool Fire(uint64_t expectedGeneration, uint64_t expectedTrigger,
            uint64_t currentTime) noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_stopped || !m_alarmActive || m_generation != expectedGeneration ||
        m_triggerTime != expectedTrigger ||
        !IsTimeReached(currentTime, m_triggerTime)) {
      return false;
    }
    m_alarmActive = false;
    m_fired = true;
    m_firedTime = currentTime;
    m_cv.notify_all();
    return true;
  }

  uint64_t Wait() noexcept {
    std::unique_lock lock{m_mutex};
    m_cv.wait(lock, [this] { return m_stopped || m_fired; });
    if (m_stopped) {
      return 0;
    }
    auto firedTime = m_firedTime;
    m_fired = false;
    m_firedTime = 0;
    return firedTime;
  }

 private:
  mutable std::mutex m_mutex;
  std::condition_variable m_cv;
  uint64_t m_triggerTime = std::numeric_limits<uint64_t>::max();
  uint64_t m_firedTime = 0;
  uint64_t m_generation = 0;
  bool m_alarmActive = false;
  bool m_stopped = false;
  bool m_fired = false;
  std::string m_name;
};

using NotifierClock = std::function<uint64_t()>;
using NotifierPrioritySetter = std::function<bool(
    std::thread::native_handle_type, bool, int32_t, int32_t&)>;
using NotifierHardwareArm = std::function<bool(uint64_t, void*)>;
using NotifierHardwareDisarm = std::function<void()>;

class NotifierScheduler final {
 public:
  explicit NotifierScheduler(NotifierClock clock,
                             NotifierPrioritySetter prioritySetter = {},
                             NotifierHardwareArm hardwareArm = {},
                             NotifierHardwareDisarm hardwareDisarm = {})
      : m_clock{std::move(clock)},
        m_prioritySetter{std::move(prioritySetter)},
        m_hardwareArm{std::move(hardwareArm)},
        m_hardwareDisarm{std::move(hardwareDisarm)} {}

  ~NotifierScheduler() { Shutdown(); }

  NotifierScheduler(const NotifierScheduler&) = delete;
  NotifierScheduler& operator=(const NotifierScheduler&) = delete;

  NotifierResult Add(const std::shared_ptr<NotifierState>& state) noexcept {
    bool startThread = false;
    bool applyPriority = false;
    bool realTime = false;
    int32_t priority = 0;
    std::thread::native_handle_type nativeHandle{};
    {
      std::scoped_lock lock{m_mutex};
      try {
        m_states.push_back(state);
      } catch (...) {
        return NotifierResult::kNoResources;
      }
      if (!m_running) {
        try {
          m_running = true;
          m_thread = std::thread([this] { Run(); });
          startThread = true;
          applyPriority = m_priorityRealTime;
          realTime = m_priorityRealTime;
          priority = m_priority;
          if (applyPriority) {
            nativeHandle = m_thread.native_handle();
          }
        } catch (...) {
          m_running = false;
          m_states.pop_back();
          return NotifierResult::kHardwareFailure;
        }
      }
    }

    if (startThread && applyPriority) {
      int32_t priorityStatus = HAL_SUCCESS;
      if (!m_prioritySetter ||
          !m_prioritySetter(nativeHandle, realTime, priority,
                            priorityStatus)) {
        Shutdown();
        return priorityStatus == PARAMETER_OUT_OF_RANGE
                   ? NotifierResult::kPriorityRange
                   : NotifierResult::kPriorityFailure;
      }
    }
    Wake();
    return NotifierResult::kOk;
  }

  void Remove(const std::shared_ptr<NotifierState>& state) noexcept {
    {
      std::scoped_lock lock{m_mutex};
      m_states.erase(
          std::remove(m_states.begin(), m_states.end(), state), m_states.end());
      ++m_wakeupGeneration;
    }
    m_cv.notify_all();
  }

  NotifierResult SetPriority(bool realTime, int32_t priority) noexcept {
    if (realTime && (priority < 1 || priority > 99)) {
      return NotifierResult::kPriorityRange;
    }
    if (!realTime) {
      priority = 0;
    }

    std::thread::native_handle_type nativeHandle{};
    bool apply = false;
    {
      std::scoped_lock lock{m_mutex};
      if (m_running && m_thread.joinable()) {
        nativeHandle = m_thread.native_handle();
        apply = true;
      }
    }
    if (apply) {
      int32_t priorityStatus = HAL_SUCCESS;
      if (!m_prioritySetter ||
          !m_prioritySetter(nativeHandle, realTime, priority,
                            priorityStatus)) {
        return priorityStatus == PARAMETER_OUT_OF_RANGE
                   ? NotifierResult::kPriorityRange
                   : NotifierResult::kPriorityFailure;
      }
    }
    {
      std::scoped_lock lock{m_mutex};
      m_priorityRealTime = realTime;
      m_priority = priority;
    }
    return NotifierResult::kOk;
  }

  void Wake() noexcept {
    {
      std::scoped_lock lock{m_mutex};
      ++m_wakeupGeneration;
    }
    m_cv.notify_all();
  }

  // VMX timer callbacks only wake the scheduler.  The worker re-reads the
  // canonical clock and validates the alarm generation before firing it.
  void HardwareWake(uint64_t timestamp) noexcept {
    static_cast<void>(timestamp);
    Wake();
  }

  void Shutdown() noexcept {
    std::vector<std::shared_ptr<NotifierState>> states;
    std::thread thread;
    {
      std::scoped_lock lock{m_mutex};
      if (!m_running && !m_thread.joinable()) {
        return;
      }
      m_running = false;
      states = m_states;
      m_states.clear();
      thread = std::move(m_thread);
      ++m_wakeupGeneration;
    }
    for (auto& state : states) {
      state->Stop();
    }
    m_cv.notify_all();
    if (thread.joinable()) {
      thread.join();
    }
    DisarmHardwareTimer();
  }

 private:
  void Run() noexcept {
    for (;;) {
      std::vector<std::shared_ptr<NotifierState>> states;
      uint64_t observedWakeupGeneration = 0;
      bool shouldStop = false;
      {
        std::scoped_lock lock{m_mutex};
        if (!m_running) {
          shouldStop = true;
        } else {
          states = m_states;
          observedWakeupGeneration = m_wakeupGeneration;
        }
      }
      if (shouldStop) {
        DisarmHardwareTimer();
        return;
      }

      uint64_t currentTime = 0;
      try {
        currentTime = m_clock ? m_clock() : 0;
      } catch (...) {
        currentTime = 0;
      }

      uint64_t earliestDelta = std::numeric_limits<uint64_t>::max();
      uint64_t earliestTrigger = std::numeric_limits<uint64_t>::max();
      std::vector<std::pair<std::shared_ptr<NotifierState>, NotifierSnapshot>> due;
      for (const auto& state : states) {
        auto snapshot = state->Snapshot();
        if (snapshot.stopped || !snapshot.alarmActive) {
          continue;
        }
        if (IsTimeReached(currentTime, snapshot.triggerTime)) {
          due.emplace_back(state, snapshot);
        } else {
          const auto delta = snapshot.triggerTime - currentTime;
          if (delta < earliestDelta) {
            earliestDelta = delta;
            earliestTrigger = snapshot.triggerTime;
          }
        }
      }

      if (!due.empty()) {
        DisarmHardwareTimer();
        for (const auto& entry : due) {
          entry.first->Fire(entry.second.generation, entry.second.triggerTime,
                            currentTime);
        }
        continue;
      }

      bool hardwareArmed = false;
      if (earliestDelta != std::numeric_limits<uint64_t>::max()) {
        hardwareArmed = ArmHardwareTimer(earliestTrigger);
      } else {
        DisarmHardwareTimer();
      }

      std::unique_lock lock{m_mutex};
      if (!m_running) {
        lock.unlock();
        DisarmHardwareTimer();
        return;
      }
      if (observedWakeupGeneration != m_wakeupGeneration) {
        continue;
      }
      if (earliestDelta == std::numeric_limits<uint64_t>::max()) {
        m_cv.wait(lock);
      } else {
        uint64_t delta = earliestDelta;
        auto maxDuration = static_cast<uint64_t>(
            std::numeric_limits<std::chrono::microseconds::rep>::max());
        if (delta > maxDuration) {
          delta = maxDuration;
        }
        if (hardwareArmed) {
          // The VMX timer callback is the primary wakeup.  The bounded host
          // wait remains a safety net if a notification is lost by the SDK.
          m_cv.wait_for(lock, std::chrono::microseconds{
                                    static_cast<int64_t>(delta)});
        } else {
          m_cv.wait_for(lock, std::chrono::microseconds{
                                    static_cast<int64_t>(delta)});
        }
      }
    }
  }

  bool ArmHardwareTimer(uint64_t triggerTime) noexcept {
    if (!m_hardwareArm) {
      return false;
    }
    if (m_hardwareTimerArmed && m_hardwareTimerDeadline == triggerTime) {
      return true;
    }
    if (m_hardwareTimerArmed && m_hardwareDisarm) {
      m_hardwareDisarm();
      m_hardwareTimerArmed = false;
    }
    try {
      if (m_hardwareArm(triggerTime, this)) {
        m_hardwareTimerArmed = true;
        m_hardwareTimerDeadline = triggerTime;
        return true;
      }
    } catch (...) {
    }
    m_hardwareTimerArmed = false;
    return false;
  }

  void DisarmHardwareTimer() noexcept {
    if (!m_hardwareTimerArmed) {
      return;
    }
    try {
      if (m_hardwareDisarm) {
        m_hardwareDisarm();
      }
    } catch (...) {
    }
    m_hardwareTimerArmed = false;
  }

  NotifierClock m_clock;
  NotifierPrioritySetter m_prioritySetter;
  NotifierHardwareArm m_hardwareArm;
  NotifierHardwareDisarm m_hardwareDisarm;
  std::mutex m_mutex;
  std::condition_variable m_cv;
  std::vector<std::shared_ptr<NotifierState>> m_states;
  std::thread m_thread;
  uint64_t m_wakeupGeneration = 0;
  bool m_running = false;
  bool m_priorityRealTime = false;
  bool m_hardwareTimerArmed = false;
  uint64_t m_hardwareTimerDeadline = 0;
  int32_t m_priority = 0;
};

struct NotifierAllocationResult {
  HAL_NotifierHandle handle = HAL_kInvalidHandle;
  NotifierResult result = NotifierResult::kHardwareFailure;
};

class NotifierHandleResource final
    : public hal::UnlimitedHandleResource<HAL_NotifierHandle, NotifierState,
                                          HAL_HandleEnum::Notifier> {
 public:
  NotifierHandleResource() { m_version = 0; }
};

class NotifierManager final {
 public:
  explicit NotifierManager(NotifierClock clock,
                           NotifierPrioritySetter prioritySetter = {},
                           NotifierHardwareArm hardwareArm = {},
                           NotifierHardwareDisarm hardwareDisarm = {})
      : m_scheduler{std::move(clock), std::move(prioritySetter),
                     std::move(hardwareArm), std::move(hardwareDisarm)} {}

  NotifierAllocationResult Allocate() noexcept {
    std::scoped_lock lock{m_mutex};
    try {
      auto state = std::make_shared<NotifierState>();
      auto handle = m_handles.Allocate(state);
      if (handle == HAL_kInvalidHandle) {
        return {HAL_kInvalidHandle, NotifierResult::kNoResources};
      }
      auto result = m_scheduler.Add(state);
      if (result != NotifierResult::kOk) {
        m_handles.Free(handle);
        return {HAL_kInvalidHandle, result};
      }
      return {handle, NotifierResult::kOk};
    } catch (...) {
      return {HAL_kInvalidHandle, NotifierResult::kNoResources};
    }
  }

  void Free(HAL_NotifierHandle handle) noexcept {
    std::scoped_lock lock{m_mutex};
    auto state = m_handles.Free(handle);
    if (!state) {
      return;
    }
    state->Stop();
    m_scheduler.Remove(state);
  }

  void Shutdown() noexcept {
    std::scoped_lock lock{m_mutex};
    m_scheduler.Shutdown();
    m_handles.ResetHandles();
  }

  bool IsValid(HAL_NotifierHandle handle) noexcept {
    return Get(handle) != nullptr;
  }

  NotifierResult SetPriority(bool realTime, int32_t priority) noexcept {
    return m_scheduler.SetPriority(realTime, priority);
  }

  NotifierResult SetName(HAL_NotifierHandle handle, const char* name) noexcept {
    auto state = Get(handle);
    if (!state) {
      return NotifierResult::kInvalidHandle;
    }
    if (name == nullptr) {
      return NotifierResult::kHardwareFailure;
    }
    return state->SetName(name);
  }

  NotifierResult Stop(HAL_NotifierHandle handle) noexcept {
    auto state = Get(handle);
    if (!state) {
      return NotifierResult::kInvalidHandle;
    }
    state->Stop();
    m_scheduler.Wake();
    return NotifierResult::kOk;
  }

  NotifierResult Update(HAL_NotifierHandle handle,
                        uint64_t triggerTime) noexcept {
    auto state = Get(handle);
    if (!state) {
      return NotifierResult::kInvalidHandle;
    }
    auto result = state->Update(triggerTime);
    m_scheduler.Wake();
    return result;
  }

  NotifierResult Cancel(HAL_NotifierHandle handle) noexcept {
    auto state = Get(handle);
    if (!state) {
      return NotifierResult::kInvalidHandle;
    }
    auto result = state->Cancel();
    m_scheduler.Wake();
    return result;
  }

  std::pair<NotifierResult, uint64_t> Wait(
      HAL_NotifierHandle handle) noexcept {
    auto state = Get(handle);
    if (!state) {
      return {NotifierResult::kInvalidHandle, 0};
    }
    return {NotifierResult::kOk, state->Wait()};
  }

 private:
  std::shared_ptr<NotifierState> Get(HAL_NotifierHandle handle) noexcept {
    return m_handles.Get(handle);
  }

  std::mutex m_mutex;
  NotifierHandleResource m_handles;
  NotifierScheduler m_scheduler;
};

NotifierManager& GetNotifierManager();
void ShutdownNotifiers() noexcept;

}  // namespace hal::vmx
