// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SPIAutoInternal.h"

#include <limits>
#include <memory>

#include "DIOInternal.h"
#include "InterruptInternal.h"
#include "NotifierInternal.h"
#include "VMXDigitalSource.h"
#include "VMXRuntime.h"
#include "VMXTimeInternal.h"

namespace hal::vmx {
namespace {

class VMXAutoRateWaiter final : public SPIAutoWaiter {
 public:
  VMXAutoRateWaiter() noexcept {
    auto allocation = GetNotifierManager().Allocate();
    if (allocation.result == NotifierResult::kOk) {
      m_handle = allocation.handle;
    }
  }

  ~VMXAutoRateWaiter() override {
    if (m_handle != HAL_kInvalidHandle) {
      GetNotifierManager().Free(m_handle);
    }
  }

  bool IsValid() const noexcept { return m_handle != HAL_kInvalidHandle; }

  bool WaitUntil(uint64_t deadline, uint64_t& firedAt) noexcept override {
    firedAt = 0;
    if (!IsValid() || m_stopped.load(std::memory_order_acquire)) {
      return false;
    }
    if (GetNotifierManager().Update(m_handle, deadline) !=
        NotifierResult::kOk) {
      return false;
    }
    auto result = GetNotifierManager().Wait(m_handle);
    if (m_stopped.load(std::memory_order_acquire) ||
        result.first != NotifierResult::kOk) {
      return false;
    }
    firedAt = result.second;
    return true;
  }

  void Stop() noexcept override {
    if (m_stopped.exchange(true, std::memory_order_acq_rel)) {
      return;
    }
    if (IsValid()) {
      GetNotifierManager().Stop(m_handle);
    }
  }

 private:
  HAL_NotifierHandle m_handle = HAL_kInvalidHandle;
  std::atomic_bool m_stopped = false;
};

std::unique_ptr<SPIAutoWaiter> CreateVMXAutoRateWaiter() {
  auto waiter = std::make_unique<VMXAutoRateWaiter>();
  return waiter->IsValid() ? std::move(waiter) : nullptr;
}

class VMXAutoTriggerWaiter final : public SPIAutoTriggerWaiter {
 public:
  VMXAutoTriggerWaiter(int32_t channel, bool rising, bool falling)
      : m_state{std::make_shared<InterruptCallbackState>(
            InterruptRisingMask(0), InterruptFallingMask(0))} {
    if (!m_state || (!rising && !falling)) {
      return;
    }
    auto factory = GetInterruptBackendFactory();
    if (!factory) {
      return;
    }
    VMXInterruptEdge edge = VMXInterruptEdge::kRising;
    if (rising && falling) {
      edge = VMXInterruptEdge::kBoth;
    } else if (falling) {
      edge = VMXInterruptEdge::kFalling;
    }
    try {
      m_backend = factory(channel, edge, m_state.get());
    } catch (...) {
      m_backend.reset();
    }
    if (m_backend) {
      m_state->SetActive(true);
      m_eligibleMask = (rising ? InterruptRisingMask(0) : 0) |
                       (falling ? InterruptFallingMask(0) : 0);
      m_valid.store(true, std::memory_order_release);
    }
  }

  ~VMXAutoTriggerWaiter() override { Stop(); }

  bool IsValid() const noexcept {
    return m_valid.load(std::memory_order_acquire);
  }

  bool Wait() noexcept override {
    if (!m_valid.load(std::memory_order_acquire) ||
        m_stopped.load(std::memory_order_acquire)) {
      return false;
    }
    auto result = m_state->Wait(m_eligibleMask,
                                std::numeric_limits<double>::infinity(),
                                false);
    return result.result == InterruptResult::kOk &&
           !m_stopped.load(std::memory_order_acquire);
  }

  void Stop() noexcept override {
    if (m_stopped.exchange(true, std::memory_order_acq_rel)) {
      return;
    }
    if (m_state) {
      m_state->Close();
    }
    m_backend.reset();
    m_valid.store(false, std::memory_order_release);
  }

 private:
  std::shared_ptr<InterruptCallbackState> m_state;
  std::unique_ptr<InterruptBackend> m_backend;
  uint64_t m_eligibleMask = 0;
  std::atomic_bool m_stopped = false;
  std::atomic_bool m_valid = false;
};

std::unique_ptr<SPIAutoTriggerWaiter> CreateVMXAutoTriggerWaiter(
    int32_t channel, bool rising, bool falling) {
  auto trigger =
      std::make_unique<VMXAutoTriggerWaiter>(channel, rising, falling);
  return trigger->IsValid() ? std::move(trigger) : nullptr;
}

}  // namespace

SPIAutoResult SPIAutoManager::ValidateTriggerSource(
    HAL_Handle source, HAL_AnalogTriggerType triggerType,
    int32_t& channel) noexcept {
  const auto decoded = DecodeVMXDigitalSource(source, triggerType);
  if (decoded == VMXDigitalSourceResult::kUnsupportedAnalogTrigger) {
    return SPIAutoResult::kUnsupportedSource;
  }
  if (decoded != VMXDigitalSourceResult::kOk) {
    return SPIAutoResult::kInvalidTrigger;
  }
  const auto validation = GetDIOManager().ValidateInputSource(source);
  channel = validation.second;
  if (validation.first != DIOResult::kOk) {
    return validation.first == DIOResult::kUnsupportedCapability
               ? SPIAutoResult::kUnsupportedSource
               : SPIAutoResult::kInvalidTrigger;
  }
  if (IsRuntimeInitialized() &&
      !GetVMXCapabilityProvider().SupportsPhysical(
          channel, VMXCapability::kInterruptInput)) {
    return SPIAutoResult::kUnsupportedSource;
  }
  return SPIAutoResult::kOk;
}

SPIAutoManager& GetSPIAutoManager() {
  static SPIAutoManager manager{
      GetSPIManager(), [] { return GetTimeMicroseconds(nullptr); },
      CreateVMXAutoRateWaiter, CreateVMXAutoTriggerWaiter};
  return manager;
}

void ShutdownSPIAuto() noexcept { GetSPIAutoManager().Shutdown(); }

}  // namespace hal::vmx
