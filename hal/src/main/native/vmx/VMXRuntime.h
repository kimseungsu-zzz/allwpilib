// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <atomic>
#include <condition_variable>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>

class VMXPi;

namespace hal::vmx {

enum class RuntimeState {
  kUninitialized,
  kInitializing,
  kReady,
  kFailed,
  kShuttingDown,
  kShutdown,
};

/** Small seam around the VMX SDK so lifecycle behavior can be host-tested. */
class SdkContext {
 public:
  virtual ~SdkContext() = default;

  virtual bool IsReady() const noexcept = 0;
  virtual std::shared_ptr<VMXPi> GetVMXPi() const noexcept = 0;
  virtual std::string_view GetError() const noexcept = 0;
};

class Runtime final {
 public:
  using ContextFactory = std::function<std::unique_ptr<SdkContext>()>;

  explicit Runtime(ContextFactory factory) : m_factory{std::move(factory)} {}

  Runtime(const Runtime&) = delete;
  Runtime& operator=(const Runtime&) = delete;

  bool Initialize() noexcept {
    std::unique_lock lock{m_mutex};

    if (m_state.load(std::memory_order_acquire) == RuntimeState::kReady) {
      return true;
    }

    if (m_state.load(std::memory_order_relaxed) ==
        RuntimeState::kInitializing) {
      m_stateChanged.wait(lock, [this] {
        return m_state.load(std::memory_order_acquire) !=
               RuntimeState::kInitializing;
      });
      return m_state.load(std::memory_order_acquire) == RuntimeState::kReady;
    }

    if (m_state.load(std::memory_order_relaxed) ==
            RuntimeState::kShuttingDown ||
        m_state.load(std::memory_order_relaxed) == RuntimeState::kShutdown) {
      return false;
    }

    m_lastError.clear();
    m_state.store(RuntimeState::kInitializing, std::memory_order_release);
    lock.unlock();

    std::unique_ptr<SdkContext> context;
    std::string error;
    try {
      context = m_factory();
      if (!context) {
        error = "VMX SDK context factory returned no context";
      } else if (!context->IsReady()) {
        error = context->GetError();
        if (error.empty()) {
          error = "VMX hardware did not open";
        }
        context.reset();
      }
    } catch (const std::exception& ex) {
      error = ex.what();
    } catch (...) {
      error = "VMX SDK threw an unknown exception";
    }

    lock.lock();
    bool initialized = context != nullptr;
    if (initialized) {
      m_context = std::move(context);
      m_state.store(RuntimeState::kReady, std::memory_order_release);
    } else {
      m_lastError = std::move(error);
      m_state.store(RuntimeState::kFailed, std::memory_order_release);
    }
    lock.unlock();
    m_stateChanged.notify_all();
    return initialized;
  }

  void Shutdown() noexcept {
    std::unique_lock lock{m_mutex};
    m_stateChanged.wait(lock, [this] {
      auto state = m_state.load(std::memory_order_acquire);
      return state != RuntimeState::kInitializing &&
             state != RuntimeState::kShuttingDown;
    });

    if (m_state.load(std::memory_order_relaxed) == RuntimeState::kShutdown) {
      return;
    }

    m_state.store(RuntimeState::kShuttingDown, std::memory_order_release);
    auto context = std::move(m_context);
    lock.unlock();

    // The imported VMX drivers use VMXPi as an RAII object and expose no
    // separate shutdown call. Releasing the final shared owner closes it.
    context.reset();

    lock.lock();
    m_state.store(RuntimeState::kShutdown, std::memory_order_release);
    lock.unlock();
    m_stateChanged.notify_all();
  }

  RuntimeState GetState() const noexcept {
    return m_state.load(std::memory_order_acquire);
  }

  bool IsInitialized() const noexcept {
    return GetState() == RuntimeState::kReady;
  }

  std::shared_ptr<VMXPi> GetContext() const noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_context ||
        m_state.load(std::memory_order_relaxed) != RuntimeState::kReady) {
      return {};
    }
    return m_context->GetVMXPi();
  }

  std::string GetLastError() const {
    std::scoped_lock lock{m_mutex};
    return m_lastError;
  }

 private:
  ContextFactory m_factory;
  mutable std::mutex m_mutex;
  std::condition_variable m_stateChanged;
  std::atomic<RuntimeState> m_state{RuntimeState::kUninitialized};
  std::unique_ptr<SdkContext> m_context;
  std::string m_lastError;
};

bool InitializeRuntime() noexcept;
void ShutdownRuntime() noexcept;
bool IsRuntimeInitialized() noexcept;
RuntimeState GetRuntimeState() noexcept;
std::shared_ptr<VMXPi> GetRuntimeContext() noexcept;

}  // namespace hal::vmx
