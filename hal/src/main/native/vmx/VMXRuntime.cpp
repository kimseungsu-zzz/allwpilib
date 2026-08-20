// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VMXRuntime.h"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>

#include <unistd.h>

#include "VMXPi.h"

namespace hal::vmx {
namespace {

class VmxSdkContext final : public SdkContext {
 public:
  VmxSdkContext() : m_vmx{std::make_shared<VMXPi>(true, 50)} {
    if (!m_vmx) {
      m_error = "unable to allocate VMXPi context";
    } else if (!m_vmx->IsOpen()) {
      if (geteuid() != 0) {
        m_error =
            "VMXPi context is not open; the process is not running as root "
            "(configure robot service permissions)";
      } else {
        m_error = "VMXPi context is not open (hardware unavailable or SDK "
                  "initialization failed)";
      }
    }
  }

  bool IsReady() const noexcept override {
    return m_vmx && m_vmx->IsOpen();
  }

  std::shared_ptr<VMXPi> GetVMXPi() const noexcept override { return m_vmx; }

  std::string_view GetError() const noexcept override { return m_error; }

 private:
  std::shared_ptr<VMXPi> m_vmx;
  std::string m_error;
};

Runtime& GetRuntimeInstance() {
  static Runtime runtime{[] { return std::make_unique<VmxSdkContext>(); }};
  return runtime;
}

void ShutdownRuntimeAtExit() {
  ShutdownRuntime();
}

}  // namespace

bool InitializeRuntime() noexcept {
  auto& runtime = GetRuntimeInstance();
  if (!runtime.Initialize()) {
    auto error = runtime.GetLastError();
    std::fprintf(stderr, "VMX HAL initialization failed: %s\n",
                 error.empty() ? "unknown VMX SDK error" : error.c_str());
    return false;
  }

  static std::once_flag atExitRegistered;
  std::call_once(atExitRegistered,
                 [] { std::atexit(ShutdownRuntimeAtExit); });
  return true;
}

void ShutdownRuntime() noexcept {
  GetRuntimeInstance().Shutdown();
}

bool IsRuntimeInitialized() noexcept {
  return GetRuntimeInstance().IsInitialized();
}

RuntimeState GetRuntimeState() noexcept {
  return GetRuntimeInstance().GetState();
}

std::shared_ptr<VMXPi> GetRuntimeContext() noexcept {
  return GetRuntimeInstance().GetContext();
}

}  // namespace hal::vmx
