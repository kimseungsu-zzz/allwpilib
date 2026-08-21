// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/SPI.h"

#include <memory>
#include <string_view>

#include "HALInitializer.h"
#include "HALInternal.h"
#include "SPIAutoInternal.h"
#include "SPIInternal.h"
#include "VMXPi.h"
#include "VMXRuntime.h"
#include "hal/Errors.h"

namespace hal::vmx {
namespace {

class DriverSPIBackend final : public SPIBackend {
 public:
  DriverSPIBackend(std::shared_ptr<VMXPi> context,
                   const VMXCommDIOChannelMap& channels,
                   const SPIPortConfig& config)
      : m_context{std::move(context)}, m_channels{channels} {
    m_initialized = Activate(config);
  }

  ~DriverSPIBackend() override {
    if (m_initialized && m_context) {
      VMXErrorCode error;
      m_context->io.DeallocateResource(m_resourceHandle, &error);
    }
  }

  bool Transaction(uint8_t* dataToSend, uint8_t* dataReceived,
                   uint16_t size) noexcept override {
    if (!m_initialized) {
      return false;
    }
    if (size == 0) {
      return true;
    }
    try {
      VMXErrorCode error;
      return m_context->io.SPI_Transaction(m_resourceHandle, dataToSend,
                                            dataReceived, size, &error);
    } catch (...) {
      return false;
    }
  }

  bool Write(uint8_t* dataToSend, uint16_t size) noexcept override {
    if (!m_initialized) {
      return false;
    }
    if (size == 0) {
      return true;
    }
    try {
      VMXErrorCode error;
      return m_context->io.SPI_Write(m_resourceHandle, dataToSend, size,
                                     &error);
    } catch (...) {
      return false;
    }
  }

  bool Read(uint8_t* dataReceived, uint16_t size) noexcept override {
    if (!m_initialized) {
      return false;
    }
    if (size == 0) {
      return true;
    }
    try {
      VMXErrorCode error;
      return m_context->io.SPI_Read(m_resourceHandle, dataReceived, size,
                                    &error);
    } catch (...) {
      return false;
    }
  }

  bool Reconfigure(const SPIPortConfig& config) noexcept override {
    if (!m_initialized || !m_context) {
      return false;
    }

    const auto previous = m_config;
    VMXErrorCode error;
    m_context->io.DeallocateResource(m_resourceHandle, &error);
    m_initialized = false;
    if (Activate(config)) {
      return true;
    }

    // A configuration update must not leave the adapter in a silently
    // different state.  Restore the previous VMX resource if possible; the
    // manager retains the old logical configuration when this fails.
    m_initialized = Activate(previous);
    return false;
  }

  int32_t GetHandle() const noexcept override {
    return m_initialized ? static_cast<int32_t>(m_resourceHandle) : 0;
  }

 private:
  bool Activate(const SPIPortConfig& config) noexcept {
    if (!m_context || !m_context->IsOpen()) {
      return false;
    }
    try {
      VMXErrorCode error;
      ::SPIConfig sdkConfig{
          static_cast<uint32_t>(config.clockRate),
          static_cast<uint8_t>(config.mode), config.chipSelectActiveLow, true};
      ::VMXChannelInfo clk{
          static_cast<VMXChannelIndex>(m_channels.spiCLK),
          VMXChannelCapability::SPI_CLK};
      ::VMXChannelInfo mosi{
          static_cast<VMXChannelIndex>(m_channels.spiMOSI),
          VMXChannelCapability::SPI_MOSI};
      ::VMXChannelInfo miso{
          static_cast<VMXChannelIndex>(m_channels.spiMISO),
          VMXChannelCapability::SPI_MISO};
      ::VMXChannelInfo cs{static_cast<VMXChannelIndex>(m_channels.spiCS),
                          VMXChannelCapability::SPI_CS};
      if (!clk.IsValid() || !mosi.IsValid() || !miso.IsValid() ||
          !cs.IsValid()) {
        return false;
      }
      auto activated = m_context->io.ActivateQuadchannelResource(
          clk, mosi, miso, cs, &sdkConfig, m_resourceHandle, &error);
      if (activated) {
        m_config = config;
      }
      return activated;
    } catch (...) {
      return false;
    }
  }

  std::shared_ptr<VMXPi> m_context;
  VMXCommDIOChannelMap m_channels;
  VMXResourceHandle m_resourceHandle = 0;
  SPIPortConfig m_config;
  bool m_initialized = false;
};

std::unique_ptr<SPIBackend> CreateSPIBackend(
    HAL_SPIPort port, const VMXCommDIOChannelMap& channels,
    const SPIPortConfig& config) {
  static_cast<void>(port);  // VMX exposes one physical SPI resource.
  auto context = GetRuntimeContext();
  if (!context) {
    return nullptr;
  }
  auto backend =
      std::make_unique<DriverSPIBackend>(std::move(context), channels, config);
  return backend->GetHandle() != 0 ? std::move(backend) : nullptr;
}

void SetSPIResult(SPIResult result, int32_t* status, std::string_view message,
                  int32_t port = -1) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  switch (result) {
    case SPIResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case SPIResult::kPortOutOfRange:
      *status = RESOURCE_OUT_OF_RANGE;
      hal::SetLastErrorIndexOutOfRange(status, message, 0, 4, port);
      return;
    case SPIResult::kUnsupportedPort:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
    case SPIResult::kInvalidSize:
    case SPIResult::kInvalidClockRate:
    case SPIResult::kInvalidMode:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status, message);
      return;
    case SPIResult::kNullPointer:
      *status = NULL_PARAMETER;
      hal::SetLastError(status, message);
      return;
    case SPIResult::kResourceConflict:
      *status = RESOURCE_IS_ALLOCATED;
      hal::SetLastError(status, message);
      return;
    case SPIResult::kNoResources:
      *status = NO_AVAILABLE_RESOURCES;
      hal::SetLastError(status, message);
      return;
    case SPIResult::kNotInitialized:
    case SPIResult::kUnsupportedConfig:
    case SPIResult::kHardwareFailure:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
  }
}

void ReportSPIResult(SPIResult result, std::string_view message,
                     int32_t port = -1) {
  if (result != SPIResult::kOk) {
    SetSPIResult(result, nullptr, message, port);
  }
}

int32_t ReturnSPIResult(SPIResult result, int32_t transferred,
                        std::string_view message, int32_t port) {
  SetSPIResult(result, nullptr, message, port);
  return result == SPIResult::kOk ? transferred : -1;
}

void SetSPIAutoResult(SPIAutoResult result, int32_t* status,
                      std::string_view message, int32_t port = -1) {
  if (status == nullptr) {
    return;
  }
  switch (result) {
    case SPIAutoResult::kOk:
      *status = HAL_SUCCESS;
      return;
    case SPIAutoResult::kPortOutOfRange:
      *status = RESOURCE_OUT_OF_RANGE;
      hal::SetLastErrorIndexOutOfRange(status, message, 0, 4, port);
      return;
    case SPIAutoResult::kAlreadyAllocated:
    case SPIAutoResult::kResourceConflict:
      *status = RESOURCE_IS_ALLOCATED;
      hal::SetLastError(status, message);
      return;
    case SPIAutoResult::kInvalidSize:
    case SPIAutoResult::kInvalidPeriod:
    case SPIAutoResult::kInvalidTrigger:
      *status = PARAMETER_OUT_OF_RANGE;
      hal::SetLastError(status, message);
      return;
    case SPIAutoResult::kNullPointer:
      *status = NULL_PARAMETER;
      hal::SetLastError(status, message);
      return;
    case SPIAutoResult::kUnsupportedSource:
    case SPIAutoResult::kNotInitialized:
    case SPIAutoResult::kUnsupportedStall:
    case SPIAutoResult::kHardwareFailure:
    default:
      *status = INCOMPATIBLE_STATE;
      hal::SetLastError(status, message);
      return;
  }
}

}  // namespace

SPIManager& GetSPIManager() {
  static SPIManager manager{CreateSPIBackend, GetDigitalChannelRegistry(),
                            GetVMXCommDIOChannelMap};
  return manager;
}

}  // namespace hal::vmx

extern "C" {

void HAL_InitializeSPI(HAL_SPIPort port, int32_t* status) {
  hal::init::CheckInit();
  hal::vmx::SetSPIResult(
      hal::vmx::GetSPIManager().Initialize(port), status,
      "VMX SPI onboard CS0-3 and MXP ports alias physical channels 28/29/30/31",
      static_cast<int32_t>(port));
}

int32_t HAL_TransactionSPI(HAL_SPIPort port, const uint8_t* dataToSend,
                           uint8_t* dataReceived, int32_t size) {
  int32_t transferred = -1;
  return hal::vmx::ReturnSPIResult(
      hal::vmx::GetSPIManager().Transaction(port, dataToSend, dataReceived,
                                            size, transferred),
      transferred, "VMX SPI transaction failed", static_cast<int32_t>(port));
}

int32_t HAL_WriteSPI(HAL_SPIPort port, const uint8_t* dataToSend,
                     int32_t sendSize) {
  int32_t transferred = -1;
  return hal::vmx::ReturnSPIResult(
      hal::vmx::GetSPIManager().Write(port, dataToSend, sendSize, transferred),
      transferred, "VMX SPI write failed", static_cast<int32_t>(port));
}

int32_t HAL_ReadSPI(HAL_SPIPort port, uint8_t* buffer, int32_t count) {
  int32_t transferred = -1;
  return hal::vmx::ReturnSPIResult(
      hal::vmx::GetSPIManager().Read(port, buffer, count, transferred),
      transferred, "VMX SPI read failed", static_cast<int32_t>(port));
}

void HAL_CloseSPI(HAL_SPIPort port) {
  hal::vmx::ReportSPIResult(hal::vmx::GetSPIManager().Close(port),
                            "Invalid VMX SPI port",
                            static_cast<int32_t>(port));
}

void HAL_SetSPISpeed(HAL_SPIPort port, int32_t speed) {
  hal::vmx::ReportSPIResult(hal::vmx::GetSPIManager().SetClockRate(port, speed),
                            "VMX SPI clock rate is outside 500 kHz to 10 MHz",
                            static_cast<int32_t>(port));
}

void HAL_SetSPIMode(HAL_SPIPort port, HAL_SPIMode mode) {
  hal::vmx::ReportSPIResult(hal::vmx::GetSPIManager().SetMode(port, mode),
                            "VMX SPI mode must be 0, 1, 2, or 3",
                            static_cast<int32_t>(port));
}

HAL_SPIMode HAL_GetSPIMode(HAL_SPIPort port) {
  return hal::vmx::GetSPIManager().GetMode(port);
}

void HAL_SetSPIChipSelectActiveHigh(HAL_SPIPort port, int32_t* status) {
  hal::vmx::SetSPIResult(
      hal::vmx::GetSPIManager().SetChipSelectActiveLow(port, false), status,
      "VMX SPI chip-select polarity update failed",
      static_cast<int32_t>(port));
}

void HAL_SetSPIChipSelectActiveLow(HAL_SPIPort port, int32_t* status) {
  hal::vmx::SetSPIResult(
      hal::vmx::GetSPIManager().SetChipSelectActiveLow(port, true), status,
      "VMX SPI chip-select polarity update failed",
      static_cast<int32_t>(port));
}

int32_t HAL_GetSPIHandle(HAL_SPIPort port) {
  return hal::vmx::GetSPIManager().GetHandle(port);
}

void HAL_SetSPIHandle(HAL_SPIPort port, int32_t handle) {
  hal::vmx::GetSPIManager().SetHandle(port, handle);
}

void HAL_InitSPIAuto(HAL_SPIPort port, int32_t bufferSize, int32_t* status) {
  hal::vmx::SetSPIAutoResult(
      hal::vmx::GetSPIAutoManager().Initialize(port, bufferSize), status,
      "VMX SPI Auto initialization failed", static_cast<int32_t>(port));
}

void HAL_FreeSPIAuto(HAL_SPIPort port, int32_t* status) {
  hal::vmx::SetSPIAutoResult(
      hal::vmx::GetSPIAutoManager().Free(port), status,
      "VMX SPI Auto free failed", static_cast<int32_t>(port));
}

void HAL_StartSPIAutoRate(HAL_SPIPort port, double period, int32_t* status) {
  hal::vmx::SetSPIAutoResult(
      hal::vmx::GetSPIAutoManager().StartRate(port, period), status,
      "VMX SPI Auto rate start failed", static_cast<int32_t>(port));
}

void HAL_StartSPIAutoTrigger(HAL_SPIPort port, HAL_Handle digitalSourceHandle,
                             HAL_AnalogTriggerType analogTriggerType,
                             HAL_Bool triggerRising, HAL_Bool triggerFalling,
                             int32_t* status) {
  hal::vmx::SetSPIAutoResult(
      hal::vmx::GetSPIAutoManager().StartTrigger(
          port, digitalSourceHandle, analogTriggerType, triggerRising != 0,
          triggerFalling != 0),
      status, "VMX SPI Auto DIO trigger start failed",
      static_cast<int32_t>(port));
}

void HAL_StopSPIAuto(HAL_SPIPort port, int32_t* status) {
  hal::vmx::SetSPIAutoResult(
      hal::vmx::GetSPIAutoManager().Stop(port), status,
      "VMX SPI Auto stop failed", static_cast<int32_t>(port));
}

void HAL_SetSPIAutoTransmitData(HAL_SPIPort port, const uint8_t* dataToSend,
                                int32_t dataSize, int32_t zeroSize,
                                int32_t* status) {
  hal::vmx::SetSPIAutoResult(
      hal::vmx::GetSPIAutoManager().SetTransmitData(
          port, dataToSend, dataSize, zeroSize),
      status, "VMX SPI Auto transmit data is invalid",
      static_cast<int32_t>(port));
}

void HAL_ForceSPIAutoRead(HAL_SPIPort port, int32_t* status) {
  hal::vmx::SetSPIAutoResult(
      hal::vmx::GetSPIAutoManager().SetForceRead(port), status,
      "VMX SPI Auto force read failed", static_cast<int32_t>(port));
}

int32_t HAL_ReadSPIAutoReceivedData(HAL_SPIPort port, uint32_t* buffer,
                                    int32_t numToRead, double timeout,
                                    int32_t* status) {
  hal::vmx::SPIAutoResult result = hal::vmx::SPIAutoResult::kHardwareFailure;
  const auto count = hal::vmx::GetSPIAutoManager().Read(
      port, buffer, numToRead, timeout, result);
  hal::vmx::SetSPIAutoResult(result, status,
                             "VMX SPI Auto receive read failed",
                             static_cast<int32_t>(port));
  return count;
}

int32_t HAL_GetSPIAutoDroppedCount(HAL_SPIPort port, int32_t* status) {
  hal::vmx::SPIAutoResult result = hal::vmx::SPIAutoResult::kHardwareFailure;
  const auto count = hal::vmx::GetSPIAutoManager().GetDropped(port, result);
  hal::vmx::SetSPIAutoResult(result, status,
                             "VMX SPI Auto dropped-count read failed",
                             static_cast<int32_t>(port));
  return count;
}

void HAL_ConfigureSPIAutoStall(HAL_SPIPort port, int32_t csToSclkTicks,
                               int32_t stallTicks, int32_t pow2BytesPerRead,
                               int32_t* status) {
  static_cast<void>(csToSclkTicks);
  static_cast<void>(stallTicks);
  static_cast<void>(pow2BytesPerRead);
  hal::vmx::SetSPIAutoResult(
      hal::vmx::GetSPIAutoManager().ConfigureStall(port), status,
      "VMX SPI Auto stall timing is unsupported by the SDK",
      static_cast<int32_t>(port));
}

}  // extern "C"
