// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VMXCANInternal.h"

#include <array>
#include <cstring>
#include <memory>
#include <mutex>
#include <vector>

#include "HALInitializer.h"
#include "VMXErrors.h"
#include "VMXPi.h"
#include "VMXRuntime.h"
#include "hal/CANAPI.h"

namespace hal::vmx {
namespace {

int32_t MapSdkStatus(int32_t status) noexcept {
  if (status == VMXERR_CAN_HW_RX_OVERFLOW) {
    return HAL_CAN_BUFFER_OVERRUN;
  }
  return status == HAL_SUCCESS ? INCOMPATIBLE_STATE : INCOMPATIBLE_STATE;
}

class VMXCANSdkAdapter final {
 public:
  explicit VMXCANSdkAdapter(std::shared_ptr<VMXPi> context)
      : m_context{std::move(context)} {}

  bool ResetToFrcBitrate(int32_t& status) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_context) {
      status = INCOMPATIBLE_STATE;
      return false;
    }
    VMXErrorCode error = 0;
    bool ok = m_context->getCAN().ResetBusBitrate(
        VMXCAN::CAN_BUS_BITRATE_1MBPS, &error);
    if (ok) {
      ok = m_context->getCAN().SetMode(VMXCAN::VMXCAN_NORMAL, &error);
    }
    status = ok ? HAL_SUCCESS : MapSdkStatus(error);
    return ok;
  }

  bool Send(const VMXCANMessage& message, int32_t periodMs,
            int32_t& status) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_context) {
      status = INCOMPATIBLE_STATE;
      return false;
    }
    ::VMXCANMessage sdkMessage{};
    sdkMessage.messageID = message.messageID;
    sdkMessage.dataSize = message.dataSize;
    if (message.dataSize != 0) {
      std::memcpy(sdkMessage.data, message.data.data(), message.dataSize);
    }
    VMXErrorCode error = 0;
    const bool ok = m_context->getCAN().SendMessage(sdkMessage, periodMs, &error);
    status = ok ? HAL_SUCCESS : MapSdkStatus(error);
    return ok;
  }

  bool Open(uint32_t& handle, uint32_t messageID, uint32_t mask,
            uint32_t maxMessages, int32_t& status) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_context) {
      status = INCOMPATIBLE_STATE;
      return false;
    }
    VMXErrorCode error = 0;
    const bool ok = m_context->getCAN().OpenReceiveStream(
        handle, messageID, mask, maxMessages, &error);
    status = ok ? HAL_SUCCESS : MapSdkStatus(error);
    return ok;
  }

  bool Read(uint32_t handle, VMXCANFrame* messages, uint32_t messagesToRead,
            uint32_t& messagesRead, int32_t& status) noexcept {
    std::scoped_lock lock{m_mutex};
    messagesRead = 0;
    if (!m_context) {
      status = INCOMPATIBLE_STATE;
      return false;
    }
    if (messagesToRead > kMaxReadBatch) {
      status = PARAMETER_OUT_OF_RANGE;
      return false;
    }
    std::vector<::VMXCANTimestampedMessage> sdkMessages;
    try {
      sdkMessages.resize(messagesToRead);
    } catch (...) {
      status = NO_AVAILABLE_RESOURCES;
      return false;
    }
    const auto count = messagesToRead;
    VMXErrorCode error = 0;
    const bool ok = m_context->getCAN().ReadReceiveStream(
        handle, sdkMessages.data(), count, messagesRead, &error);
    if (ok) {
      messagesRead = std::min<uint32_t>(messagesRead, count);
      for (uint32_t i = 0; i < messagesRead; ++i) {
        messages[i].messageID = sdkMessages[i].messageID;
        if (sdkMessages[i].dataSize > sizeof(messages[i].data)) {
          status = PARAMETER_OUT_OF_RANGE;
          messagesRead = 0;
          return false;
        }
        messages[i].dataSize = sdkMessages[i].dataSize;
        std::memcpy(messages[i].data.data(), sdkMessages[i].data,
                    sdkMessages[i].dataSize);
        messages[i].timeStampMS = sdkMessages[i].timeStampMS;
        messages[i].sysTimeStampUS = sdkMessages[i].sysTimeStampUS;
      }
    }
    status = ok ? HAL_SUCCESS : MapSdkStatus(error);
    return ok;
  }

  bool Close(uint32_t handle, int32_t& status) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_context) {
      status = INCOMPATIBLE_STATE;
      return false;
    }
    VMXErrorCode error = 0;
    const bool ok = m_context->getCAN().CloseReceiveStream(handle, &error);
    status = ok ? HAL_SUCCESS : MapSdkStatus(error);
    return ok;
  }

  bool GetStatus(VMXCANBusStatus& value, int32_t& status) noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_context) {
      value = {};
      status = INCOMPATIBLE_STATE;
      return false;
    }
    ::VMXCANBusStatus sdkStatus{};
    VMXErrorCode error = 0;
    const bool ok = m_context->getCAN().GetCANBUSStatus(sdkStatus, &error);
    if (ok) {
      value.percentBusUtilization = sdkStatus.percentBusUtilization;
      value.busOffCount = sdkStatus.busOffCount;
      value.txFullCount = sdkStatus.txFullCount;
      value.receiveErrorCount = sdkStatus.receiveErrorCount;
      value.transmitErrorCount = sdkStatus.transmitErrorCount;
      value.busWarning = sdkStatus.busWarning;
      value.busPassiveError = sdkStatus.busPassiveError;
      value.busOffError = sdkStatus.busOffError;
      value.hwRxOverflow = sdkStatus.hwRxOverflow;
      value.swRxOverflow = sdkStatus.swRxOverflow;
      value.busError = sdkStatus.busError;
      value.wake = sdkStatus.wake;
      value.messageError = sdkStatus.messageError;
    } else {
      value = {};
    }
    status = ok ? HAL_SUCCESS : MapSdkStatus(error);
    return ok;
  }

  uint64_t NowMicros() const noexcept {
    std::scoped_lock lock{m_mutex};
    if (!m_context) {
      return 0;
    }
    try {
      return m_context->getTime().GetCurrentTotalMicroseconds();
    } catch (...) {
      return 0;
    }
  }

 private:
  static constexpr uint32_t kMaxReadBatch = 1'000'000;
  std::shared_ptr<VMXPi> m_context;
  mutable std::mutex m_mutex;
};

class VMXCANRuntime final {
 public:
  bool Initialize() noexcept {
    std::scoped_lock lock{m_mutex};
    if (m_manager) {
      return true;
    }
    auto context = GetRuntimeContext();
    if (!context) {
      m_lastError = INCOMPATIBLE_STATE;
      return false;
    }
    try {
      m_sdk = std::make_shared<VMXCANSdkAdapter>(std::move(context));
      int32_t status = HAL_SUCCESS;
      if (!m_sdk->ResetToFrcBitrate(status)) {
        m_lastError = status;
        m_sdk.reset();
        return false;
      }
      VMXCANBackend backend;
      backend.send = [sdk = m_sdk](const VMXCANMessage& message, int32_t period,
                                   int32_t& status) {
        return sdk->Send(message, period, status);
      };
      backend.open = [sdk = m_sdk](uint32_t& handle, uint32_t id, uint32_t mask,
                                   uint32_t max, int32_t& status) {
        return sdk->Open(handle, id, mask, max, status);
      };
      backend.read = [sdk = m_sdk](uint32_t handle, VMXCANFrame* messages,
                                   uint32_t count, uint32_t& read,
                                   int32_t& status) {
        return sdk->Read(handle, messages, count, read, status);
      };
      backend.close = [sdk = m_sdk](uint32_t handle, int32_t& status) {
        return sdk->Close(handle, status);
      };
      backend.status = [sdk = m_sdk](VMXCANBusStatus& value, int32_t& status) {
        return sdk->GetStatus(value, status);
      };
      m_manager = std::make_unique<VMXCANReceiveManager>(
          std::move(backend), [sdk = m_sdk] { return sdk->NowMicros(); });
      if (!m_manager->Start()) {
        m_lastError = m_manager->LastError();
        m_manager.reset();
        m_sdk.reset();
        return false;
      }
      m_lastError = HAL_SUCCESS;
      return true;
    } catch (...) {
      m_lastError = INCOMPATIBLE_STATE;
      m_manager.reset();
      m_sdk.reset();
      return false;
    }
  }

  void Shutdown() noexcept {
    std::unique_ptr<VMXCANReceiveManager> manager;
    {
      std::scoped_lock lock{m_mutex};
      manager = std::move(m_manager);
      m_sdk.reset();
    }
    if (manager) {
      manager->Shutdown();
    }
    m_api.Clear();
  }

  VMXCANReceiveManager* Manager() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_manager.get();
  }

  VMXCANApiState& Api() noexcept { return m_api; }

 private:
  mutable std::mutex m_mutex;
  std::shared_ptr<VMXCANSdkAdapter> m_sdk;
  std::unique_ptr<VMXCANReceiveManager> m_manager;
  VMXCANApiState m_api;
  int32_t m_lastError = HAL_SUCCESS;
};

VMXCANRuntime& GetRuntime() {
  static VMXCANRuntime runtime;
  return runtime;
}

VMXCANReceiveManager& FallbackManager() {
  static VMXCANReceiveManager manager;
  return manager;
}

}  // namespace

bool InitializeCAN() noexcept { return GetRuntime().Initialize(); }

void ShutdownCAN() noexcept { GetRuntime().Shutdown(); }

VMXCANReceiveManager& GetCANReceiveManager() noexcept {
  if (auto* manager = GetRuntime().Manager()) {
    return *manager;
  }
  return FallbackManager();
}

VMXCANApiState& GetCANApiState() noexcept { return GetRuntime().Api(); }

}  // namespace hal::vmx

extern "C" {

void HAL_CAN_SendMessage(uint32_t messageID, const uint8_t* data,
                         uint8_t dataSize, int32_t periodMs, int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  hal::init::CheckInit();
  if (dataSize > 8 || (dataSize != 0 && data == nullptr)) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  hal::vmx::VMXCANMessage message;
  message.messageID = messageID;
  message.dataSize = dataSize;
  if (dataSize != 0) {
    std::memcpy(message.data.data(), data, dataSize);
  }
  auto& manager = hal::vmx::GetCANReceiveManager();
  if (!manager.Send(message, periodMs, *status)) {
    return;
  }
}

void HAL_CAN_ReceiveMessage(uint32_t* messageID, uint32_t messageIDMask,
                            uint8_t* data, uint8_t* dataSize,
                            uint32_t* timeStamp, int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  hal::init::CheckInit();
  if (!messageID || !dataSize || !timeStamp || !data) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  hal::vmx::VMXCANFrame frame;
  if (!hal::vmx::GetCANReceiveManager().ReceiveLatest(
          *messageID, messageIDMask, frame, *status)) {
    return;
  }
  *messageID = frame.messageID;
  *dataSize = frame.dataSize;
  std::memcpy(data, frame.data.data(), frame.dataSize);
  *timeStamp = frame.timeStampMS;
  *status = HAL_SUCCESS;
}

void HAL_CAN_GetCANStatus(float* percentBusUtilization, uint32_t* busOffCount,
                          uint32_t* txFullCount, uint32_t* receiveErrorCount,
                          uint32_t* transmitErrorCount, int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  if (!percentBusUtilization || !busOffCount || !txFullCount ||
      !receiveErrorCount || !transmitErrorCount) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  hal::vmx::VMXCANBusStatus value;
  if (!hal::vmx::GetCANReceiveManager().GetStatus(value, *status)) {
    return;
  }
  *percentBusUtilization = value.percentBusUtilization;
  *busOffCount = value.busOffCount;
  *txFullCount = value.txFullCount;
  *receiveErrorCount = value.receiveErrorCount;
  *transmitErrorCount = value.transmitErrorCount;
}

void HAL_CAN_OpenStreamSession(uint32_t* sessionHandle, uint32_t messageID,
                               uint32_t messageIDMask, uint32_t maxMessages,
                               int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  if (!sessionHandle) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  *sessionHandle = HAL_kInvalidHandle;
  hal::init::CheckInit();
  hal::vmx::GetCANReceiveManager().OpenStream(
      *sessionHandle, messageID, messageIDMask, maxMessages, *status);
}

void HAL_CAN_CloseStreamSession(uint32_t sessionHandle) {
  hal::vmx::GetCANReceiveManager().CloseStream(sessionHandle);
}

void HAL_CAN_ReadStreamSession(uint32_t sessionHandle,
                               struct HAL_CANStreamMessage* messages,
                               uint32_t messagesToRead, uint32_t* messagesRead,
                               int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  if (!messagesRead || (messagesToRead != 0 && !messages)) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  *messagesRead = 0;
  if (messagesToRead > 1'000'000) {
    *status = PARAMETER_OUT_OF_RANGE;
    return;
  }
  std::vector<hal::vmx::VMXCANFrame> frames;
  try {
    frames.resize(messagesToRead);
  } catch (...) {
    *status = NO_AVAILABLE_RESOURCES;
    return;
  }
  const auto count = messagesToRead;
  if (!hal::vmx::GetCANReceiveManager().ReadStream(
          sessionHandle, frames.data(), count, *messagesRead, *status)) {
    return;
  }
  *messagesRead = std::min<uint32_t>(*messagesRead, count);
  for (uint32_t i = 0; i < *messagesRead; ++i) {
    if (frames[i].dataSize > sizeof(messages[i].data)) {
      *messagesRead = 0;
      *status = PARAMETER_OUT_OF_RANGE;
      return;
    }
  }
  for (uint32_t i = 0; i < *messagesRead; ++i) {
    messages[i].messageID = frames[i].messageID;
    messages[i].dataSize = frames[i].dataSize;
    std::memcpy(messages[i].data, frames[i].data.data(), frames[i].dataSize);
    messages[i].timeStamp = frames[i].timeStampMS;
  }
}

uint32_t HAL_GetCANPacketBaseTime(void) {
  hal::init::CheckInit();
  return static_cast<uint32_t>(hal::vmx::GetCANReceiveManager().NowMicros() /
                               1000);
}

HAL_CANHandle HAL_InitializeCAN(HAL_CANManufacturer manufacturer,
                                int32_t deviceId, HAL_CANDeviceType deviceType,
                                int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  hal::init::CheckInit();
  return hal::vmx::GetCANApiState().Initialize(manufacturer, deviceId,
                                                deviceType, *status);
}

void HAL_CleanCAN(HAL_CANHandle handle) {
  auto device = hal::vmx::GetCANApiState().Clean(handle);
  if (!device) {
    return;
  }
  auto& manager = hal::vmx::GetCANReceiveManager();
  std::scoped_lock lock{device->mutex};
  for (const auto& [apiId, period] : device->periodicSends) {
    if (period > 0) {
      hal::vmx::VMXCANMessage message;
      message.messageID = hal::vmx::VMXCreateCANId(
          device->manufacturer, device->deviceId, device->deviceType, apiId);
      int32_t ignored = HAL_SUCCESS;
      manager.Send(message, HAL_CAN_SEND_PERIOD_STOP_REPEATING, ignored);
    }
  }
}

void HAL_WriteCANPacket(HAL_CANHandle handle, const uint8_t* data,
                        int32_t length, int32_t apiId, int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  auto device = hal::vmx::GetCANApiState().Get(handle);
  if (!device || apiId < 0 || apiId > 1023 || length < 0 || length > 8 ||
      (length != 0 && data == nullptr)) {
    *status = device ? PARAMETER_OUT_OF_RANGE : HAL_HANDLE_ERROR;
    return;
  }
  hal::vmx::VMXCANMessage message;
  message.messageID = hal::vmx::VMXCreateCANId(
      device->manufacturer, device->deviceId, device->deviceType, apiId);
  message.dataSize = static_cast<uint8_t>(length);
  if (length != 0) {
    std::memcpy(message.data.data(), data, length);
  }
  if (hal::vmx::GetCANReceiveManager().Send(message,
                                             HAL_CAN_SEND_PERIOD_NO_REPEAT,
                                             *status)) {
    std::scoped_lock lock{device->mutex};
    device->periodicSends[apiId] = -1;
  }
}

void HAL_WriteCANPacketRepeating(HAL_CANHandle handle, const uint8_t* data,
                                 int32_t length, int32_t apiId,
                                 int32_t repeatMs, int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  auto device = hal::vmx::GetCANApiState().Get(handle);
  if (!device || apiId < 0 || apiId > 1023 || length < 0 || length > 8 ||
      repeatMs <= 0 || (length != 0 && data == nullptr)) {
    *status = device ? PARAMETER_OUT_OF_RANGE : HAL_HANDLE_ERROR;
    return;
  }
  hal::vmx::VMXCANMessage message;
  message.messageID = hal::vmx::VMXCreateCANId(
      device->manufacturer, device->deviceId, device->deviceType, apiId);
  message.dataSize = static_cast<uint8_t>(length);
  if (length != 0) {
    std::memcpy(message.data.data(), data, length);
  }
  if (hal::vmx::GetCANReceiveManager().Send(message, repeatMs, *status)) {
    std::scoped_lock lock{device->mutex};
    device->periodicSends[apiId] = repeatMs;
  }
}

void HAL_WriteCANRTRFrame(HAL_CANHandle handle, int32_t length, int32_t apiId,
                          int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  auto device = hal::vmx::GetCANApiState().Get(handle);
  if (!device || apiId < 0 || apiId > 1023 || length < 0 || length > 8) {
    *status = device ? PARAMETER_OUT_OF_RANGE : HAL_HANDLE_ERROR;
    return;
  }
  hal::vmx::VMXCANMessage message;
  message.messageID = hal::vmx::VMXCreateCANId(
                          device->manufacturer, device->deviceId,
                          device->deviceType, apiId) |
                      HAL_CAN_IS_FRAME_REMOTE;
  message.dataSize = static_cast<uint8_t>(length);
  if (hal::vmx::GetCANReceiveManager().Send(message,
                                             HAL_CAN_SEND_PERIOD_NO_REPEAT,
                                             *status)) {
    std::scoped_lock lock{device->mutex};
    device->periodicSends[apiId] = -1;
  }
}

void HAL_StopCANPacketRepeating(HAL_CANHandle handle, int32_t apiId,
                                int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  auto device = hal::vmx::GetCANApiState().Get(handle);
  if (!device || apiId < 0 || apiId > 1023) {
    *status = device ? PARAMETER_OUT_OF_RANGE : HAL_HANDLE_ERROR;
    return;
  }
  hal::vmx::VMXCANMessage message;
  message.messageID = hal::vmx::VMXCreateCANId(
      device->manufacturer, device->deviceId, device->deviceType, apiId);
  if (hal::vmx::GetCANReceiveManager().Send(
          message, HAL_CAN_SEND_PERIOD_STOP_REPEATING, *status)) {
    std::scoped_lock lock{device->mutex};
    device->periodicSends.erase(apiId);
  }
}

void HAL_ReadCANPacketNew(HAL_CANHandle handle, int32_t apiId, uint8_t* data,
                          int32_t* length, uint64_t* receivedTimestamp,
                          int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  auto device = hal::vmx::GetCANApiState().Get(handle);
  if (!device) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  hal::vmx::VMXCANReadApiPacket(hal::vmx::GetCANReceiveManager(), *device,
                                apiId, data, length, receivedTimestamp, 0, 0,
                                *status);
}

void HAL_ReadCANPacketLatest(HAL_CANHandle handle, int32_t apiId,
                             uint8_t* data, int32_t* length,
                             uint64_t* receivedTimestamp, int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  auto device = hal::vmx::GetCANApiState().Get(handle);
  if (!device) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  hal::vmx::VMXCANReadApiPacket(hal::vmx::GetCANReceiveManager(), *device,
                                apiId, data, length, receivedTimestamp, 0, 1,
                                *status);
}

void HAL_ReadCANPacketTimeout(HAL_CANHandle handle, int32_t apiId,
                              uint8_t* data, int32_t* length,
                              uint64_t* receivedTimestamp, int32_t timeoutMs,
                              int32_t* status) {
  int32_t localStatus = HAL_SUCCESS;
  if (!status) {
    status = &localStatus;
  }
  *status = HAL_SUCCESS;
  auto device = hal::vmx::GetCANApiState().Get(handle);
  if (!device) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  hal::vmx::VMXCANReadApiPacket(hal::vmx::GetCANReceiveManager(), *device,
                                apiId, data, length, receivedTimestamp,
                                timeoutMs, 2, *status);
}

}  // extern "C"
