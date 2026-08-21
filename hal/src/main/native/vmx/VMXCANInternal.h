// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "hal/CAN.h"
#include "hal/CANAPITypes.h"
#include "hal/Errors.h"
#include "hal/Types.h"

namespace hal::vmx {

struct VMXCANMessage {
  uint32_t messageID = 0;
  std::array<uint8_t, 8> data{};
  uint8_t dataSize = 0;
};

struct VMXCANFrame final : VMXCANMessage {
  uint32_t timeStampMS = 0;
  uint64_t sysTimeStampUS = 0;
  uint64_t generation = 0;
};

struct VMXCANBusStatus final {
  float percentBusUtilization = 0.0F;
  uint32_t busOffCount = 0;
  uint32_t txFullCount = 0;
  uint32_t receiveErrorCount = 0;
  uint32_t transmitErrorCount = 0;
  bool busWarning = false;
  bool busPassiveError = false;
  bool busOffError = false;
  bool hwRxOverflow = false;
  bool swRxOverflow = false;
  bool busError = false;
  bool wake = false;
  bool messageError = false;
};

struct VMXCANBackend final {
  using Send = std::function<bool(const VMXCANMessage&, int32_t, int32_t&)>;
  using Open = std::function<bool(uint32_t&, uint32_t, uint32_t, uint32_t,
                                 int32_t&)>;
  using Read = std::function<bool(uint32_t, VMXCANFrame*, uint32_t, uint32_t&,
                                 int32_t&)>;
  using Close = std::function<bool(uint32_t, int32_t&)>;
  using Status = std::function<bool(VMXCANBusStatus&, int32_t&)>;

  Send send;
  Open open;
  Read read;
  Close close;
  Status status;
};

inline bool VMXCANIsValidMessageID(uint32_t messageID) noexcept {
  constexpr uint32_t kFlags = HAL_CAN_IS_FRAME_REMOTE | HAL_CAN_IS_FRAME_11BIT;
  if ((messageID & ~(kFlags | 0x1FFFFFFFU)) != 0) {
    return false;
  }
  if ((messageID & HAL_CAN_IS_FRAME_11BIT) != 0) {
    return (messageID & 0x1FFFFFFFU) <= 0x7FFU;
  }
  return true;
}

inline bool VMXCANMatches(uint32_t messageID, uint32_t requestedID,
                          uint32_t mask) noexcept {
  return (messageID & mask) == (requestedID & mask);
}

/**
 * Owns the one VMX SDK receive retrieval stream and exposes software-owned
 * raw stream sessions.  The SDK has a small mask/filter budget, so public HAL
 * streams are deliberately multiplexed over this single retrieval stream.
 */
class VMXCANReceiveManager final {
 public:
  explicit VMXCANReceiveManager(VMXCANBackend backend = {},
                                std::function<uint64_t()> clock = {})
      : m_backend{std::move(backend)}, m_clock{std::move(clock)} {
    if (!m_clock) {
      m_clock = [] {
        return static_cast<uint64_t>(std::chrono::duration_cast<
                                         std::chrono::microseconds>(
                                         std::chrono::steady_clock::now()
                                             .time_since_epoch())
                                         .count());
      };
    }
  }

  VMXCANReceiveManager(const VMXCANReceiveManager&) = delete;
  VMXCANReceiveManager& operator=(const VMXCANReceiveManager&) = delete;

  ~VMXCANReceiveManager() { Shutdown(); }

  bool Start() noexcept {
    std::unique_lock lock{m_mutex};
    if (m_started) {
      return true;
    }
    m_started = true;
    m_stopRequested = false;
    if (!m_backend.open) {
      return true;
    }
    int32_t status = HAL_SUCCESS;
    try {
      if (!m_backend.open(m_sdkStream, 0, 0, kSdkStreamDepth, status)) {
        m_started = false;
        m_lastError = status;
        return false;
      }
    } catch (...) {
      m_started = false;
      m_lastError = INCOMPATIBLE_STATE;
      return false;
    }
    try {
      m_worker = std::thread{[this] { Run(); }};
    } catch (...) {
      if (m_backend.close) {
        int32_t ignored = HAL_SUCCESS;
        m_backend.close(m_sdkStream, ignored);
      }
      m_started = false;
      m_lastError = HAL_ERR_CANSessionMux_NotInitialized;
      return false;
    }
    return true;
  }

  void Shutdown() noexcept {
    std::thread worker;
    uint32_t sdkStream = 0;
    bool closeSdkStream = false;
    {
      std::scoped_lock lock{m_mutex};
      if (!m_started && !m_worker.joinable()) {
        return;
      }
      m_stopRequested = true;
      m_started = false;
      worker = std::move(m_worker);
      sdkStream = m_sdkStream;
      closeSdkStream = static_cast<bool>(m_backend.close);
    }
    m_cv.notify_all();
    if (worker.joinable()) {
      worker.join();
    }
    if (closeSdkStream) {
      int32_t ignored = HAL_SUCCESS;
      m_backend.close(sdkStream, ignored);
    }
    std::scoped_lock lock{m_mutex};
    m_streams.clear();
    m_frames.clear();
  }

  bool IsStarted() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_started;
  }

  int32_t LastError() const noexcept {
    std::scoped_lock lock{m_mutex};
    return m_lastError;
  }

  uint64_t NowMicros() const noexcept {
    try {
      return m_clock();
    } catch (...) {
      return 0;
    }
  }

  bool Send(const VMXCANMessage& message, int32_t periodMs,
            int32_t& status) noexcept {
    if (!VMXCANIsValidMessageID(message.messageID)) {
      status = PARAMETER_OUT_OF_RANGE;
      return false;
    }
    if (message.dataSize > message.data.size()) {
      status = PARAMETER_OUT_OF_RANGE;
      return false;
    }
    if (!m_backend.send) {
      status = INCOMPATIBLE_STATE;
      return false;
    }
    try {
      return m_backend.send(message, periodMs, status);
    } catch (...) {
      status = INCOMPATIBLE_STATE;
      return false;
    }
  }

  bool ReceiveLatest(uint32_t messageID, uint32_t mask, VMXCANFrame& frame,
                     int32_t& status) const noexcept {
    std::scoped_lock lock{m_mutex};
    for (auto it = m_frames.rbegin(); it != m_frames.rend(); ++it) {
      if (VMXCANMatches(it->messageID, messageID, mask)) {
        frame = *it;
        status = HAL_SUCCESS;
        return true;
      }
    }
    status = HAL_ERR_CANSessionMux_MessageNotFound;
    return false;
  }

  bool OpenStream(uint32_t& handle, uint32_t messageID, uint32_t mask,
                  uint32_t maxMessages, int32_t& status) noexcept {
    if (!VMXCANIsValidMessageID(messageID) || maxMessages == 0 ||
        maxMessages > kMaxPublicStreamDepth) {
      status = PARAMETER_OUT_OF_RANGE;
      handle = HAL_kInvalidHandle;
      return false;
    }
    auto stream = std::make_shared<Stream>();
    stream->messageID = messageID;
    stream->mask = mask;
    stream->maxMessages = maxMessages;
    try {
      std::scoped_lock lock{m_mutex};
      if (m_nextHandle == std::numeric_limits<uint32_t>::max()) {
        status = NO_AVAILABLE_RESOURCES;
        handle = HAL_kInvalidHandle;
        return false;
      }
      handle = ++m_nextHandle;
      if (handle == HAL_kInvalidHandle) {
        handle = ++m_nextHandle;
      }
      m_streams.emplace(handle, std::move(stream));
      status = HAL_SUCCESS;
      return true;
    } catch (...) {
      handle = HAL_kInvalidHandle;
      status = NO_AVAILABLE_RESOURCES;
      return false;
    }
  }

  bool CloseStream(uint32_t handle) noexcept {
    std::scoped_lock lock{m_mutex};
    return m_streams.erase(handle) != 0;
  }

  bool ReadStream(uint32_t handle, VMXCANFrame* messages,
                  uint32_t messagesToRead, uint32_t& messagesRead,
                  int32_t& status) noexcept {
    messagesRead = 0;
    if (messagesToRead != 0 && messages == nullptr) {
      status = PARAMETER_OUT_OF_RANGE;
      return false;
    }
    std::scoped_lock lock{m_mutex};
    auto it = m_streams.find(handle);
    if (it == m_streams.end()) {
      status = HAL_HANDLE_ERROR;
      return false;
    }
    auto& stream = *it->second;
    if (stream.overflowed) {
      stream.overflowed = false;
      status = HAL_CAN_BUFFER_OVERRUN;
      return false;
    }
    while (messagesRead < messagesToRead && !stream.queue.empty()) {
      messages[messagesRead++] = stream.queue.front();
      stream.queue.pop_front();
    }
    status = HAL_SUCCESS;
    return true;
  }

  bool GetStatus(VMXCANBusStatus& value, int32_t& status) const noexcept {
    if (!m_backend.status) {
      value = {};
      status = HAL_SUCCESS;
      return true;
    }
    try {
      return m_backend.status(value, status);
    } catch (...) {
      value = {};
      status = INCOMPATIBLE_STATE;
      return false;
    }
  }

  /** Host-test and SDK-independent injection seam. */
  void InjectFrame(VMXCANFrame frame) noexcept {
    try {
      if (frame.dataSize > frame.data.size()) {
        return;
      }
      if (frame.sysTimeStampUS == 0) {
        frame.sysTimeStampUS = m_clock();
      }
      if (frame.timeStampMS == 0) {
        frame.timeStampMS = static_cast<uint32_t>(frame.sysTimeStampUS / 1000);
      }
      std::scoped_lock lock{m_mutex};
      frame.generation = ++m_generation;
      m_frames.push_back(frame);
      while (m_frames.size() > kHistoryDepth) {
        m_frames.pop_front();
      }
      for (auto& [unused, stream] : m_streams) {
        if (!VMXCANMatches(frame.messageID, stream->messageID, stream->mask)) {
          continue;
        }
        if (stream->queue.size() >= stream->maxMessages) {
          stream->queue.pop_front();
          stream->overflowed = true;
        }
        stream->queue.push_back(frame);
      }
      m_cv.notify_all();
    } catch (...) {
      // A received packet must never throw through the SDK callback/thread.
    }
  }

 private:
  struct Stream final {
    uint32_t messageID = 0;
    uint32_t mask = 0;
    uint32_t maxMessages = 0;
    bool overflowed = false;
    std::deque<VMXCANFrame> queue;
  };

  static constexpr uint32_t kSdkStreamDepth = 1024;
  static constexpr uint32_t kMaxPublicStreamDepth = 1'000'000;
  static constexpr size_t kHistoryDepth = 4096;

  void Run() noexcept {
    std::array<VMXCANFrame, 64> frames{};
    while (true) {
      {
        std::unique_lock lock{m_mutex};
        if (m_stopRequested) {
          return;
        }
      }
      uint32_t read = 0;
      int32_t status = HAL_SUCCESS;
      bool ok = false;
      try {
        ok = m_backend.read &&
             m_backend.read(m_sdkStream, frames.data(), frames.size(), read,
                            status);
      } catch (...) {
        status = INCOMPATIBLE_STATE;
      }
      if (ok) {
        for (uint32_t i = 0; i < std::min<uint32_t>(read, frames.size()); ++i) {
          InjectFrame(frames[i]);
        }
        if (read != 0) {
          continue;
        }
      }
      std::unique_lock lock{m_mutex};
      if (m_stopRequested) {
        return;
      }
      m_cv.wait_for(lock, std::chrono::milliseconds{2});
      if (!ok && status != HAL_SUCCESS) {
        m_lastError = status;
      }
    }
  }

  VMXCANBackend m_backend;
  std::function<uint64_t()> m_clock;
  mutable std::mutex m_mutex;
  std::condition_variable m_cv;
  std::map<uint32_t, std::shared_ptr<Stream>> m_streams;
  std::deque<VMXCANFrame> m_frames;
  std::thread m_worker;
  uint32_t m_nextHandle = 0;
  uint32_t m_sdkStream = 0;
  uint64_t m_generation = 0;
  int32_t m_lastError = HAL_SUCCESS;
  bool m_started = false;
  bool m_stopRequested = false;
};

inline bool VMXCANIsValidManufacturer(HAL_CANManufacturer manufacturer) {
  return static_cast<int32_t>(manufacturer) >= HAL_CAN_Man_kBroadcast &&
         static_cast<int32_t>(manufacturer) <= HAL_CAN_Man_kBrushlandLabs;
}

inline bool VMXCANIsValidDeviceType(HAL_CANDeviceType deviceType) {
  const auto value = static_cast<int32_t>(deviceType);
  return (value >= HAL_CAN_Dev_kBroadcast && value <= HAL_CAN_Dev_ColorSensor) ||
         value == HAL_CAN_Dev_kFirmwareUpdate;
}

inline uint32_t VMXCreateCANId(HAL_CANManufacturer manufacturer,
                               uint8_t deviceId, HAL_CANDeviceType deviceType,
                               int32_t apiId) noexcept {
  return (static_cast<uint32_t>(deviceType) & 0x1FU) << 24 |
         (static_cast<uint32_t>(manufacturer) & 0xFFU) << 16 |
         (static_cast<uint32_t>(apiId) & 0x3FFU) << 6 | deviceId;
}

struct VMXCANReceiveCache final {
  VMXCANFrame frame{};
  uint64_t consumedGeneration = 0;
  bool present = false;
};

struct VMXCANDevice final {
  HAL_CANManufacturer manufacturer = HAL_CAN_Man_kBroadcast;
  HAL_CANDeviceType deviceType = HAL_CAN_Dev_kBroadcast;
  uint8_t deviceId = 0;
  std::mutex mutex;
  std::map<int32_t, int32_t> periodicSends;
  std::map<int32_t, VMXCANReceiveCache> receives;
};

/** WPILib-owned logical CAN device handle table. */
class VMXCANApiState final {
 public:
  HAL_CANHandle Initialize(HAL_CANManufacturer manufacturer, int32_t deviceId,
                           HAL_CANDeviceType deviceType, int32_t& status) {
    if (!VMXCANIsValidManufacturer(manufacturer) || deviceId < 0 ||
        deviceId > 63 || !VMXCANIsValidDeviceType(deviceType)) {
      status = PARAMETER_OUT_OF_RANGE;
      return HAL_kInvalidHandle;
    }
    try {
      auto device = std::make_shared<VMXCANDevice>();
      device->manufacturer = manufacturer;
      device->deviceType = deviceType;
      device->deviceId = static_cast<uint8_t>(deviceId);
      std::scoped_lock lock{m_mutex};
      if (m_nextHandle == std::numeric_limits<HAL_CANHandle>::max()) {
        status = NO_AVAILABLE_RESOURCES;
        return HAL_kInvalidHandle;
      }
      const auto handle = ++m_nextHandle;
      m_devices.emplace(handle, std::move(device));
      status = HAL_SUCCESS;
      return handle;
    } catch (...) {
      status = NO_AVAILABLE_RESOURCES;
      return HAL_kInvalidHandle;
    }
  }

  std::shared_ptr<VMXCANDevice> Get(HAL_CANHandle handle) const {
    std::scoped_lock lock{m_mutex};
    auto it = m_devices.find(handle);
    return it == m_devices.end() ? nullptr : it->second;
  }

  std::shared_ptr<VMXCANDevice> Clean(HAL_CANHandle handle) {
    std::scoped_lock lock{m_mutex};
    auto it = m_devices.find(handle);
    if (it == m_devices.end()) {
      return nullptr;
    }
    auto result = std::move(it->second);
    m_devices.erase(it);
    return result;
  }

  void Clear() {
    std::scoped_lock lock{m_mutex};
    m_devices.clear();
  }

 private:
  mutable std::mutex m_mutex;
  std::map<HAL_CANHandle, std::shared_ptr<VMXCANDevice>> m_devices;
  HAL_CANHandle m_nextHandle = 0;
};

inline bool VMXCANReadApiPacket(VMXCANReceiveManager& manager,
                                VMXCANDevice& device, int32_t apiId,
                                uint8_t* data, int32_t* length,
                                uint64_t* receivedTimestamp, int32_t timeoutMs,
                                int32_t mode, int32_t& status) {
  if (apiId < 0 || apiId > 1023 || !length || !receivedTimestamp ||
      *length < 0 || *length > 8 ||
      (data == nullptr && *length > 0) || (mode == 2 && timeoutMs < 0)) {
    status = PARAMETER_OUT_OF_RANGE;
    return false;
  }
  const auto messageID = VMXCreateCANId(device.manufacturer, device.deviceId,
                                        device.deviceType, apiId);
  VMXCANFrame frame;
  int32_t managerStatus = HAL_SUCCESS;
  const bool fresh =
      manager.ReceiveLatest(messageID, 0x1FFFFFFFU, frame, managerStatus);
  std::scoped_lock lock{device.mutex};
  auto& cache = device.receives[apiId];
  if (fresh) {
    cache.frame = frame;
    cache.present = true;
  }
  if (mode == 0 &&
      (!cache.present || cache.frame.generation == cache.consumedGeneration)) {
    status = HAL_ERR_CANSessionMux_MessageNotFound;
    return false;
  }
  if (!cache.present) {
    status = managerStatus;
    return false;
  }
  if (mode == 2) {
    const auto nowMs = static_cast<uint32_t>(manager.NowMicros() / 1000);
    const auto age = static_cast<uint32_t>(nowMs - cache.frame.timeStampMS);
    if (age > static_cast<uint32_t>(timeoutMs)) {
      status = HAL_CAN_TIMEOUT;
      return false;
    }
  }
  const auto copyLength = std::min<int32_t>(*length, cache.frame.dataSize);
  if (copyLength < cache.frame.dataSize) {
    status = PARAMETER_OUT_OF_RANGE;
    return false;
  }
  if (copyLength != 0) {
    std::memcpy(data, cache.frame.data.data(), copyLength);
  }
  *length = cache.frame.dataSize;
  *receivedTimestamp = cache.frame.timeStampMS;
  if (mode == 0) {
    cache.consumedGeneration = cache.frame.generation;
  }
  status = HAL_SUCCESS;
  return true;
}

bool InitializeCAN() noexcept;
void ShutdownCAN() noexcept;
VMXCANReceiveManager& GetCANReceiveManager() noexcept;
VMXCANApiState& GetCANApiState() noexcept;

}  // namespace hal::vmx
