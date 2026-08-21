// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <type_traits>

#include "hal/DriverStationTypes.h"
#include "hal/Errors.h"

namespace wpi {
struct EventVector;
}

namespace hal::vmx {

// The VMX transport follows the KauaiLabs VMX-pi Driver Station protocol
// (UDP v1, robot port 1110; TCP metadata port 1740).  This packet is a
// transport-independent DTO used by the parser, receiver, and host tests.
struct VMXDriverStationPacket {
  uint16_t wireSequence = 0;
  HAL_ControlWord controlWord{};
  HAL_AllianceStationID alliance = HAL_AllianceStationID_kUnknown;
  std::array<HAL_JoystickAxes, HAL_kMaxJoysticks> axes{};
  std::array<HAL_JoystickPOVs, HAL_kMaxJoysticks> povs{};
  std::array<HAL_JoystickButtons, HAL_kMaxJoysticks> buttons{};
  std::array<HAL_JoystickDescriptor, HAL_kMaxJoysticks> descriptors{};
  HAL_MatchInfo matchInfo{};
  double matchTime = -1.0;
  bool hasMatchTime = false;
  bool valid = false;

  VMXDriverStationPacket() { Reset(); }

  void Reset() {
    std::memset(this, 0, sizeof(*this));
    alliance = HAL_AllianceStationID_kUnknown;
    matchTime = -1.0;
    for (auto& descriptor : descriptors) {
      descriptor.type = 0;
    }
    matchInfo.matchType = HAL_kMatchType_none;
  }
};

class VMXDriverStationPacketParser final {
 public:
  // Decode the documented KauaiLabs UDP v1 packet.  Unknown tagged payloads
  // are ignored only after their declared bounds have been validated.
  static bool ParseUdp(const uint8_t* data, size_t length,
                       VMXDriverStationPacket& packet) noexcept {
    if (!data || length < 6 || data[2] != 1) {
      return false;
    }

    VMXDriverStationPacket decoded;
    decoded.wireSequence = static_cast<uint16_t>(data[0] << 8 | data[1]);
    const uint8_t control = data[3];
    decoded.controlWord.enabled = (control & 0x04) != 0;
    decoded.controlWord.autonomous = (control & 0x02) != 0;
    decoded.controlWord.test = (control & 0x01) != 0;
    decoded.controlWord.eStop = (control & 0x80) != 0;
    decoded.controlWord.fmsAttached = (control & 0x08) != 0;
    decoded.controlWord.dsAttached = true;
    decoded.alliance = DecodeAlliance(data[5]);

    size_t offset = 6;
    int joystick = 0;
    while (offset < length) {
      if (length - offset < 2) {
        return false;
      }
      const size_t structLength = data[offset];
      const uint8_t tag = data[offset + 1];
      if (structLength < 1 || structLength > length - offset - 1) {
        return false;
      }
      const uint8_t* payload = data + offset + 2;
      const size_t payloadLength = structLength - 1;
      if (tag == 12) {  // DS_UDP_PROTOCOL_TAG_JOYSTICK_STATE
        if (joystick >= HAL_kMaxJoysticks ||
            !ParseJoystick(payload, payloadLength, decoded.axes[joystick],
                            decoded.povs[joystick], decoded.buttons[joystick])) {
          return false;
        }
        ++joystick;
      } else if (tag == 7) {  // DS_UDP_PROTOCOL_MATCH_TIME_COUNTDOWN
        if (payloadLength != 4) {
          return false;
        }
        uint32_t bits = (static_cast<uint32_t>(payload[0]) << 24) |
                        (static_cast<uint32_t>(payload[1]) << 16) |
                        (static_cast<uint32_t>(payload[2]) << 8) |
                        payload[3];
        float value = 0.0F;
        std::memcpy(&value, &bits, sizeof(value));
        if (!std::isfinite(value)) {
          return false;
        }
        decoded.matchTime = value;
        decoded.hasMatchTime = true;
      }
      offset += structLength + 1;
    }
    decoded.valid = true;
    packet = decoded;
    return true;
  }

  // Decode the KauaiLabs TCP metadata stream.  A TCP read may contain one or
  // more length-prefixed records; incomplete records are rejected so callers
  // can retain bytes until the next read.
  static bool ParseTcp(const uint8_t* data, size_t length,
                       VMXDriverStationPacket& packet) noexcept {
    if (!data && length != 0) {
      return false;
    }
    VMXDriverStationPacket decoded = packet;
    size_t offset = 0;
    while (offset < length) {
      if (length - offset < 2) {
        return false;
      }
      const size_t recordLength =
          static_cast<size_t>(data[offset] << 8 | data[offset + 1]);
      if (recordLength < 1 || recordLength > length - offset - 2) {
        return false;
      }
      const uint8_t tag = data[offset + 2];
      const uint8_t* payload = data + offset + 3;
      const size_t payloadLength = recordLength - 1;
      switch (tag) {
        case 2:  // DS_TCP_PROTOCOL_TAG_JOYSTICK_DESCRIPTIONS
          if (!ParseDescriptors(payload, payloadLength, decoded.descriptors)) {
            return false;
          }
          break;
        case 7:  // DS_TCP_PROTOCOL_TAG_NEWER_MATCH_INFO
          if (!ParseMatchInfo(payload, payloadLength, decoded.matchInfo)) {
            return false;
          }
          break;
        case 14:  // DS_TCP_PROTOCOL_TAG_GAME_SPECIFIC_MSG
          if (payloadLength > sizeof(decoded.matchInfo.gameSpecificMessage)) {
            return false;
          }
          decoded.matchInfo.gameSpecificMessageSize =
              static_cast<uint16_t>(payloadLength);
          std::memcpy(decoded.matchInfo.gameSpecificMessage, payload,
                      payloadLength);
          break;
        default:
          break;
      }
      offset += recordLength + 2;
    }
    packet = decoded;
    return true;
  }

 private:
  static HAL_AllianceStationID DecodeAlliance(uint8_t value) noexcept {
    return value <= HAL_AllianceStationID_kBlue3
               ? static_cast<HAL_AllianceStationID>(value)
               : HAL_AllianceStationID_kUnknown;
  }

  static bool ParseJoystick(const uint8_t* data, size_t length,
                            HAL_JoystickAxes& axes, HAL_JoystickPOVs& povs,
                            HAL_JoystickButtons& buttons) noexcept {
    if (!data || length < 1) {
      return false;
    }
    size_t offset = 0;
    const uint8_t axisCount = data[offset++];
    if (axisCount > HAL_kMaxJoystickAxes || length - offset < axisCount) {
      return false;
    }
    axes.count = axisCount;
    for (uint8_t i = 0; i < axisCount; ++i) {
      const auto raw = static_cast<int8_t>(data[offset++]);
      axes.raw[i] = static_cast<uint8_t>(raw);
      axes.axes[i] = std::clamp(static_cast<float>(raw) / 127.0F, -1.0F,
                                1.0F);
    }
    if (length - offset < 1) {
      return false;
    }
    const uint8_t buttonCount = data[offset++];
    if (buttonCount > 32) {
      return false;
    }
    const size_t buttonBytes = (buttonCount + 7) / 8;
    if (length - offset < buttonBytes + 1) {
      return false;
    }
    buttons.count = buttonCount;
    buttons.buttons = 0;
    for (size_t i = 0; i < buttonBytes; ++i) {
      buttons.buttons |= static_cast<uint32_t>(data[offset++]) << (i * 8);
    }
    const uint8_t povCount = data[offset++];
    if (povCount > HAL_kMaxJoystickPOVs || length - offset < povCount * 2) {
      return false;
    }
    povs.count = povCount;
    for (uint8_t i = 0; i < povCount; ++i) {
      povs.povs[i] = static_cast<int16_t>(
          static_cast<uint16_t>(data[offset] << 8 | data[offset + 1]));
      offset += 2;
    }
    return offset == length;
  }

  static bool ParseDescriptors(
      const uint8_t* data, size_t length,
      std::array<HAL_JoystickDescriptor, HAL_kMaxJoysticks>& descriptors) {
    size_t offset = 0;
    while (offset < length) {
      if (length - offset < 4) {
        return false;
      }
      const uint8_t joystick = data[offset++];
      const uint8_t xbox = data[offset++];
      const uint8_t type = data[offset++];
      const uint8_t nameLength = data[offset++];
      if (joystick >= HAL_kMaxJoysticks ||
          length - offset < static_cast<size_t>(nameLength) + 3U) {
        return false;
      }
      HAL_JoystickDescriptor descriptor{};
      descriptor.isXbox = xbox != 0;
      descriptor.type = type;
      const size_t copiedName =
          std::min<size_t>(nameLength, sizeof(descriptor.name) - 1);
      std::memcpy(descriptor.name, data + offset, copiedName);
      descriptor.name[copiedName] = '\0';
      offset += nameLength;
      descriptor.axisCount = data[offset++];
      if (descriptor.axisCount > HAL_kMaxJoystickAxes ||
          length - offset < static_cast<size_t>(descriptor.axisCount) + 2U) {
        return false;
      }
      std::memcpy(descriptor.axisTypes, data + offset, descriptor.axisCount);
      offset += descriptor.axisCount;
      descriptor.buttonCount = data[offset++];
      descriptor.povCount = data[offset++];
      descriptors[joystick] = descriptor;
    }
    return true;
  }

  static bool ParseMatchInfo(const uint8_t* data, size_t length,
                             HAL_MatchInfo& info) {
    if (!data || length < 1) {
      return false;
    }
    size_t offset = 0;
    const uint8_t eventLength = data[offset++];
    if (eventLength >= sizeof(info.eventName) ||
        length - offset < static_cast<size_t>(eventLength) + 4U) {
      return false;
    }
    std::memcpy(info.eventName, data + offset, eventLength);
    info.eventName[eventLength] = '\0';
    offset += eventLength;
    const uint8_t matchType = data[offset++];
    if (matchType > HAL_kMatchType_elimination) {
      return false;
    }
    info.matchType = static_cast<HAL_MatchType>(matchType);
    info.matchNumber = static_cast<uint16_t>(data[offset] << 8 | data[offset + 1]);
    offset += 2;
    info.replayNumber = data[offset];
    return true;
  }
};

class VMXDriverStationState final {
 public:
  using Clock = std::function<uint64_t()>;
  using Wakeup = std::function<void()>;
  using Output = std::function<bool(int32_t, int64_t, int32_t, int32_t)>;
  using Program = std::function<void(int32_t)>;

  explicit VMXDriverStationState(
      Clock clock = [] {
        return static_cast<uint64_t>(std::chrono::duration_cast<
                                         std::chrono::microseconds>(
                                         std::chrono::steady_clock::now()
                                             .time_since_epoch())
                                         .count());
      },
      Wakeup wakeup = {})
      : m_clock{std::move(clock)}, m_wakeup{std::move(wakeup)} {
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

  void SetOutput(Output output) {
    std::scoped_lock lock{m_mutex};
    m_output = std::move(output);
  }

  void SetProgramCallback(Program program) {
    std::scoped_lock lock{m_mutex};
    m_program = std::move(program);
  }

  void CommitUdp(const VMXDriverStationPacket& packet, uint64_t now = 0) {
    if (!packet.valid) {
      return;
    }
    if (now == 0) {
      now = m_clock();
    }
    {
      std::scoped_lock lock{m_mutex};
      if (m_shutdown) {
        return;
      }
      for (size_t i = 0; i < HAL_kMaxJoysticks; ++i) {
        m_packet.axes[i] = packet.axes[i];
        m_packet.povs[i] = packet.povs[i];
        m_packet.buttons[i] = packet.buttons[i];
      }
      const bool hasDescriptors = std::any_of(
          packet.descriptors.begin(), packet.descriptors.end(),
          [](const HAL_JoystickDescriptor& descriptor) {
            return descriptor.name[0] != '\0' || descriptor.axisCount != 0 ||
                   descriptor.buttonCount != 0 || descriptor.povCount != 0;
          });
      if (hasDescriptors) {
        m_packet.descriptors = packet.descriptors;
      }
      MergeMatchInfoLocked(packet.matchInfo);
      m_packet.controlWord = packet.controlWord;
      m_packet.alliance = packet.alliance;
      if (packet.hasMatchTime) {
        m_packet.matchTime = packet.matchTime;
        m_packet.hasMatchTime = true;
      }
      m_packet.wireSequence = packet.wireSequence;
      m_packet.valid = true;
      m_lastPacket = now;
      m_fresh = true;
      ++m_generation;
    }
    WakeupIfNeeded();
  }

  void CommitTcp(const VMXDriverStationPacket& packet) {
    {
      std::scoped_lock lock{m_mutex};
      if (m_shutdown) {
        return;
      }
      const bool hasDescriptors = std::any_of(
          packet.descriptors.begin(), packet.descriptors.end(),
          [](const HAL_JoystickDescriptor& descriptor) {
            return descriptor.name[0] != '\0' || descriptor.axisCount != 0 ||
                   descriptor.buttonCount != 0 || descriptor.povCount != 0;
          });
      if (hasDescriptors) {
        m_packet.descriptors = packet.descriptors;
      }
      MergeMatchInfoLocked(packet.matchInfo);
      ++m_generation;
    }
    WakeupIfNeeded();
  }

  void MarkInvalidPacket() {
    bool changed = false;
    {
      std::scoped_lock lock{m_mutex};
      if (!m_shutdown && m_fresh) {
        FailSafeLocked();
        ++m_generation;
        changed = true;
      }
    }
    if (changed) {
      WakeupIfNeeded();
    }
  }

  void Disconnect() { MarkInvalidPacket(); }

  bool PollTimeout(uint64_t now = 0) {
    if (now == 0) {
      now = m_clock();
    }
    bool changed = false;
    {
      std::scoped_lock lock{m_mutex};
      if (!m_shutdown && m_fresh && now >= m_lastPacket &&
          now - m_lastPacket > m_timeoutMicros) {
        FailSafeLocked();
        ++m_generation;
        changed = true;
      }
    }
    if (changed) {
      WakeupIfNeeded();
    }
    return changed;
  }

  void Shutdown() {
    {
      std::scoped_lock lock{m_mutex};
      if (m_shutdown) {
        return;
      }
      m_shutdown = true;
      FailSafeLocked();
      ++m_generation;
    }
    WakeupIfNeeded();
  }

  void SetTimeoutMicros(uint64_t timeout) {
    std::scoped_lock lock{m_mutex};
    m_timeoutMicros = timeout;
  }

  bool Refresh() {
    PollTimeout();
    std::scoped_lock lock{m_mutex};
    const bool changed = m_generation != m_lastRefreshGeneration;
    m_lastRefreshGeneration = m_generation;
    return changed && !m_shutdown;
  }

  int32_t GetControlWord(HAL_ControlWord* word) {
    if (!word) {
      return PARAMETER_OUT_OF_RANGE;
    }
    std::scoped_lock lock{m_mutex};
    *word = EffectiveControlWordLocked();
    return m_shutdown ? INCOMPATIBLE_STATE : HAL_SUCCESS;
  }

  HAL_AllianceStationID GetAlliance(int32_t* status) {
    std::scoped_lock lock{m_mutex};
    if (status) {
      *status = m_shutdown ? INCOMPATIBLE_STATE : HAL_SUCCESS;
    }
    return m_fresh && !m_shutdown ? m_packet.alliance
                                  : HAL_AllianceStationID_kUnknown;
  }

  int32_t GetAxes(int32_t joystick, HAL_JoystickAxes* axes) {
    HAL_JoystickAxes zero{};
    return GetJoystick(joystick, axes, &zero);
  }

  int32_t GetPOVs(int32_t joystick, HAL_JoystickPOVs* povs) {
    HAL_JoystickPOVs zero{};
    return GetJoystick(joystick, povs, &zero);
  }

  int32_t GetButtons(int32_t joystick, HAL_JoystickButtons* buttons) {
    HAL_JoystickButtons zero{};
    return GetJoystick(joystick, buttons, &zero);
  }

  void GetAll(HAL_JoystickAxes* axes, HAL_JoystickPOVs* povs,
              HAL_JoystickButtons* buttons) {
    std::scoped_lock lock{m_mutex};
    for (size_t i = 0; i < HAL_kMaxJoysticks; ++i) {
      axes[i] = m_fresh && !m_shutdown ? m_packet.axes[i] : HAL_JoystickAxes{};
      povs[i] = m_fresh && !m_shutdown ? m_packet.povs[i] : HAL_JoystickPOVs{};
      buttons[i] = m_fresh && !m_shutdown ? m_packet.buttons[i]
                                          : HAL_JoystickButtons{};
    }
  }

  int32_t GetDescriptor(int32_t joystick, HAL_JoystickDescriptor* descriptor) {
    if (!descriptor) {
      return PARAMETER_OUT_OF_RANGE;
    }
    if (!IsJoystickValid(joystick)) {
      *descriptor = {};
      return PARAMETER_OUT_OF_RANGE;
    }
    std::scoped_lock lock{m_mutex};
    *descriptor = m_fresh && !m_shutdown ? m_packet.descriptors[joystick]
                                         : HAL_JoystickDescriptor{};
    return m_shutdown ? INCOMPATIBLE_STATE : HAL_SUCCESS;
  }

  int32_t GetAxisType(int32_t joystick, int32_t axis) {
    HAL_JoystickDescriptor descriptor{};
    const int32_t status = GetDescriptor(joystick, &descriptor);
    if (status != HAL_SUCCESS) {
      return status;
    }
    if (axis < 0 || axis >= descriptor.axisCount || axis >= HAL_kMaxJoystickAxes) {
      return PARAMETER_OUT_OF_RANGE;
    }
    return descriptor.axisTypes[axis];
  }

  int32_t SetJoystickOutputs(int32_t joystick, int64_t outputs, int32_t left,
                             int32_t right) {
    if (!IsJoystickValid(joystick)) {
      return PARAMETER_OUT_OF_RANGE;
    }
    Output output;
    {
      std::scoped_lock lock{m_mutex};
      if (m_shutdown || !m_output) {
        return INCOMPATIBLE_STATE;
      }
      output = m_output;
    }
    return output(joystick, outputs, left, right) ? HAL_SUCCESS
                                                   : INCOMPATIBLE_STATE;
  }

  void ObserveProgram(int32_t mode) {
    Program program;
    {
      std::scoped_lock lock{m_mutex};
      m_programMode = mode;
      program = m_program;
    }
    if (program) {
      program(mode);
    }
  }

  int32_t GetJoystickName(int32_t joystick, const char** name, size_t* length) {
    if (!name || !length) {
      return PARAMETER_OUT_OF_RANGE;
    }
    if (!IsJoystickValid(joystick)) {
      *name = "";
      *length = 0;
      return PARAMETER_OUT_OF_RANGE;
    }
    std::scoped_lock lock{m_mutex};
    *name = m_packet.descriptors[joystick].name;
    *length = m_fresh && !m_shutdown ? std::strlen(*name) : 0;
    return m_shutdown ? INCOMPATIBLE_STATE : HAL_SUCCESS;
  }

  double GetMatchTime(int32_t* status) {
    std::scoped_lock lock{m_mutex};
    if (status) {
      *status = m_shutdown ? INCOMPATIBLE_STATE : HAL_SUCCESS;
    }
    return m_fresh && !m_shutdown && m_packet.hasMatchTime ? m_packet.matchTime
                                                            : -1.0;
  }

  int32_t GetMatchInfo(HAL_MatchInfo* info) {
    if (!info) {
      return PARAMETER_OUT_OF_RANGE;
    }
    std::scoped_lock lock{m_mutex};
    *info = m_fresh && !m_shutdown ? m_packet.matchInfo : HAL_MatchInfo{};
    return m_shutdown ? INCOMPATIBLE_STATE : HAL_SUCCESS;
  }

  bool OutputsEnabled() {
    std::scoped_lock lock{m_mutex};
    const auto word = EffectiveControlWordLocked();
    return word.enabled && word.dsAttached && !word.eStop;
  }

 private:
  template <typename T>
  int32_t GetJoystick(int32_t joystick, T* value, const T* zero) {
    if (!value) {
      return PARAMETER_OUT_OF_RANGE;
    }
    if (!IsJoystickValid(joystick)) {
      *value = *zero;
      return PARAMETER_OUT_OF_RANGE;
    }
    std::scoped_lock lock{m_mutex};
    if (m_shutdown) {
      *value = *zero;
      return INCOMPATIBLE_STATE;
    }
    if constexpr (std::is_same_v<T, HAL_JoystickAxes>) {
      *value = m_fresh ? m_packet.axes[joystick] : *zero;
    } else if constexpr (std::is_same_v<T, HAL_JoystickPOVs>) {
      *value = m_fresh ? m_packet.povs[joystick] : *zero;
    } else {
      *value = m_fresh ? m_packet.buttons[joystick] : *zero;
    }
    return HAL_SUCCESS;
  }

  static bool IsJoystickValid(int32_t joystick) {
    return joystick >= 0 && joystick < HAL_kMaxJoysticks;
  }

  HAL_ControlWord EffectiveControlWordLocked() const {
    HAL_ControlWord word{};
    if (m_fresh && !m_shutdown) {
      word = m_packet.controlWord;
      if (word.eStop) {
        word.enabled = false;
      }
    }
    return word;
  }

  void FailSafeLocked() {
    m_fresh = false;
    m_packet.controlWord = {};
    m_packet.controlWord.eStop = false;
    m_packet.alliance = HAL_AllianceStationID_kUnknown;
    m_packet.matchTime = -1.0;
    m_packet.hasMatchTime = false;
    m_packet.axes = {};
    m_packet.povs = {};
    m_packet.buttons = {};
  }

  void MergeMatchInfoLocked(const HAL_MatchInfo& incoming) {
    const bool hasIdentity = incoming.eventName[0] != '\0' ||
                             incoming.matchType != HAL_kMatchType_none ||
                             incoming.matchNumber != 0;
    const uint16_t previousSize = static_cast<uint16_t>(std::min<size_t>(
        m_packet.matchInfo.gameSpecificMessageSize,
        sizeof(m_packet.matchInfo.gameSpecificMessage)));
    std::array<uint8_t, sizeof(m_packet.matchInfo.gameSpecificMessage)>
        previousMessage{};
    std::memcpy(previousMessage.data(), m_packet.matchInfo.gameSpecificMessage,
                previousSize);
    if (hasIdentity) {
      m_packet.matchInfo = incoming;
      std::memcpy(m_packet.matchInfo.gameSpecificMessage, previousMessage.data(),
                  previousSize);
      m_packet.matchInfo.gameSpecificMessageSize = previousSize;
    }
    if (incoming.gameSpecificMessageSize != 0) {
      const uint16_t gameSize = static_cast<uint16_t>(std::min<size_t>(
          incoming.gameSpecificMessageSize,
          sizeof(m_packet.matchInfo.gameSpecificMessage)));
      m_packet.matchInfo.gameSpecificMessageSize = gameSize;
      std::memcpy(m_packet.matchInfo.gameSpecificMessage,
                  incoming.gameSpecificMessage,
                  gameSize);
    }
  }

  void WakeupIfNeeded() {
    if (m_wakeup) {
      m_wakeup();
    }
  }

  mutable std::mutex m_mutex;
  Clock m_clock;
  Wakeup m_wakeup;
  Output m_output;
  Program m_program;
  VMXDriverStationPacket m_packet;
  uint64_t m_lastPacket = 0;
  uint64_t m_timeoutMicros = 2'000'000;
  uint64_t m_generation = 0;
  uint64_t m_lastRefreshGeneration = 0;
  int32_t m_programMode = 0;
  bool m_fresh = false;
  bool m_shutdown = false;
};

bool InitializeDriverStation() noexcept;
void ShutdownDriverStation() noexcept;
VMXDriverStationState& GetDriverStationState() noexcept;
wpi::EventVector& GetDriverStationEvents() noexcept;

}  // namespace hal::vmx
