// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <mutex>
#include <string>
#include <string_view>

#include "VMXConstants.h"

namespace hal::vmx {

enum class DigitalChannelOwner {
  kNone,
  kDIO,
  kPWM,
  kEncoder,
  kCounter,
  kI2C,
  kSPI,
  kUART,
  kDutyCycle,
};

struct DigitalChannelReservation {
  bool reserved = false;
  DigitalChannelOwner previousOwner = DigitalChannelOwner::kNone;
  std::string previousAllocation;
};

struct FlexTimerReservation {
  bool reserved = false;
  DigitalChannelOwner previousOwner = DigitalChannelOwner::kNone;
  std::string previousAllocation;
};

class DigitalChannelRegistry final {
 public:
  DigitalChannelReservation Reserve(int32_t channel, DigitalChannelOwner owner,
                                    std::string_view allocationLocation) {
    std::scoped_lock lock{m_mutex};
    if (channel < 0 || channel >= kNumPhysicalChannels) {
      return {};
    }

    auto& entry = m_entries[channel];
    if (entry.owner != DigitalChannelOwner::kNone) {
      return {false, entry.owner, entry.allocationLocation};
    }
    entry.owner = owner;
    entry.allocationLocation = allocationLocation;
    return {true, DigitalChannelOwner::kNone, {}};
  }

  void Release(int32_t channel, DigitalChannelOwner owner) noexcept {
    std::scoped_lock lock{m_mutex};
    if (channel < 0 || channel >= kNumPhysicalChannels) {
      return;
    }
    auto& entry = m_entries[channel];
    if (entry.owner == owner) {
      entry = {};
    }
  }

  bool Transfer(int32_t channel, DigitalChannelOwner expectedOwner,
                DigitalChannelOwner newOwner,
                std::string_view allocationLocation) {
    std::scoped_lock lock{m_mutex};
    if (channel < 0 || channel >= kNumPhysicalChannels) {
      return false;
    }
    auto& entry = m_entries[channel];
    if (entry.owner != expectedOwner) {
      return false;
    }
    entry.owner = newOwner;
    entry.allocationLocation = allocationLocation;
    return true;
  }

  bool TransferPair(int32_t channelA, int32_t channelB,
                    DigitalChannelOwner expectedOwner,
                    DigitalChannelOwner newOwner,
                    std::string_view allocationLocation) {
    std::scoped_lock lock{m_mutex};
    if (channelA < 0 || channelA >= kNumPhysicalChannels || channelB < 0 ||
        channelB >= kNumPhysicalChannels || channelA == channelB ||
        m_entries[channelA].owner != expectedOwner ||
        m_entries[channelB].owner != expectedOwner) {
      return false;
    }
    m_entries[channelA] = {newOwner, std::string{allocationLocation}};
    m_entries[channelB] = {newOwner, std::string{allocationLocation}};
    return true;
  }

  FlexTimerReservation ReserveFlexTimerGroup(
      int32_t physicalChannel, DigitalChannelOwner owner,
      std::string_view allocationLocation) {
    std::scoped_lock lock{m_mutex};
    int32_t group = FlexTimerGroup(physicalChannel);
    if (group < 0) {
      return {false, DigitalChannelOwner::kNone, {}};
    }
    auto& entry = m_timerEntries[group];
    if (entry.owner != DigitalChannelOwner::kNone) {
      return {false, entry.owner, entry.allocationLocation};
    }
    entry.owner = owner;
    entry.allocationLocation = allocationLocation;
    return {true, DigitalChannelOwner::kNone, {}};
  }

  void ReleaseFlexTimerGroup(int32_t physicalChannel,
                             DigitalChannelOwner owner) noexcept {
    std::scoped_lock lock{m_mutex};
    int32_t group = FlexTimerGroup(physicalChannel);
    if (group < 0) {
      return;
    }
    auto& entry = m_timerEntries[group];
    if (entry.owner == owner) {
      entry = {};
    }
  }

 private:
  struct Entry {
    DigitalChannelOwner owner = DigitalChannelOwner::kNone;
    std::string allocationLocation;
  };

  struct TimerEntry {
    DigitalChannelOwner owner = DigitalChannelOwner::kNone;
    std::string allocationLocation;
  };

  static constexpr int32_t FlexTimerGroup(int32_t physicalChannel) noexcept {
    return physicalChannel >= 0 && physicalChannel < 12
               ? physicalChannel / 2
               : -1;
  }

  std::mutex m_mutex;
  std::array<Entry, kNumPhysicalChannels> m_entries;
  std::array<TimerEntry, 6> m_timerEntries;
};

inline DigitalChannelRegistry& GetDigitalChannelRegistry() {
  static DigitalChannelRegistry registry;
  return registry;
}

}  // namespace hal::vmx
