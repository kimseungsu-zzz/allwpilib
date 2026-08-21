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
  kAddressableLED,
  // Studica vendor resources.  Cobra shares the physical I2C bus with the
  // standard WPILib I2C aliases; LightTower owns its output pins exclusively.
  kCobra,
  kLightTower,
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

  // Reserve one of the two physical I2C pins for a resource that can share
  // the bus with the compatible owner.  Generic DIO/PWM/SPI/UART resources
  // still conflict normally.  Keeping the reference counts in the central
  // registry prevents one shared user from releasing the other user's
  // reservation.
  DigitalChannelReservation ReserveShared(
      int32_t channel, DigitalChannelOwner owner,
      std::string_view allocationLocation,
      DigitalChannelOwner compatibleOwner) {
    std::scoped_lock lock{m_mutex};
    if (channel < 0 || channel >= kNumPhysicalChannels ||
        !IsI2CSharedOwner(owner) || !IsI2CSharedOwner(compatibleOwner) ||
        owner == compatibleOwner) {
      return {};
    }

    auto& entry = m_entries[channel];
    if (entry.owner == DigitalChannelOwner::kNone) {
      entry.owner = owner;
      entry.allocationLocation = allocationLocation;
      SetSharedReference(entry, owner, 1);
      return {true, DigitalChannelOwner::kNone, {}};
    }

    if (entry.owner != owner && entry.owner != compatibleOwner) {
      return {false, entry.owner, entry.allocationLocation};
    }
    // A legacy direct Reserve() may have populated the owner without a
    // shared count.  Treat that owner as one active reference before adding
    // the compatible resource.
    EnsureSharedReference(entry, entry.owner);
    AddSharedReference(entry, owner);
    return {true, entry.owner, entry.allocationLocation};
  }

  void ReleaseShared(int32_t channel, DigitalChannelOwner owner) noexcept {
    std::scoped_lock lock{m_mutex};
    if (channel < 0 || channel >= kNumPhysicalChannels ||
        !IsI2CSharedOwner(owner)) {
      return;
    }
    auto& entry = m_entries[channel];
    if (GetSharedReference(entry, owner) == 0) {
      return;
    }
    SetSharedReference(entry, owner, GetSharedReference(entry, owner) - 1);
    if (entry.i2cReferences == 0 && entry.cobraReferences == 0) {
      entry = {};
    } else if (entry.owner == owner && GetSharedReference(entry, owner) == 0) {
      entry.owner = owner == DigitalChannelOwner::kI2C
                        ? DigitalChannelOwner::kCobra
                        : DigitalChannelOwner::kI2C;
      entry.allocationLocation = entry.owner == DigitalChannelOwner::kI2C
                                     ? "VMX I2C shared bus"
                                     : "Studica Cobra shared bus";
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
    uint32_t i2cReferences = 0;
    uint32_t cobraReferences = 0;
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

  static constexpr bool IsI2CSharedOwner(DigitalChannelOwner owner) noexcept {
    return owner == DigitalChannelOwner::kI2C ||
           owner == DigitalChannelOwner::kCobra;
  }

  static uint32_t GetSharedReference(
      const Entry& entry, DigitalChannelOwner owner) noexcept {
    return owner == DigitalChannelOwner::kI2C ? entry.i2cReferences
                                              : entry.cobraReferences;
  }

  static void SetSharedReference(Entry& entry, DigitalChannelOwner owner,
                                 uint32_t value) noexcept {
    if (owner == DigitalChannelOwner::kI2C) {
      entry.i2cReferences = value;
    } else if (owner == DigitalChannelOwner::kCobra) {
      entry.cobraReferences = value;
    }
  }

  static void EnsureSharedReference(Entry& entry,
                                    DigitalChannelOwner owner) noexcept {
    if (GetSharedReference(entry, owner) == 0) {
      SetSharedReference(entry, owner, 1);
    }
  }

  static void AddSharedReference(Entry& entry,
                                 DigitalChannelOwner owner) noexcept {
    auto references = GetSharedReference(entry, owner);
    SetSharedReference(entry, owner, references + 1);
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
