// Copyright (c) 2026 WPILib contributors.
// Open Source Software; you may modify it under the terms of the WPILib
// BSD license file in the root directory of this project.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

struct VMXPi;

namespace studica {

enum class TransportKind : uint32_t {
  kCAN = 1,
  kUSB = 2,
};

enum class TransportStatus : int32_t {
  kOk = 0,
  kInvalidArgument = -22,
  kUnavailable = -19,
  kTimeout = -110,
  kDisconnected = -107,
  kResourceConflict = -16,
  kUnsupported = -95,
  kInternalError = -5,
};

struct TransportResult {
  TransportStatus status = TransportStatus::kUnavailable;
  std::size_t bytes = 0;

  explicit operator bool() const noexcept {
    return status == TransportStatus::kOk;
  }
};

/**
 * Shared transport contract for Studica vendor devices.
 *
 * Vendor protocols remain in the pinned Studica drivers.  This interface is
 * the project-owned lifecycle/ownership seam used by adapters and host tests;
 * it is deliberately independent of WPILib HAL and of any C++ SDK ABI.
 */
class StudicaVendorTransport {
 public:
  virtual ~StudicaVendorTransport() = default;

  virtual TransportKind Kind() const noexcept = 0;
  virtual bool IsOpen() const noexcept = 0;
  virtual TransportResult Write(uint32_t address, const uint8_t* data,
                                std::size_t length,
                                int timeoutMs) noexcept = 0;
  virtual TransportResult Read(uint32_t address, uint8_t* data,
                               std::size_t capacity,
                               int timeoutMs) noexcept = 0;

  virtual TransportResult Transaction(uint32_t address, const uint8_t* tx,
                                      std::size_t txLength, uint8_t* rx,
                                      std::size_t rxCapacity,
                                      int timeoutMs) noexcept {
    auto writeResult = Write(address, tx, txLength, timeoutMs);
    if (!writeResult) return writeResult;
    return Read(address, rx, rxCapacity, timeoutMs);
  }

  virtual void Close() noexcept = 0;
};

/**
 * Process-local ownership registry for vendor transports.
 *
 * CAN is shared as a bus, so distinct device IDs may coexist while the same
 * (channel, ID) pair is exclusive. USB paths are exclusive after normalizing
 * the user-provided path; /dev/serial/by-id paths are retained verbatim so
 * they remain stable across re-enumeration.
 */
class StudicaTransportRegistry final {
 public:
  bool ReserveCAN(uint32_t channel, uint32_t deviceId, std::string owner) {
    std::scoped_lock lock{m_mutex};
    const auto key = std::to_string(channel) + ":" + std::to_string(deviceId);
    if (m_canOwners.contains(key)) return false;
    m_canOwners.emplace(key, std::move(owner));
    return true;
  }

  void ReleaseCAN(uint32_t channel, uint32_t deviceId) noexcept {
    std::scoped_lock lock{m_mutex};
    m_canOwners.erase(std::to_string(channel) + ":" +
                      std::to_string(deviceId));
  }

  bool ReserveUSB(const std::string& path, std::string owner) {
    if (path.empty()) return false;
    std::scoped_lock lock{m_mutex};
    if (m_usbOwners.contains(path)) return false;
    m_usbOwners.emplace(path, std::move(owner));
    return true;
  }

  void ReleaseUSB(const std::string& path) noexcept {
    std::scoped_lock lock{m_mutex};
    m_usbOwners.erase(path);
  }

 private:
  std::mutex m_mutex;
  std::unordered_map<std::string, std::string> m_canOwners;
  std::unordered_map<std::string, std::string> m_usbOwners;
};

inline StudicaTransportRegistry& GetStudicaTransportRegistry() {
  static StudicaTransportRegistry registry;
  return registry;
}

}  // namespace studica
