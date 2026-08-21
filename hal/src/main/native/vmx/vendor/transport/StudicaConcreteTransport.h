// Copyright (c) 2026 WPILib contributors.

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "studica/Transport.h"

namespace studica::vendor {

/** Raw VMX CAN transport backed by the shared VMXRuntime VMXPi instance. */
class StudicaCANTransport final : public StudicaVendorTransport {
 public:
  StudicaCANTransport(std::shared_ptr<VMXPi> context, uint32_t filterId,
                      uint32_t filterMask, uint32_t bufferSize = 64);
  ~StudicaCANTransport() override;

  TransportKind Kind() const noexcept override { return TransportKind::kCAN; }
  bool IsOpen() const noexcept override;
  TransportResult Write(uint32_t address, const uint8_t* data,
                        std::size_t length, int timeoutMs) noexcept override;
  TransportResult Read(uint32_t address, uint8_t* data, std::size_t capacity,
                       int timeoutMs) noexcept override;
  void Close() noexcept override;

 private:
  std::shared_ptr<VMXPi> m_context;
  uint32_t m_stream = 0;
  bool m_open = false;
  mutable std::mutex m_mutex;
};

/** Linux USB CDC transport.  It owns one fd and serializes close/read/write. */
class StudicaUSBSerialTransport final : public StudicaVendorTransport {
 public:
  StudicaUSBSerialTransport(std::string path, int baud = 115200);
  ~StudicaUSBSerialTransport() override;

  TransportKind Kind() const noexcept override { return TransportKind::kUSB; }
  bool IsOpen() const noexcept override;
  TransportResult Write(uint32_t address, const uint8_t* data,
                        std::size_t length, int timeoutMs) noexcept override;
  TransportResult Read(uint32_t address, uint8_t* data, std::size_t capacity,
                       int timeoutMs) noexcept override;
  void Close() noexcept override;

  const std::string& Path() const noexcept { return m_path; }

 private:
  std::string m_path;
  int m_fd = -1;
  mutable std::mutex m_mutex;
};

}  // namespace studica::vendor
