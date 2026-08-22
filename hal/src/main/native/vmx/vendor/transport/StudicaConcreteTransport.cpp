// Copyright (c) 2026 WPILib contributors.

#include "StudicaConcreteTransport.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "VMXPi.h"

#include "../../VMXRuntime.h"
#include "VMXErrors.h"

namespace studica::vendor {
namespace {

speed_t ToBaud(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    case 115200:
    default:
      return B115200;
  }
}

TransportResult Result(TransportStatus status, std::size_t bytes = 0) {
  return {status, bytes};
}

int PollFd(int fd, short events, int timeoutMs) {
  pollfd pfd{fd, events, 0};
  return ::poll(&pfd, 1, std::max(timeoutMs, 0));
}

}  // namespace

StudicaCANTransport::StudicaCANTransport(std::shared_ptr<VMXPi> context,
                                         uint32_t filterId,
                                         uint32_t filterMask,
                                         uint32_t bufferSize)
    : m_context{std::move(context)} {
  if (!m_context || !m_context->IsOpen()) return;
  VMXErrorCode error = 0;
  if (m_context->getCAN().OpenReceiveStream(m_stream, filterId, filterMask,
                                            bufferSize, &error)) {
    m_context->getCAN().EnableReceiveStreamBlackboard(m_stream, true, &error);
    m_open = true;
  }
}

StudicaCANTransport::~StudicaCANTransport() { Close(); }

bool StudicaCANTransport::IsOpen() const noexcept {
  std::scoped_lock lock{m_mutex};
  return m_open && m_context && m_context->IsOpen();
}

TransportResult StudicaCANTransport::Write(uint32_t address,
                                           const uint8_t* data,
                                           std::size_t length,
                                           int timeoutMs) noexcept {
  std::scoped_lock lock{m_mutex};
  if (!m_open || !m_context || (length != 0 && !data) || length > 64) {
    return Result((length != 0 && !data) || length > 64
                      ? TransportStatus::kInvalidArgument
                                       : TransportStatus::kDisconnected);
  }
  VMXCANMessage message{};
  message.messageID = address;
  message.dataSize = static_cast<int>(length);
  if (length != 0) {
    message.setData(const_cast<uint8_t*>(data), static_cast<int>(length));
  }
  VMXErrorCode error = 0;
  const bool ok = m_context->getCAN().SendMessage(message, timeoutMs, &error);
  return Result(ok ? TransportStatus::kOk : TransportStatus::kUnavailable,
                ok ? length : 0);
}

TransportResult StudicaCANTransport::Read(uint32_t address, uint8_t* data,
                                          std::size_t capacity,
                                          int /*timeoutMs*/) noexcept {
  std::scoped_lock lock{m_mutex};
  if (!m_open || !m_context) return Result(TransportStatus::kDisconnected);
  if (!data || capacity == 0) return Result(TransportStatus::kInvalidArgument);
  VMXCANTimestampedMessage message{};
  uint64_t timestamp = 0;
  bool alreadyRetrieved = false;
  VMXErrorCode error = 0;
  const bool found =
      m_context->getCAN().GetBlackboardEntry(m_stream, address, message,
                                             timestamp, alreadyRetrieved,
                                             &error);
  if (!found) return Result(TransportStatus::kTimeout);
  const std::size_t count = std::min<std::size_t>(capacity, message.dataSize);
  std::memcpy(data, message.data, count);
  return Result(TransportStatus::kOk, count);
}

void StudicaCANTransport::Close() noexcept {
  std::scoped_lock lock{m_mutex};
  if (m_open && m_context) {
    VMXErrorCode error = 0;
    m_context->getCAN().CloseReceiveStream(m_stream, &error);
  }
  m_open = false;
  m_stream = 0;
}

StudicaUSBSerialTransport::StudicaUSBSerialTransport(std::string path,
                                                     int baud)
    : m_path{std::move(path)} {
  if (m_path.empty()) return;
  m_fd = ::open(m_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (m_fd < 0) return;
  termios tty{};
  if (tcgetattr(m_fd, &tty) != 0) {
    Close();
    return;
  }
  cfmakeraw(&tty);
  const auto speed = ToBaud(baud);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(m_fd, TCSANOW, &tty) != 0) Close();
}

StudicaUSBSerialTransport::~StudicaUSBSerialTransport() { Close(); }

bool StudicaUSBSerialTransport::IsOpen() const noexcept {
  std::scoped_lock lock{m_mutex};
  return m_fd >= 0;
}

TransportResult StudicaUSBSerialTransport::Write(uint32_t /*address*/,
                                                 const uint8_t* data,
                                                 std::size_t length,
                                                 int timeoutMs) noexcept {
  std::scoped_lock lock{m_mutex};
  if (m_fd < 0) return Result(TransportStatus::kDisconnected);
  if (!data && length != 0) return Result(TransportStatus::kInvalidArgument);
  const auto disconnect = [this]() noexcept {
    if (m_fd >= 0) ::close(m_fd);
    m_fd = -1;
  };
  std::size_t written = 0;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(std::max(timeoutMs, 0));
  while (written < length) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    if (remaining.count() < 0 || PollFd(m_fd, POLLOUT,
                                        static_cast<int>(remaining.count())) <=
                                    0) {
      return Result(TransportStatus::kTimeout, written);
    }
    const auto count = ::write(m_fd, data + written, length - written);
    if (count > 0) {
      written += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
      continue;
    disconnect();
    return Result(TransportStatus::kDisconnected, written);
  }
  return Result(TransportStatus::kOk, written);
}

TransportResult StudicaUSBSerialTransport::Read(uint32_t /*address*/,
                                                uint8_t* data,
                                                std::size_t capacity,
                                                int timeoutMs) noexcept {
  std::scoped_lock lock{m_mutex};
  if (m_fd < 0) return Result(TransportStatus::kDisconnected);
  if (!data || capacity == 0) return Result(TransportStatus::kInvalidArgument);
  const auto disconnect = [this]() noexcept {
    if (m_fd >= 0) ::close(m_fd);
    m_fd = -1;
  };
  const int pollResult = PollFd(m_fd, POLLIN, timeoutMs);
  if (pollResult == 0) return Result(TransportStatus::kTimeout);
  if (pollResult < 0) {
    if (errno == EINTR) return Result(TransportStatus::kTimeout);
    disconnect();
    return Result(TransportStatus::kDisconnected);
  }
  const auto count = ::read(m_fd, data, capacity);
  if (count > 0) return Result(TransportStatus::kOk, count);
  if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
    return Result(TransportStatus::kTimeout);
  disconnect();
  return Result(TransportStatus::kDisconnected);
}

void StudicaUSBSerialTransport::Close() noexcept {
  std::scoped_lock lock{m_mutex};
  if (m_fd >= 0) {
    ::close(m_fd);
    m_fd = -1;
  }
}

}  // namespace studica::vendor
