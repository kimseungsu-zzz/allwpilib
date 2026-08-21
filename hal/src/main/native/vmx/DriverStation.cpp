// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriverStationInternal.h"

#include <array>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <wpi/EventVector.h>

#include "HALInitializer.h"
#include "hal/DriverStation.h"
#include "hal/Errors.h"
#include "VMXWatchdogInternal.h"

namespace hal::vmx {
namespace {

constexpr int kRobotUdpPort = 1110;
constexpr int kRobotTcpPort = 1740;
constexpr size_t kMaxDatagram = 4096;
constexpr size_t kMaxTcpBuffer = 8192;

class VMXDriverStationTransport final {
 public:
  VMXDriverStationTransport() = default;
  VMXDriverStationTransport(const VMXDriverStationTransport&) = delete;
  VMXDriverStationTransport& operator=(const VMXDriverStationTransport&) =
      delete;

  ~VMXDriverStationTransport() { Stop(); }

  void Start(VMXDriverStationState& state) {
    std::scoped_lock lock{m_mutex};
    if (m_running) {
      return;
    }
    m_state = &state;
    m_running = true;
    OpenSocketsLocked();
    m_thread = std::thread{[this] { Run(); }};
  }

  void Stop() noexcept {
    {
      std::scoped_lock lock{m_mutex};
      if (!m_running) {
        return;
      }
      m_running = false;
    }
    if (m_thread.joinable()) {
      m_thread.join();
    }
    std::scoped_lock lock{m_mutex};
    CloseSocketsLocked();
    m_state = nullptr;
  }

  bool SendJoystickOutputs(int32_t joystick, int64_t outputs, int32_t left,
                           int32_t right) {
    if (joystick < 0 || joystick >= HAL_kMaxJoysticks) {
      return false;
    }
    std::scoped_lock lock{m_mutex};
    if (m_udp < 0 || !m_haveDsAddress) {
      return false;
    }
    std::array<uint8_t, 24> packet{};
    packet[0] = static_cast<uint8_t>(m_robotSequence >> 8);
    packet[1] = static_cast<uint8_t>(m_robotSequence++);
    packet[2] = 1;
    packet[3] = ProgramStatusLocked();
    packet[4] = m_programStarted ? 0x20 : 0x10;
    packet[5] = 0;
    packet[6] = 0;
    packet[7] = 0;
    packet[8] = 9;  // tag length (tag + u32 + u16 + u16)
    packet[9] = 1;  // DS_UDP_PROTOCOL_HID_RUMBLE
    const uint32_t hid = htonl(static_cast<uint32_t>(outputs));
    const uint16_t leftValue = htons(static_cast<uint16_t>(left));
    const uint16_t rightValue = htons(static_cast<uint16_t>(right));
    std::memcpy(packet.data() + 10, &hid, sizeof(hid));
    std::memcpy(packet.data() + 14, &leftValue, sizeof(leftValue));
    std::memcpy(packet.data() + 16, &rightValue, sizeof(rightValue));
    return sendto(m_udp, packet.data(), 17, 0,
                  reinterpret_cast<const sockaddr*>(&m_dsAddress),
                  sizeof(m_dsAddress)) == 17;
  }

  void ObserveProgram(int32_t mode) {
    std::scoped_lock lock{m_mutex};
    m_programStarted = true;
    m_programMode = mode;
    SendStatusLocked();
  }

 private:
  void OpenSocketsLocked() {
    m_udp = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_udp >= 0) {
      int reuse = 1;
      setsockopt(m_udp, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
      sockaddr_in address{};
      address.sin_family = AF_INET;
      address.sin_addr.s_addr = htonl(INADDR_ANY);
      address.sin_port = htons(kRobotUdpPort);
      if (bind(m_udp, reinterpret_cast<const sockaddr*>(&address),
               sizeof(address)) != 0) {
        close(m_udp);
        m_udp = -1;
      }
    }

    m_tcp = socket(AF_INET, SOCK_STREAM, 0);
    if (m_tcp >= 0) {
      int reuse = 1;
      setsockopt(m_tcp, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
      sockaddr_in address{};
      address.sin_family = AF_INET;
      address.sin_addr.s_addr = htonl(INADDR_ANY);
      address.sin_port = htons(kRobotTcpPort);
      if (bind(m_tcp, reinterpret_cast<const sockaddr*>(&address),
               sizeof(address)) != 0 || listen(m_tcp, 1) != 0) {
        close(m_tcp);
        m_tcp = -1;
      } else {
        fcntl(m_tcp, F_SETFL, O_NONBLOCK);
      }
    }
  }

  void CloseSocketsLocked() {
    if (m_client >= 0) {
      close(m_client);
      m_client = -1;
    }
    if (m_udp >= 0) {
      close(m_udp);
      m_udp = -1;
    }
    if (m_tcp >= 0) {
      close(m_tcp);
      m_tcp = -1;
    }
  }

  bool IsRunning() const {
    std::scoped_lock lock{m_mutex};
    return m_running;
  }

  void Run() {
    std::array<uint8_t, kMaxDatagram> datagram{};
    std::vector<uint8_t> tcpBuffer;
    while (IsRunning()) {
      fd_set readSet;
      FD_ZERO(&readSet);
      int maxFd = -1;
      {
        std::scoped_lock lock{m_mutex};
        for (int fd : {m_udp, m_tcp, m_client}) {
          if (fd >= 0) {
            FD_SET(fd, &readSet);
            maxFd = std::max(maxFd, fd);
          }
        }
      }
      timeval timeout{0, 50'000};
      if (maxFd < 0 || select(maxFd + 1, &readSet, nullptr, nullptr,
                              &timeout) < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (!IsRunning()) {
          break;
        }
      }

      int udp = -1;
      int tcp = -1;
      int client = -1;
      {
        std::scoped_lock lock{m_mutex};
        udp = m_udp;
        tcp = m_tcp;
        client = m_client;
      }
      if (udp >= 0 && FD_ISSET(udp, &readSet)) {
        sockaddr_in source{};
        socklen_t sourceLength = sizeof(source);
        const ssize_t count = recvfrom(
            udp, datagram.data(), datagram.size(), 0,
            reinterpret_cast<sockaddr*>(&source), &sourceLength);
        if (count > 0) {
          VMXDriverStationPacket packet;
          if (VMXDriverStationPacketParser::ParseUdp(
                  datagram.data(), static_cast<size_t>(count), packet)) {
            std::scoped_lock lock{m_mutex};
            m_dsAddress = source;
            m_haveDsAddress = true;
            if (m_state) {
              m_state->CommitUdp(packet);
            }
          } else {
            std::scoped_lock lock{m_mutex};
            if (m_state) {
              m_state->MarkInvalidPacket();
            }
          }
        }
      }
      if (tcp >= 0 && FD_ISSET(tcp, &readSet)) {
        const int accepted = accept(tcp, nullptr, nullptr);
        if (accepted >= 0) {
          std::scoped_lock lock{m_mutex};
          if (m_client >= 0) {
            close(m_client);
          }
          m_client = accepted;
          fcntl(m_client, F_SETFL, O_NONBLOCK);
          tcpBuffer.clear();
        }
      }
      if (client >= 0 && FD_ISSET(client, &readSet)) {
        std::array<uint8_t, 2048> buffer{};
        const ssize_t count = recv(client, buffer.data(), buffer.size(), 0);
        if (count <= 0) {
          std::scoped_lock lock{m_mutex};
          close(m_client);
          m_client = -1;
          tcpBuffer.clear();
        } else {
          tcpBuffer.insert(tcpBuffer.end(), buffer.begin(),
                           buffer.begin() + count);
          if (tcpBuffer.size() > kMaxTcpBuffer) {
            tcpBuffer.clear();
          } else {
            VMXDriverStationPacket metadata;
            if (VMXDriverStationPacketParser::ParseTcp(
                    tcpBuffer.data(), tcpBuffer.size(), metadata)) {
              std::scoped_lock lock{m_mutex};
              if (m_state) {
                m_state->CommitTcp(metadata);
              }
              tcpBuffer.clear();
            }
          }
        }
      }
      if (m_state) {
        m_state->PollTimeout();
      }
    }
  }

  uint8_t ProgramStatusLocked() const {
    switch (m_programMode) {
      case 1:
        return 0x04 | 0x02;  // enabled + autonomous
      case 2:
        return 0x04;  // enabled + teleop
      case 3:
        return 0x04 | 0x01;  // enabled + test
      default:
        return 0;
    }
  }

  void SendStatusLocked() {
    if (m_udp < 0 || !m_haveDsAddress) {
      return;
    }
    std::array<uint8_t, 8> packet{};
    packet[0] = static_cast<uint8_t>(m_robotSequence >> 8);
    packet[1] = static_cast<uint8_t>(m_robotSequence++);
    packet[2] = 1;
    packet[3] = ProgramStatusLocked();
    packet[4] = m_programStarted ? 0x20 : 0x10;
    sendto(m_udp, packet.data(), packet.size(), 0,
           reinterpret_cast<const sockaddr*>(&m_dsAddress),
           sizeof(m_dsAddress));
  }

  mutable std::mutex m_mutex;
  VMXDriverStationState* m_state = nullptr;
  std::thread m_thread;
  bool m_running = false;
  bool m_haveDsAddress = false;
  bool m_programStarted = false;
  int32_t m_programMode = 0;
  uint16_t m_robotSequence = 0;
  int m_udp = -1;
  int m_tcp = -1;
  int m_client = -1;
  sockaddr_in m_dsAddress{};
};

struct VMXDriverStationRuntime final {
  wpi::EventVector events;
  VMXDriverStationState state;
  VMXDriverStationTransport transport;

  VMXDriverStationRuntime()
      : state{VMXDriverStationState::Clock{}, [this] {
          events.Wakeup();
          NotifyWatchdogStateChanged();
        }} {
    state.SetOutput([this](int32_t joystick, int64_t outputs, int32_t left,
                           int32_t right) {
      return transport.SendJoystickOutputs(joystick, outputs, left, right);
    });
    state.SetProgramCallback(
        [this](int32_t mode) { transport.ObserveProgram(mode); });
  }
};

VMXDriverStationRuntime& GetRuntime() {
  static VMXDriverStationRuntime runtime;
  return runtime;
}

std::atomic<void (*)(const char*, size_t)> gPrintErrorImpl{nullptr};

}  // namespace

bool InitializeDriverStation() noexcept {
  auto& runtime = GetRuntime();
  runtime.transport.Start(runtime.state);
  return true;
}

void ShutdownDriverStation() noexcept {
  auto& runtime = GetRuntime();
  runtime.state.Shutdown();
  runtime.transport.Stop();
}

VMXDriverStationState& GetDriverStationState() noexcept {
  return GetRuntime().state;
}

wpi::EventVector& GetDriverStationEvents() noexcept {
  return GetRuntime().events;
}

namespace {
void PrintError(const char* line, size_t size) {
  auto handler = gPrintErrorImpl.load();
  if (handler) {
    handler(line, size);
  } else {
    std::fwrite(line, size, 1, stderr);
  }
}
}  // namespace

}  // namespace hal::vmx

extern "C" {

int32_t HAL_SendError(HAL_Bool isError, int32_t errorCode, HAL_Bool isLVCode,
                      const char* details, const char* location,
                      const char* callStack, HAL_Bool printMsg) {
  static_cast<void>(isLVCode);
  char buffer[1024];
  const int written = std::snprintf(
      buffer, sizeof(buffer), "%s%s%s%s%s%s\n", isError ? "Error" : "Warning",
      location && location[0] ? " at " : "", location ? location : "",
      details ? ": " : "", details ? details : "", callStack ? callStack : "");
  if (printMsg || isError) {
    hal::vmx::PrintError(buffer, static_cast<size_t>(
                                 std::clamp(written, 0, static_cast<int>(sizeof(buffer) - 1))));
  }
  static_cast<void>(errorCode);
  return HAL_SUCCESS;
}

void HAL_SetPrintErrorImpl(void (*func)(const char* line, size_t size)) {
  hal::vmx::gPrintErrorImpl.store(func);
}

int32_t HAL_SendConsoleLine(const char* line) {
  if (!line) {
    return PARAMETER_OUT_OF_RANGE;
  }
  std::fputs(line, stdout);
  std::fputc('\n', stdout);
  std::fflush(stdout);
  return HAL_SUCCESS;
}

int32_t HAL_GetControlWord(HAL_ControlWord* controlWord) {
  return hal::vmx::GetDriverStationState().GetControlWord(controlWord);
}

HAL_AllianceStationID HAL_GetAllianceStation(int32_t* status) {
  return hal::vmx::GetDriverStationState().GetAlliance(status);
}

int32_t HAL_GetJoystickAxes(int32_t joystickNum, HAL_JoystickAxes* axes) {
  return hal::vmx::GetDriverStationState().GetAxes(joystickNum, axes);
}

int32_t HAL_GetJoystickPOVs(int32_t joystickNum, HAL_JoystickPOVs* povs) {
  return hal::vmx::GetDriverStationState().GetPOVs(joystickNum, povs);
}

int32_t HAL_GetJoystickButtons(int32_t joystickNum,
                               HAL_JoystickButtons* buttons) {
  return hal::vmx::GetDriverStationState().GetButtons(joystickNum, buttons);
}

void HAL_GetAllJoystickData(HAL_JoystickAxes* axes, HAL_JoystickPOVs* povs,
                            HAL_JoystickButtons* buttons) {
  if (axes && povs && buttons) {
    hal::vmx::GetDriverStationState().GetAll(axes, povs, buttons);
  }
}

int32_t HAL_GetJoystickDescriptor(int32_t joystickNum,
                                  HAL_JoystickDescriptor* desc) {
  return hal::vmx::GetDriverStationState().GetDescriptor(joystickNum, desc);
}

HAL_Bool HAL_GetJoystickIsXbox(int32_t joystickNum) {
  HAL_JoystickDescriptor descriptor{};
  return hal::vmx::GetDriverStationState().GetDescriptor(joystickNum,
                                                         &descriptor) == HAL_SUCCESS
             ? descriptor.isXbox
             : false;
}

int32_t HAL_GetJoystickType(int32_t joystickNum) {
  HAL_JoystickDescriptor descriptor{};
  const int32_t status =
      hal::vmx::GetDriverStationState().GetDescriptor(joystickNum, &descriptor);
  return status == HAL_SUCCESS ? descriptor.type : -1;
}

void HAL_GetJoystickName(struct WPI_String* name, int32_t joystickNum) {
  if (!name) {
    return;
  }
  const char* value = "";
  size_t length = 0;
  hal::vmx::GetDriverStationState().GetJoystickName(joystickNum, &value,
                                                    &length);
  auto destination = WPI_AllocateString(name, length);
  if (length != 0) {
    std::memcpy(destination, value, length);
  }
}

int32_t HAL_GetJoystickAxisType(int32_t joystickNum, int32_t axis) {
  const int32_t result =
      hal::vmx::GetDriverStationState().GetAxisType(joystickNum, axis);
  return result == PARAMETER_OUT_OF_RANGE || result == INCOMPATIBLE_STATE
             ? -1
             : result;
}

int32_t HAL_SetJoystickOutputs(int32_t joystickNum, int64_t outputs,
                               int32_t leftRumble, int32_t rightRumble) {
  return hal::vmx::GetDriverStationState().SetJoystickOutputs(
      joystickNum, outputs, leftRumble, rightRumble);
}

double HAL_GetMatchTime(int32_t* status) {
  return hal::vmx::GetDriverStationState().GetMatchTime(status);
}

HAL_Bool HAL_GetOutputsEnabled(void) {
  return hal::vmx::GetDriverStationState().OutputsEnabled();
}

int32_t HAL_GetMatchInfo(HAL_MatchInfo* info) {
  return hal::vmx::GetDriverStationState().GetMatchInfo(info);
}

HAL_Bool HAL_RefreshDSData(void) {
  return hal::vmx::GetDriverStationState().Refresh();
}

void HAL_ProvideNewDataEventHandle(WPI_EventHandle handle) {
  if (handle) {
    hal::init::CheckInit();
    hal::vmx::GetDriverStationEvents().Add(handle);
  }
}

void HAL_RemoveNewDataEventHandle(WPI_EventHandle handle) {
  if (handle) {
    hal::vmx::GetDriverStationEvents().Remove(handle);
  }
}

void HAL_ObserveUserProgramStarting(void) {
  hal::vmx::GetDriverStationState().ObserveProgram(0);
}

void HAL_ObserveUserProgramDisabled(void) {
  hal::vmx::GetDriverStationState().ObserveProgram(0);
}

void HAL_ObserveUserProgramAutonomous(void) {
  hal::vmx::GetDriverStationState().ObserveProgram(1);
}

void HAL_ObserveUserProgramTeleop(void) {
  hal::vmx::GetDriverStationState().ObserveProgram(2);
}

void HAL_ObserveUserProgramTest(void) {
  hal::vmx::GetDriverStationState().ObserveProgram(3);
}

}  // extern "C"
