
#include <fcntl.h>
#include <cerrno>
#include <cstdio>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <array>
#include <deque>

#include "uart.hpp"

UART::UART(const char* dev) {
  fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) { perror("open"); return; }
  termios tio{};
  tcgetattr(fd, &tio);
  cfmakeraw(&tio);
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  tio.c_cflag |= (CLOCAL | CREAD);
  tcsetattr(fd, TCSANOW, &tio);
}

UART::~UART() {
  if (fd >= 0) {
    close(fd);
  }
}

bool UART::isOpen() const {
  return fd >= 0;
}

namespace {

std::optional<UART::Packet> decodePacket(uint8_t type, const uint8_t* payload, uint8_t len) {
  switch (type) {
    case dlink::PKT_TELEMETRY:
      if (len == sizeof(dlink::Telemetry)) {
        dlink::Telemetry t{};
        std::memcpy(&t, payload, sizeof(t));
        return t;
      }
      break;
    case dlink::PKT_TARGET:
      if (len == sizeof(dlink::TargetPos)) {
        dlink::TargetPos t{};
        std::memcpy(&t, payload, sizeof(t));
        return t;
      }
      break;
    case dlink::PKT_AMMO:
      if (len == sizeof(dlink::AmmoCfg)) {
        dlink::AmmoCfg a{};
        std::memcpy(&a, payload, sizeof(a));
        LOG("Received PKT_AMMO: name=" << a.name << ", mass=" << a.mass << ", drag=" << a.drag << ", lift=" << a.lift << ", hitRadius=" << a.hitRadius << ", nTargets=" << static_cast<int>(a.nTargets));
        return a;
      }
      break;
    case dlink::PKT_CONFIG:
      if (len == sizeof(dlink::DroneCfg)) {
        dlink::DroneCfg c{};
        std::memcpy(&c, payload, sizeof(c));
        LOG("Received PKT_CONFIG: attackSpeed=" << c.attackSpeed << ", accelerationPath=" << c.accelerationPath << ", angularSpeed=" << c.angularSpeed << ", turnThreshold=" << c.turnThreshold << ", timeStep=" << c.timeStep << ", timeScale=" << c.timeScale);
        return c;
      }
      break;
    case dlink::PKT_RESULT:
      if (len == sizeof(dlink::Result)) {
        dlink::Result r{};
        std::memcpy(&r, payload, sizeof(r));
        return r;
      }
      break;
    case dlink::PKT_CONTROL:
      if (len == sizeof(dlink::Control)) {
        dlink::Control c{};
        std::memcpy(&c, payload, sizeof(c));
        return c;
      }
      break;
    default:
      std::cerr << "Unknown packet type: " << static_cast<int>(type) << std::endl;
      break;
  }

  return std::nullopt;
}

} // namespace

std::optional<UART::Packet> UART::readPacket() {
  std::lock_guard<std::mutex> lock(mtx);

  if (!pendingPackets.empty()) {
    Packet packet = pendingPackets.front();
    pendingPackets.pop_front();
    return packet;
  }

  if (!isOpen()) {
    return std::nullopt;
  }

  std::array<uint8_t, 256> buf{};
  uint8_t type = 0;
  uint8_t len = 0;
  uint8_t payload[260] = {0};

  const ssize_t n = ::read(fd, buf.data(), buf.size());
  if (n <= 0) {
    return std::nullopt;
  }

  for (ssize_t i = 0; i < n; i++) {
    if (!parser.feed(buf[static_cast<std::size_t>(i)], type, payload, len)) {
      continue;
    }

    std::optional<Packet> packet = decodePacket(type, payload, len);
    if (!packet.has_value()) {
      continue;
    }

    if (pendingPackets.size() >= kPendingPacketCapacity) {
      pendingPackets.pop_front();
    }
    pendingPackets.push_back(*packet);
  }

  if (!pendingPackets.empty()) {
    Packet packet = pendingPackets.front();
    pendingPackets.pop_front();
    return packet;
  }

  return std::nullopt;
}

int UART::sendControl(float accel, float turnRate) {
  if (fd < 0) {
    return -1;
  }

  dlink::Control c{
      std::clamp(accel, -1.0f, 1.0f),
      std::clamp(turnRate, -1.0f, 1.0f)
  };
  uint8_t out[64];
  size_t m = dlink::encode(dlink::PKT_CONTROL, &c, sizeof(c), out);
  const ssize_t written = write(fd, out, m);
  if (written < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }
    perror("write UART");
    return -1;
  }

  return static_cast<int>(written);
}
