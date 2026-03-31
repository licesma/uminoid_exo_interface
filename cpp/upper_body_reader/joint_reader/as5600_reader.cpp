#include "as5600_reader.hpp"
#include "constants.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

static constexpr uint16_t SYNC_WORD = 0xAA55;
static constexpr size_t FRAME_SIZE = 40;   // 2 header + 8 ts + 28 vals + 2 crc
static constexpr size_t VALUES_COUNT = 14;

static uint16_t crc16_ccitt(const uint8_t* data, size_t len,
                            uint16_t init = 0xFFFF) {
  uint16_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

static std::pair<std::string, uint16_t> parse_relay_address(
    const std::string& relay_address) {
  size_t colon = relay_address.find(':');
  if (colon == std::string::npos || colon == 0 || colon == relay_address.size() - 1) {
    throw std::invalid_argument("relay_address must be 'host:port'");
  }
  std::string host = relay_address.substr(0, colon);
  uint16_t port = static_cast<uint16_t>(std::stoul(relay_address.substr(colon + 1)));
  return {host, port};
}

AS5600Reader::AS5600Reader(const std::string& relay_address) {
  try {
    auto [host, port] = parse_relay_address(relay_address);

    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error("socket() failed");
    }

    int flags = 1;
    if (setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flags, sizeof(flags)) < 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("setsockopt(TCP_NODELAY) failed");
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) <= 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("invalid host address");
    }

    if (connect(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr),
                sizeof(addr)) < 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("connect() failed");
    }
  } catch (const std::exception& e) {
    std::cerr << "AS5600Reader: failed to connect to relay " << relay_address
              << ": " << e.what() << "\n";
  }
}

AS5600Reader::~AS5600Reader() {
  Stop();
}

void AS5600Reader::Stop() {
  running_.store(false);
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

std::optional<JointLine> AS5600Reader::GetNextLine() {
  uint8_t buf[FRAME_SIZE];

  while (running_.load() && socket_fd_ >= 0) {
    // Sync: find first byte of LE sync word (0x55)
    ssize_t n = recv(socket_fd_, &buf[0], 1, 0);
    if (n <= 0) return std::nullopt;
    if (buf[0] != (SYNC_WORD & 0xFF)) continue;

    n = recv(socket_fd_, &buf[1], 1, 0);
    if (n <= 0) return std::nullopt;
    if (buf[1] != (SYNC_WORD >> 8)) continue;

    // Read the remaining 38 bytes (timestamp + values + crc)
    size_t remaining = FRAME_SIZE - 2;
    size_t total = 0;
    while (total < remaining && running_.load()) {
      n = recv(socket_fd_, &buf[2] + total, remaining - total, 0);
      if (n <= 0) return std::nullopt;
      total += static_cast<size_t>(n);
    }

    // CRC covers timestamp + values (bytes 2..37, 36 bytes)
    uint16_t expected_crc;
    std::memcpy(&expected_crc, &buf[38], sizeof(uint16_t));
    if (crc16_ccitt(&buf[2], 36) != expected_crc) continue;

    uint64_t timestamp;
    std::memcpy(&timestamp, &buf[2], sizeof(uint64_t));

    std::array<uint16_t, JOINT_COUNT> jointData;
    for (size_t joint = 0; joint < JOINT_COUNT; ++joint) {
      uint16_t val;
      std::memcpy(&val, &buf[10 + joint * 2], sizeof(uint16_t));
      jointData[joint] = val;
    }
    return JointLine{timestamp, jointData};
  }
  return std::nullopt;
}
