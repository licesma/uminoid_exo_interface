#include "serial_reader.hpp"
#include "constants.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
static constexpr uint16_t SYNC_WORD    = 0xAA55;
static constexpr size_t   FRAME_SIZE   = 40;   // 2 header + 8 ts + 28 vals + 2 crc
static constexpr size_t   VALUES_COUNT = 14;

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

SerialReader::SerialReader(const std::string& device_path) {
  try {
    port_.setPort(device_path);
    port_.setBaudrate(230400);
    port_.setBytesize(serial::eightbits);
    port_.setParity(serial::parity_none);
    port_.setStopbits(serial::stopbits_one);
    port_.setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    port_.setTimeout(timeout);
    port_.open();
  } catch (const std::exception& e) {
    std::cerr << "SerialReader: failed to open " << device_path << ": "
              << e.what() << "\n";
  }
}

SerialReader::~SerialReader() {
  Stop();
}

void SerialReader::Stop() {
  running_.store(false);
  if (port_.isOpen()) port_.close();
}

std::optional<SerialLine> SerialReader::GetNextLine() {
  uint8_t buf[FRAME_SIZE];

  while (running_.load()) {
    try {
      // Sync: find first byte of LE sync word (0x55)
      if (port_.read(&buf[0], 1) != 1) continue;
      if (buf[0] != (SYNC_WORD & 0xFF)) continue;

      if (port_.read(&buf[1], 1) != 1) continue;
      if (buf[1] != (SYNC_WORD >> 8)) continue;

      // Read the remaining 38 bytes (timestamp + values + crc)
      size_t remaining = FRAME_SIZE - 2;
      if (port_.read(&buf[2], remaining) != remaining) continue;

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
      return SerialLine{timestamp, jointData};

    } catch (const std::exception&) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}
