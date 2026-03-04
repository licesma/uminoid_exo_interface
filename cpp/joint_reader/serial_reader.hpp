#pragma once
#include "constants.hpp"

#include <array>
#include <atomic>
#include <optional>
#include <string>

#include <serial/serial.h>



/**
 * Low-level serial port reader backed by wjwwood/serial.
 * Opens the device at 230400 8N1 and reads binary frames:
 *   [0xAA55 sync (u16)] [timestamp_us (u64)] [14×value (u16)] [CRC-16 (u16)]
 * GetNextLine() syncs, validates CRC, and returns the data as a CSV string
 * ("timestamp,v0,v1,...,v13") so the caller can parse it the same way.
 */

 struct SerialLine {
  u_int64_t timestamp;
  std::array<u_int16_t, JOINT_COUNT> data;
};


class SerialReader {
 public:
 

  explicit SerialReader(const std::string& device_path);
  ~SerialReader();

  SerialReader(const SerialReader&) = delete;
  SerialReader& operator=(const SerialReader&) = delete;

  bool IsOk() const { return port_.isOpen(); }

  void Stop();

  std::optional<SerialLine> GetNextLine();

 private:
  serial::Serial port_;
  std::atomic<bool> running_{true};
};
