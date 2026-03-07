#pragma once
#include "constants.hpp"

#include <array>
#include <atomic>
#include <optional>
#include <string>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>

/**
 * Low-level relay reader backed by TCP socket.
 * Connects to relay.py at host:port and reads binary frames:
 *   [0xAA55 sync (u16)] [timestamp_us (u64)] [14×value (u16)] [CRC-16 (u16)]
 * GetNextLine() syncs, validates CRC, and returns the data as SerialLine.
 */
struct SerialLine {
  u_int64_t timestamp;
  std::array<u_int16_t, JOINT_COUNT> data;
};


class SerialReader {
 public:
  /** Connect to relay at address "host:port" (e.g. "127.0.0.1:5000"). */
  explicit SerialReader(const std::string& relay_address);
  ~SerialReader();

  SerialReader(const SerialReader&) = delete;
  SerialReader& operator=(const SerialReader&) = delete;

  bool IsOk() const { return socket_fd_ >= 0; }

  void Stop();

  std::optional<SerialLine> GetNextLine();

 private:
  int socket_fd_{-1};
  std::atomic<bool> running_{true};
};
