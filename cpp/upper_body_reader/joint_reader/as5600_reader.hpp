#pragma once

#include "joint_reader.hpp"

#include <atomic>
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
 */
class AS5600Reader : public JointReader {
 public:
  /** Connect to relay at address "host:port" (e.g. "127.0.0.1:5000"). */
  explicit AS5600Reader(const std::string& relay_address);
  ~AS5600Reader() override;

  bool IsOk() const override { return socket_fd_ >= 0; }
  void Stop() override;
  std::optional<JointLine> GetNextLine() override;

 private:
  int socket_fd_{-1};
  std::atomic<bool> running_{true};
};
