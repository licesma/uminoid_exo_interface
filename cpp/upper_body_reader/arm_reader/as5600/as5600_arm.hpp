#pragma once

#include "../skeleton_arm.hpp"

#include <atomic>
#include <cstddef>
#include <string>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>

/**
 * Low-level relay reader backed by TCP socket (one arm).
 * Connects to relay.py at host:port and reads binary frames:
 *   [0xAA55 sync (u16)] [timestamp_us (u64)] [14×value (u16)] [CRC-16 (u16)]
 * Extracts only the ARM_JOINT_COUNT joints starting at joint_offset.
 */
class AS5600Arm : public SkeletonArm {
 public:
  /**
   * @param relay_address  "host:port" for the TCP relay.
   * @param joint_offset   Index of the first joint to extract from the 14-joint frame.
   *                        0 for left arm, ARM_JOINT_COUNT for right arm.
   */
  AS5600Arm(const std::string& relay_address, size_t joint_offset,
            const std::function<void(const std::string&)>& raise_error = nullptr);
  ~AS5600Arm() override;

  void Stop() override;
  std::optional<ArmLine> GetNextLine(
      const std::function<void(const std::string&)>& raise_error) override;

 private:
  int socket_fd_{-1};
  std::atomic<bool> running_{true};
  size_t joint_offset_;
};
