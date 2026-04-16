#pragma once

#include "../skeleton_arm.hpp"
#include "../../../utils/dynamixel_timestamp_helper.hpp"

#include <atomic>
#include <cstdint>
#include <string>

namespace dynamixel {
class PortHandler;
class PacketHandler;
class GroupFastSyncRead;
}

/**
 * Single-arm Dynamixel reader using FastSyncRead over USB (U2D2).
 * GetNextLine() blocks on txRxPacket() and returns fresh data each call.
 * If a joint is unavailable, its value defaults to FALLBACK_VALUE (5000).
 */
class DynamixelArm : public SkeletonArm {
 public:
  static constexpr uint16_t FALLBACK_VALUE = 5000;

  explicit DynamixelArm(const std::string& device = "/dev/ttyUSB0",
                           int baudrate = 1000000,
                           const std::function<void(const std::string&)>& raise_error = nullptr);
  ~DynamixelArm() override;

  void Stop() override;
  std::optional<ArmLine> GetNextLine(
      const std::function<void(const std::string&)>& raise_error) override;

 private:
  // Realtime Tick: address 120, 2 bytes (uint16, ms, wraps at 32767)
  static constexpr uint16_t ADDR_REALTIME_TICK = 120;
  static constexpr uint16_t LEN_REALTIME_TICK = 2;

  // Present Position: address 132, 4 bytes
  static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
  static constexpr uint16_t LEN_PRESENT_POSITION = 4;

  // Sync read from ADDR_REALTIME_TICK to end of ADDR_PRESENT_POSITION
  static constexpr uint16_t SYNC_READ_START = ADDR_REALTIME_TICK;
  static constexpr uint16_t SYNC_READ_LEN =
      (ADDR_PRESENT_POSITION + LEN_PRESENT_POSITION) - ADDR_REALTIME_TICK;

  static constexpr float PROTOCOL_VERSION = 2.0;
  static constexpr uint8_t DXL_IDS[] = {0, 1, 2, 3, 4, 5, 6};

  dynamixel::PortHandler* port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  dynamixel::GroupFastSyncRead* group_sync_read_{nullptr};

  std::atomic<bool> running_{true};

  DynamixelTimestampHelper timestamp_helper_{DXL_IDS[0]};
};
