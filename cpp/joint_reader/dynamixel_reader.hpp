#pragma once
#include "constants.hpp"
#include "../utils/dynamixel_timestamp_helper.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>
#include <string>

#include "dynamixel_sdk.h"

/**
 * Data returned by DynamixelReader::GetNextLine().
 * timestamp_ms comes from the actuator's Realtime Tick register (address 120),
 * which is a 16-bit counter in milliseconds that wraps every ~32.8 s.
 * We unwrap it into a monotonic uint64_t (ms since reader creation).
 */
struct DynamixelLine {
  uint64_t timestamp_ms;
  std::array<uint16_t, JOINT_COUNT> data;
};

/**
 * Low-level Dynamixel reader using FastSyncRead over USB (U2D2).
 * Reads Realtime Tick + present position from a set of XL330 motors.
 */
class DynamixelReader {
 public:
  explicit DynamixelReader(const std::string& device = "/dev/ttyUSB0",
                           int baudrate = 1000000);
  ~DynamixelReader();

  DynamixelReader(const DynamixelReader&) = delete;
  DynamixelReader& operator=(const DynamixelReader&) = delete;

  bool IsOk() const { return ok_; }

  void Stop();

  std::optional<DynamixelLine> GetNextLine();

 private:
  // Realtime Tick: address 120, 2 bytes (uint16, ms, wraps at 32767)
  static constexpr uint16_t ADDR_REALTIME_TICK = 120;
  static constexpr uint16_t LEN_REALTIME_TICK = 2;

  // Present Position: address 132, 4 bytes
  static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
  static constexpr uint16_t LEN_PRESENT_POSITION = 4;

  // Sync read from ADDR_REALTIME_TICK to end of ADDR_PRESENT_POSITION
  // 132 + 4 - 120 = 16 bytes
  static constexpr uint16_t SYNC_READ_START = ADDR_REALTIME_TICK;
  static constexpr uint16_t SYNC_READ_LEN =
      (ADDR_PRESENT_POSITION + LEN_PRESENT_POSITION) - ADDR_REALTIME_TICK;

  static constexpr float PROTOCOL_VERSION = 2.0;
  static constexpr uint8_t DXL_IDS[] = {1, 2, 3, 4, 5, 6, 7};
  static constexpr size_t DXL_ID_COUNT = sizeof(DXL_IDS) / sizeof(DXL_IDS[0]);

  dynamixel::PortHandler* port_handler_{nullptr};
  dynamixel::PacketHandler* packet_handler_{nullptr};
  dynamixel::GroupFastSyncRead* group_sync_read_{nullptr};

  std::atomic<bool> running_{true};
  bool ok_{false};

  DynamixelTimestampHelper timestamp_helper_{DXL_IDS[0]};
};
