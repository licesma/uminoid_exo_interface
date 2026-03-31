#pragma once

#include <cstdint>
#include <optional>

#include "dynamixel_sdk.h"

/**
 * Extracts and unwraps the Realtime Tick register (address 120, 2 bytes)
 * from a GroupFastSyncRead result into a monotonic millisecond counter.
 *
 * The tick is a 15-bit value (0–32767) that wraps every ~32.8 s.
 * This helper tracks the wrap and accumulates a monotonic timestamp.
 */
class DynamixelTimestampHelper {
 public:
  /**
   * @param motor_id  The Dynamixel ID to read the Realtime Tick from.
   */
  explicit DynamixelTimestampHelper(uint8_t motor_id) : motor_id_(motor_id) {}

  /**
   * Extract the Realtime Tick from a completed sync read and return
   * a monotonic timestamp in milliseconds.
   *
   * @param sync_read  A GroupFastSyncRead that has already called txRxPacket().
   * @return           Monotonic timestamp in ms since first call, or nullopt if unavailable.
   */
  std::optional<uint64_t> getTimestamp(dynamixel::GroupFastSyncRead& sync_read) {
    if (!sync_read.isAvailable(motor_id_, ADDR_REALTIME_TICK, LEN_REALTIME_TICK)) {
      return std::nullopt;
    }

    uint16_t tick = static_cast<uint16_t>(
        sync_read.getData(motor_id_, ADDR_REALTIME_TICK, LEN_REALTIME_TICK));

    if (first_tick_) {
      first_tick_ = false;
    } else {
      uint16_t delta = (tick >= last_tick_)
          ? (tick - last_tick_)
          : (tick + TICK_MAX - last_tick_);
      accumulated_ms_ += delta;
    }
    last_tick_ = tick;

    return accumulated_ms_;
  }

 private:
  static constexpr uint16_t ADDR_REALTIME_TICK = 120;
  static constexpr uint16_t LEN_REALTIME_TICK = 2;
  static constexpr uint16_t TICK_MAX = 32768;

  uint8_t motor_id_;
  uint16_t last_tick_{0};
  uint64_t accumulated_ms_{0};
  bool first_tick_{true};
};
