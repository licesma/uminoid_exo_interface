#include "dynamixel_timestamp_helper.hpp"

#include "group_fast_sync_read.h"

std::optional<uint64_t> DynamixelTimestampHelper::getTimestamp(
    dynamixel::GroupFastSyncRead& sync_read) {
  if (!sync_read.isAvailable(motor_id_, ADDR_REALTIME_TICK, LEN_REALTIME_TICK)) {
    return std::nullopt;
  }

  uint16_t tick = static_cast<uint16_t>(
      sync_read.getData(motor_id_, ADDR_REALTIME_TICK, LEN_REALTIME_TICK));

  if (first_tick_) {
    first_tick_ = false;
  } else {
    uint16_t delta = (tick >= last_tick_) ? (tick - last_tick_)
                                          : (tick + TICK_MAX - last_tick_);
    accumulated_ms_ += delta;
  }
  last_tick_ = tick;

  return accumulated_ms_;
}
