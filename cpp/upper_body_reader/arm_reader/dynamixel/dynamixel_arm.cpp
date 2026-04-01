#include "dynamixel_arm.hpp"

#include "group_fast_sync_read.h"
#include "packet_handler.h"
#include "port_handler.h"

#include <chrono>
#include <cstdio>

DynamixelArm::DynamixelArm(const std::string& device, int baudrate) {
  port_handler_ = dynamixel::PortHandler::getPortHandler(device.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!port_handler_->openPort()) {
    fprintf(stderr, "DynamixelArm: failed to open port %s\n", device.c_str());
    return;
  }

  if (!port_handler_->setBaudRate(baudrate)) {
    fprintf(stderr, "DynamixelArm: failed to set baudrate %d on %s\n", baudrate, device.c_str());
    return;
  }

  group_sync_read_ = new dynamixel::GroupFastSyncRead(
      port_handler_, packet_handler_, SYNC_READ_START, SYNC_READ_LEN);

  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    if (!group_sync_read_->addParam(DXL_IDS[i])) {
      fprintf(stderr, "[%s][ID:%03d] groupFastSyncRead addParam failed\n",
              device.c_str(), DXL_IDS[i]);
      return;
    }
  }

  ok_ = true;
}

DynamixelArm::~DynamixelArm() {
  Stop();
}

void DynamixelArm::Stop() {
  running_.store(false);
  if (port_handler_) {
    port_handler_->closePort();
  }
  delete group_sync_read_;
  group_sync_read_ = nullptr;
}

std::optional<ArmLine> DynamixelArm::GetNextLine() {
  if (!ok_ || !running_.load()) return std::nullopt;

  int result = group_sync_read_->txRxPacket();
  auto host_clock = std::chrono::steady_clock::now();
  if (result != COMM_SUCCESS) {
    return std::nullopt;
  }

  // Extract timestamp from Realtime Tick
  auto timestamp_ms = timestamp_helper_.getTimestamp(*group_sync_read_);
  if (!timestamp_ms) {
    return std::nullopt;
  }

  // Read positions
  std::array<uint16_t, ARM_JOINT_COUNT> data;
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    if (!group_sync_read_->isAvailable(DXL_IDS[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
      data[i] = FALLBACK_VALUE;
      continue;
    }
    int32_t pos = group_sync_read_->getData(DXL_IDS[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    data[i] = static_cast<uint16_t>(pos & 0x0FFF);
  }

  uint64_t host_ts = std::chrono::duration_cast<std::chrono::microseconds>(
                         host_clock.time_since_epoch())
                         .count();
  return ArmLine{*timestamp_ms, host_ts, data};
}
