#include "dynamixel_arm.hpp"
#include "utils/time.hpp"

#include "group_fast_sync_read.h"
#include "packet_handler.h"
#include "port_handler.h"

#include <cstdio>

DynamixelArm::DynamixelArm(const std::string& device, int baudrate,
                           const std::function<void(const std::string&)>& raise_error) {
  port_handler_ = dynamixel::PortHandler::getPortHandler(device.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!port_handler_->openPort()) {
    if (raise_error) raise_error("[DynamixelArm] Failed to open port " + device);
    return;
  }

  if (!port_handler_->setBaudRate(baudrate)) {
    if (raise_error) raise_error("[DynamixelArm] Failed to set baudrate on " + device);
    return;
  }

  group_sync_read_ = new dynamixel::GroupFastSyncRead(
      port_handler_, packet_handler_, SYNC_READ_START, SYNC_READ_LEN);

  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    if (!group_sync_read_->addParam(DXL_IDS[i])) {
      if (raise_error) raise_error("[DynamixelArm] addParam failed for ID " + std::to_string(DXL_IDS[i]) + " on " + device);
      return;
    }
  }
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

std::optional<ArmLine> DynamixelArm::GetNextLine(
    const std::function<void(const std::string&)>& raise_error) {
  if (!running_.load() || !group_sync_read_) return std::nullopt;

  int result = group_sync_read_->txRxPacket();
  if (result != COMM_SUCCESS) {
    raise_error("[DynamixelArm] txRxPacket failed (code " + std::to_string(result) + ")");
    return std::nullopt;
  }

  // Extract timestamp from Realtime Tick
  auto timestamp_ms = timestamp_helper_.getTimestamp(*group_sync_read_);
  if (!timestamp_ms) {
    raise_error("[DynamixelArm] Timestamp unavailable");
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

  return ArmLine{*timestamp_ms, Time::ts(), data};
}
