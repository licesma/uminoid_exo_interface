#include "upper_body_reader.hpp"

#include "arm_reader/as5600/as5600_arm.hpp"
#include "arm_reader/dynamixel/dynamixel_arm.hpp"

#include "../utils/circular_math.hpp"
#include "constants.hpp"

#include <iostream>

static G1JointReading invalidReading(G1JointIndex joint) {
  return {joint, -1, false};
}

UpperBodyReader::UpperBodyReader(const std::string& relay_address,
                                 double default_value,
                                 const std::string& bounds_path)
    : left(std::make_unique<AS5600Arm>(relay_address, 0)),
      right(std::make_unique<AS5600Arm>(relay_address, ARM_JOINT_COUNT)),
      bounds_(LoadBounds(bounds_path)) {}

UpperBodyReader::UpperBodyReader(const std::string& left_device,
                                 const std::string& right_device,
                                 int baudrate,
                                 const std::string& bounds_path)
    : left(left_device.empty() ? nullptr : std::make_unique<DynamixelArm>(left_device, baudrate)),
      right(right_device.empty() ? nullptr : std::make_unique<DynamixelArm>(right_device, baudrate)),
      bounds_(LoadBounds(bounds_path)) {}

void UpperBodyReader::PrintRaw() const {
  const auto left_data = left.Snapshot();
  const auto right_data = right.Snapshot();

  auto p = [](uint16_t val) -> std::string {
    char buf[12];
    std::snprintf(buf, sizeof(buf), "  %5u |", val);
    return buf;
  };

  std::string out;
  out += "\033[H\033[J";
  out += "__________________________________________________________________________\n";
  out += " L_sh_pit | L_sh_rol | L_sh_yaw |  L_elbow | L_wr_rol | L_wr_pit | L_wr_yaw\n";
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) out += p(left_data.data[i]);
  out += "\n";
  out += " R_sh_pit | R_sh_rol | R_sh_yaw |  R_elbow | R_wr_rol | R_wr_pit | R_wr_yaw\n";
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) out += p(right_data.data[i]);
  out += "\n";
  std::cout << out << std::flush;
}

UpperBodyReadings UpperBodyReader::Eval() const {
  const auto left_data = left.Snapshot();
  const auto right_data = right.Snapshot();

  std::array<uint16_t, JOINT_COUNT> combined{};
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    combined[i] = left_data.data[i];
    combined[ARM_JOINT_COUNT + i] = right_data.data[i];
  }

  auto getReadingValue = [&bounds = bounds_, &combined](ExoIndex j) {
    int exoIdx = static_cast<int>(j);
    G1JointIndex g1Idx = getG1JointIndex(j);
    auto [lower, upper] = bounds[g1Idx];
    int bit_value = combined[exoIdx];
    double value = (bit_value - ENCODER_RESOLUTION / 2.0) * ENCODER_PRECISION_RAD;

    if (bit_value == 5000) return invalidReading(g1Idx);

    if (circularDistance(upper, lower) < circularDistance(value, lower))
      return invalidReading(g1Idx);
    return G1JointReading{g1Idx, circularDistance(value, lower), true};
  };

  UpperBodyReadings joint_values;
  for (auto joint : EXO_JOINT_INDICES)
    joint_values[static_cast<int>(joint)] = getReadingValue(joint);
  return joint_values;
}
