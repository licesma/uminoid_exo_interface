#include "upper_body_reader.hpp"

#include "arm_reader/as5600/as5600_arm.hpp"
#include "arm_reader/dynamixel/dynamixel_arm.hpp"

#include "../utils/circular_math.hpp"
#include "../utils/repo_constants.hpp"
#include "constants.hpp"
#include "utils/metadata_loader.hpp"

#include <filesystem>
#include <iostream>
#include <thread>

static G1JointReading invalidReading(G1JointIndex joint) {
  return {joint, -1, false};
}

UpperBodyReader::UpperBodyReader(const std::string& relay_address,
                                 const std::string& recording_label,
                                 double default_value)
    : metadata(LoadMetadata(AS5600_BOUNDS_PATH)),
      left(std::make_unique<AS5600Arm>(relay_address, 0),
           recording_label.empty() ? "" : repo_constants::DATA_DIR + "/" + recording_label + "/left_arm.csv"),
      right(std::make_unique<AS5600Arm>(relay_address, ARM_JOINT_COUNT),
            recording_label.empty() ? "" : repo_constants::DATA_DIR + "/" + recording_label + "/right_arm.csv"),
      recording_label_(recording_label),
      bounds_(LoadBounds(AS5600_BOUNDS_PATH)) {}

UpperBodyReader::UpperBodyReader(const std::string& left_device,
                                 const std::string& right_device,
                                 int baudrate,
                                 const std::string& recording_label,
                                 const std::function<void(const std::string&)>& raise_error)
    : metadata(LoadMetadata(DYNAMIXEL_BOUNDS_PATH)),
      left(left_device.empty() ? nullptr : std::make_unique<DynamixelArm>(left_device, baudrate, raise_error),
           repo_constants::DATA_DIR + "/" + recording_label + "/left_arm.csv",
           raise_error),
      right(right_device.empty() ? nullptr : std::make_unique<DynamixelArm>(right_device, baudrate, raise_error),
            repo_constants::DATA_DIR + "/" + recording_label + "/right_arm.csv",
            raise_error),
      recording_label_(recording_label),
      bounds_(LoadBounds(DYNAMIXEL_BOUNDS_PATH)) {}

void UpperBodyReader::PrintRaw() const {
  const auto left_data = left.snapshot();
  const auto right_data = right.snapshot();

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

void UpperBodyReader::collect_loop(
    const std::function<int()>&  collection_id,
    const std::function<bool()>& stop,
    const std::function<bool()>& pause) {
  std::thread left_thread([&] {
    left.collect_loop(collection_id, stop, pause);
  });
  std::thread right_thread([&] {
    right.collect_loop(collection_id, stop, pause);
  });

  left_thread.join();
  right_thread.join();
}

UpperBodyReadings UpperBodyReader::Eval() const {
  const auto left_data = left.snapshot();
  const auto right_data = right.snapshot();
  std::array<uint16_t, JOINT_COUNT> combined{};
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    combined[i] = left_data.data[i];
    combined[ARM_JOINT_COUNT + i] = right_data.data[i];
  }

  auto getReadingValue = [this, &combined](ExoIndex j) {
    int exoIdx = static_cast<int>(j);
    G1JointIndex g1Idx = getG1JointIndex(j);
    auto [lower, upper] = bounds_[g1Idx];
    int bit_value = combined[exoIdx];
    double value = (bit_value - ENCODER_RESOLUTION / 2.0) * ENCODER_PRECISION_RAD;

    if (bit_value == 5000) return invalidReading(g1Idx);

    if (circularDistance(upper, lower) < circularDistance(value, lower))
      return invalidReading(g1Idx);

    double net_angle = metadata[g1Idx].skeleton_ref == LOWER_BOUND
                           ? circularDistance(value, lower)
                           : circularDistance(upper, value);

    return G1JointReading{g1Idx, net_angle, true};
  };

  UpperBodyReadings joint_values;
  for (auto joint : EXO_JOINT_INDICES)
    joint_values[static_cast<int>(joint)] = getReadingValue(joint);
  return joint_values;
}
