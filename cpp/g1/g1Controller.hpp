#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "../utils/csv_saver.hpp"
#include "../utils/metadata_loader.hpp"
#include "upper_body_reader/arm_reader/skeleton_arm.hpp"
#include "upper_body_reader/constants.hpp"
#include "../utils/bounds_loader.hpp"
#include "amo_bridge.hpp"
#include "g1Robot.hpp"

#ifndef READER_BOUNDS_PATH
#define READER_BOUNDS_PATH \
  "../upper_body_reader/arm_reader/as5600/as5600_bounds.yaml"
#endif

#ifndef DYNAMIXEL_BOUNDS_PATH
#define DYNAMIXEL_BOUNDS_PATH \
  "../upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
#endif

struct G1JointReading {
  G1JointIndex joint;
  double netAngle;
  bool is_valid;
};

using ArmReadings = std::array<G1JointReading, ARM_JOINT_COUNT>;

class G1Controller : public G1Robot {
 private:
  double control_dt_;
  double max_target_velocity_;
  std::array<double, G1_NUM_MOTOR> commanded_targets_;
  CsvSaver left_measured_csv_;
  CsvSaver right_measured_csv_;
  CsvSaver left_command_csv_;
  CsvSaver right_command_csv_;
  std::mutex update_mutex_;
  JointsReadingMetadata metadata_;
  JointBounds bounds_;
  JointBounds reader_bounds_;

  // AMO sidecar plumbing. The bridge is owned unconditionally; G1Controller
  // is always run with AMO under the g1_upper_body_reader path.
  AmoBridge amo_bridge_;
  AmoCommand amo_command_;          // hardcoded standstill until step (f)
  uint64_t   amo_state_seq_ = 0;    // increments on every publish_amo_state()

  ArmReadings decode_arm(const ArmLine& sample, bool from_left) const;
  double toG1Angle(G1JointReading reading);
  void record_arm(const ArmLine& sample, const MotorState& state,
                  const MotorCommand& command, bool from_left,
                  int collection_id);

  // Snapshot motor + IMU state, pack into a state frame, hand to AmoBridge.
  // Cheap (a few memcpys + one ZMQ send). Safe to call at any rate.
  void publish_amo_state();

  // Pull the freshest AMO action from AmoBridge, rate-limit-clamp it, and
  // write into commanded_targets_[0..14]. No-op if no fresh action is
  // available (legs hold their last commanded pose via LowCommandWriter).
  void apply_amo_action();

 public:
  G1Controller(std::string networkInterface, bool isSimulation,
               const JointsReadingMetadata& metadata,
               const JointBounds& reader_bounds,
               const std::string& recording_label = "");
  ~G1Controller() override = default;

  bool initialize_targets_from_robot_state(
      const std::function<bool()>& stop_requested);
  void process_arm_sample(const ArmLine& sample, bool from_left,
                          int collection_id, bool record);
};
