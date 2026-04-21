#pragma once

#include <array>
#include <functional>
#include <mutex>
#include <string>

#include "../utils/csv_saver.hpp"
#include "../utils/metadata_loader.hpp"
#include "upper_body_reader/constants.hpp"
#include "../utils/bounds_loader.hpp"
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

using UpperBodyReadings = std::array<G1JointReading, JOINT_COUNT>;

struct UpperBodyLine {
  uint64_t timestamp = 0;
  uint64_t host_timestamp = 0;
  std::array<uint16_t, JOINT_COUNT> data{};
};

class G1Controller : public G1Robot {
 private:
  double control_dt_;
  double max_target_velocity_;
  std::array<double, G1_NUM_MOTOR> commanded_targets_;
  CsvSaver measured_csv_;
  CsvSaver command_csv_;
  std::mutex update_mutex_;
  JointsReadingMetadata metadata_;
  JointBounds bounds_;
  JointBounds reader_bounds_;
  UpperBodyReadings decode_upper_body(const UpperBodyLine& sample) const;
  double toG1Angle(G1JointReading reading);
  void record_upper_body(const MotorState& state, const MotorCommand& command);

 public:
  G1Controller(std::string networkInterface, bool isSimulation,
               const JointsReadingMetadata& metadata,
               const JointBounds& reader_bounds,
               const std::string& recording_label = "");
  ~G1Controller() override = default;

  bool initialize_targets_from_robot_state(
      const std::function<bool()>& stop_requested);
  void process_upper_body_sample(const UpperBodyLine& sample);
};
