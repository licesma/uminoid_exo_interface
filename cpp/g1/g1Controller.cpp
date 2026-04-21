#include "g1Controller.hpp"

#include "g1/model/g1Enums.hpp"
#include "utils/circular_math.hpp"
#include "utils/repo_constants.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <sstream>

namespace {

constexpr uint16_t INVALID_EXO_READING = 5000;

constexpr std::array<G1JointIndex, ARM_JOINT_COUNT> LEFT_ARM_JOINTS = {
    G1JointIndex::LeftShoulderPitch, G1JointIndex::LeftShoulderRoll,
    G1JointIndex::LeftShoulderYaw,   G1JointIndex::LeftElbow,
    G1JointIndex::LeftWristRoll,     G1JointIndex::LeftWristPitch,
    G1JointIndex::LeftWristYaw,
};

constexpr std::array<G1JointIndex, ARM_JOINT_COUNT> RIGHT_ARM_JOINTS = {
    G1JointIndex::RightShoulderPitch, G1JointIndex::RightShoulderRoll,
    G1JointIndex::RightShoulderYaw,   G1JointIndex::RightElbow,
    G1JointIndex::RightWristRoll,     G1JointIndex::RightWristPitch,
    G1JointIndex::RightWristYaw,
};

std::string csv_header() {
  std::string h = "collection_id,timestamp,host_timestamp";
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i)
    h += ",joint_" + std::to_string(i);
  return h;
}

CsvSaver make_csv_saver(const std::string& recording_label,
                        const std::string& filename) {
  if (recording_label.empty()) return CsvSaver{};
  const std::string dir = repo_constants::DATA_DIR + "/" + recording_label;
  std::filesystem::create_directories(dir);
  return CsvSaver(dir + "/" + filename, csv_header());
}

MotorCommand make_motor_command(
    const std::array<double, G1_NUM_MOTOR>& commanded_targets) {
  MotorCommand motor_command;
  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    motor_command.tau_ff.at(i) = 0.0;
    motor_command.q_target.at(i) = commanded_targets.at(i);
    motor_command.dq_target.at(i) = 0.0;
    motor_command.kp.at(i) = Kp[i];
    motor_command.kd.at(i) = Kd[i];
  }
  return motor_command;
}

G1JointReading invalidReading(G1JointIndex joint) {
  return {joint, -1, false};
}

}  // namespace

G1Controller::G1Controller(std::string networkInterface, bool isSimulation,
                           const JointsReadingMetadata& metadata,
                           const JointBounds& reader_bounds,
                           const std::string& recording_label)
    : G1Robot(networkInterface, isSimulation),
      control_dt_(0.002),
      max_target_velocity_(2.0),
      commanded_targets_{},
      left_measured_csv_(make_csv_saver(recording_label, "left_measured.csv")),
      right_measured_csv_(
          make_csv_saver(recording_label, "right_measured.csv")),
      left_command_csv_(make_csv_saver(recording_label, "left_command.csv")),
      right_command_csv_(make_csv_saver(recording_label, "right_command.csv")),
      metadata_(metadata),
      bounds_(LoadBounds(G1_BOUNDS_PATH)),
      reader_bounds_(reader_bounds) {
}

ArmReadings G1Controller::decode_arm(const ArmLine& sample,
                                     bool from_left) const {
  const auto& joints = from_left ? LEFT_ARM_JOINTS : RIGHT_ARM_JOINTS;
  ArmReadings readings{};
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i) {
    const G1JointIndex g1_idx = joints[i];
    const auto [lower, upper] = reader_bounds_[g1_idx];
    const int bit_value = sample.data[i];
    const double value =
        (bit_value - ENCODER_RESOLUTION / 2.0) * ENCODER_PRECISION_RAD;

    if (bit_value == INVALID_EXO_READING) {
      readings[i] = invalidReading(g1_idx);
      continue;
    }
    if (circularDistance(upper, lower) < circularDistance(value, lower)) {
      readings[i] = invalidReading(g1_idx);
      continue;
    }

    const ReadingMetadata md = metadata_[g1_idx];
    const double net_angle = md.skeleton_ref == LOWER_BOUND
                                 ? circularDistance(value, lower)
                                 : circularDistance(upper, value);
    readings[i] = {g1_idx, net_angle, true};
  }
  return readings;
}

double G1Controller::toG1Angle(G1JointReading reading) {
  auto [joint, netAngle, is_valid_] = reading;
  auto [low, high] = bounds_[joint];
  ReadingMetadata md = metadata_[joint];

  double reference = md.g1_ref == LOWER_BOUND ? low : high;
  double direction = md.g1_ref == LOWER_BOUND ? 1.0 : -1.0;

  double value = reference + direction * netAngle;
  return std::clamp(value, low, high);
}

void G1Controller::record_arm(const ArmLine& sample, const MotorState& state,
                              const MotorCommand& command, bool from_left,
                              int collection_id) {
  CsvSaver& measured = from_left ? left_measured_csv_ : right_measured_csv_;
  CsvSaver& cmd = from_left ? left_command_csv_ : right_command_csv_;
  if (!measured || !cmd) return;

  const auto& joints = from_left ? LEFT_ARM_JOINTS : RIGHT_ARM_JOINTS;
  const std::string prefix = std::to_string(collection_id) + "," +
                             std::to_string(sample.timestamp) + "," +
                             std::to_string(sample.host_timestamp);
  std::ostringstream measured_row;
  std::ostringstream command_row;
  measured_row << prefix;
  command_row << prefix;

  for (const auto joint : joints) {
    const int idx = static_cast<int>(joint);
    measured_row << "," << state.q.at(idx);
    command_row << "," << command.q_target.at(idx);
  }

  measured.write_line(measured_row.str());
  cmd.write_line(command_row.str());
}

bool G1Controller::initialize_targets_from_robot_state(
    const std::function<bool()>& stop_requested) {
  while (!stop_requested()) {
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
    if (!ms) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    std::lock_guard<std::mutex> lock(update_mutex_);
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      const double q = ms->q.at(i);
      commanded_targets_.at(i) = q;
    }
    MotorCommand motor_command_tmp = make_motor_command(commanded_targets_);

    mode_pr_ = Mode::PR;
    motor_command_buffer_.SetData(motor_command_tmp);
    return true;
  }

  return false;
}

void G1Controller::process_arm_sample(const ArmLine& sample, bool from_left,
                                      int collection_id, bool record) {
  const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
  if (!ms) return;

  const ArmReadings arm_readings = decode_arm(sample, from_left);
  const double max_step = max_target_velocity_ * control_dt_;

  std::lock_guard<std::mutex> lock(update_mutex_);
  MotorCommand motor_command_tmp = make_motor_command(commanded_targets_);

  mode_pr_ = Mode::PR;
  for (const auto& reading : arm_readings) {
    if (!reading.is_valid) continue;
    const int joint_index = static_cast<int>(reading.joint);
    const double desired_target = toG1Angle(reading);
    commanded_targets_.at(joint_index) += std::clamp(
        desired_target - commanded_targets_.at(joint_index), -max_step,
        max_step);
    motor_command_tmp.q_target.at(joint_index) =
        commanded_targets_.at(joint_index);
  }

  if (record) record_arm(sample, *ms, motor_command_tmp, from_left, collection_id);
  motor_command_buffer_.SetData(motor_command_tmp);
}
