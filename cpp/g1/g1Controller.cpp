#include "g1Controller.hpp"
#include "joint_reader/joint_reader.hpp"
#include "utils/metadata_loader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <unitree/common/any.hpp>

G1Controller::G1Controller(std::string networkInterface,
                           std::string serialDevice,
                           bool isSimulation)
    : G1Robot(networkInterface, isSimulation),
      control_dt_(0.002),
      max_target_velocity_(1.0),
      targets_initialized_(false),
      commanded_targets_{},
      joint_reader_(serialDevice),
      bounds_(LoadBounds(G1_BOUNDS_PATH)),
      joints_metadata_(LoadMetadata(READER_BOUNDS_PATH)) {
  StartControlThread();
}



void printDebug(ExoReadings exo){

  auto p = [&](int index) -> std::string {
    
    double val = exo[index].netAngle;
    char buf[8];
    char sign = val < 0 ? '-' : '+';
    std::snprintf(buf, sizeof(buf), "%c%.2f", sign, std::abs(val));
    return std::string("    ") + buf + " |";
};

  std::string out;
  out += "\033[H\033[J";
  out += "__________________________________________________________________________\n";
  out += " L_sh_pit | L_sh_rol | L_sh_yaw |  L_elbow | L_wr_rol | L_wr_pit | L_wr_yaw\n";
  out += p(0) + p(1) + p(2) + p(3) + p(4) + p(5) + p(6) + "\n";
  out += " R_sh_pit | R_sh_rol | R_sh_yaw |  R_elbow | R_wr_rol | R_wr_pit | R_wr_yaw\n";
  out += p(7) + p(8) + p(9) + p(10) + p(11) + p(12) + p(13) + "\n";
  std::cout << out << std::flush;
}

double G1Controller::toG1Angle(JointReading reading){
  auto [joint, netAngle, is_valid_] = reading;
  auto [low, high] = bounds_[joint];
  ReadingMetadata metadata = joints_metadata_[joint];

  double value = metadata.alignedWithRobot ? low + netAngle : high - netAngle;
  return std::clamp(value, low, high);
}

static bool isInLeftArm(G1JointIndex joint){
  bool res = false;
  res |= joint == G1JointIndex::LeftShoulderPitch;
  res |= joint == G1JointIndex::LeftShoulderRoll;
  res |= joint == G1JointIndex::LeftShoulderYaw;
  res |= joint == G1JointIndex::LeftElbow;
  res |= joint == G1JointIndex::LeftWristPitch;
  res |= joint == G1JointIndex::LeftWristRoll;
  res |= joint == G1JointIndex::LeftWristYaw;
  return res;
}

void G1Controller::Control() {
  MotorCommand motor_command_tmp;
  const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
  if (!ms) {
    return;
  }

  if (!targets_initialized_) {
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      const double q = ms->q.at(i);
      commanded_targets_.at(i) = q;
    }
    targets_initialized_ = true;
  }

  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    motor_command_tmp.tau_ff.at(i) = 0.0;
    motor_command_tmp.q_target.at(i) = commanded_targets_.at(i);
    motor_command_tmp.dq_target.at(i) = 0.0;
    motor_command_tmp.kp.at(i) = Kp[i];
    motor_command_tmp.kd.at(i) = Kd[i];
  }

  mode_pr_ = Mode::PR;

  const ExoReadings exo = joint_reader_.Eval();
  const double max_step = max_target_velocity_ * control_dt_;
  for (const auto& reading : exo) {
    if (reading.is_valid && isInLeftArm(reading.joint)) {
      const int joint_index = static_cast<int>(reading.joint);
      const double desired_target = toG1Angle(reading);

      commanded_targets_.at(joint_index) += std::clamp(
          desired_target - commanded_targets_.at(joint_index), -max_step, max_step);
      motor_command_tmp.q_target.at(joint_index) = commanded_targets_.at(joint_index);
    }
  }

  motor_command_buffer_.SetData(motor_command_tmp);

  //printDebug(exo);
}
