#include "g1Controller.hpp"

#include <cmath>
#include <cstdio>

G1Controller::G1Controller(std::string networkInterface,
                           std::string serial_device)
    : G1Robot(networkInterface),
      time_(0.0),
      control_dt_(0.002),
      duration_(3.0),
      right_elbow_cmd_(0.0),
      elbow_ramp_rate_(3),
      joint_reader_(serial_device, 0.0) {
  StartControlThread();
}

void G1Controller::Control() {
  MotorCommand motor_command_tmp;
  const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();

  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    motor_command_tmp.tau_ff.at(i) = 0.0;
    motor_command_tmp.q_target.at(i) = 0.0;
    motor_command_tmp.dq_target.at(i) = 0.0;
    motor_command_tmp.kp.at(i) = Kp[i];
    motor_command_tmp.kd.at(i) = Kd[i];
  }

  if (ms) {
    time_ += control_dt_;
    mode_pr_ = Mode::PR;

    if (time_ < duration_) {
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        double ratio = std::clamp(time_ / duration_, 0.0, 1.0);
        motor_command_tmp.q_target.at(i) = (1.0 - ratio) * ms->q.at(i);
      }
    } else {
      double goal_rad = right_elbow_target_rad_.load();
      double max_step = elbow_ramp_rate_ * control_dt_;
      right_elbow_cmd_ += std::clamp(goal_rad - right_elbow_cmd_, -max_step, max_step);
      motor_command_tmp.q_target.at(RightElbow) = joint_reader_.Eval();
      printf("%.2f,", joint_reader_.Eval());
    }

    motor_command_buffer_.SetData(motor_command_tmp);
  }
}
