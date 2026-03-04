#include "g1Controller.hpp"
#include "g1/model/g1Enums.hpp"
#include "joint_reader.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <unitree/common/any.hpp>

G1Controller::G1Controller(std::string networkInterface,
                           std::string serialDevice,
                           bool isSimulation)
    : G1Robot(networkInterface, isSimulation),
      time_(0.0),
      control_dt_(0.002),
      duration_(30000.0),
      joint_reader_(serialDevice),
      bounds_(LoadBounds(G1_BOUNDS_PATH)) {
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
  auto [joint, netAngle] = reading;
  auto [low, high] = bounds_[joint];

  return std::clamp(low + netAngle, low, high);
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

    time_ += control_dt_;
    mode_pr_ = Mode::PR;

      /*
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        //double ratio = std::clamp(time_ / duration_, 0.0, 1.0);
        //motor_command_tmp.q_target.at(i) = (1.0 - ratio) * ms->q.at(i);
      }
      */
      const ExoReadings exo = joint_reader_.Eval();
      
      for (const auto& reading : exo) {
        motor_command_tmp.q_target.at(reading.joint) = toG1Angle(reading);
      }

      motor_command_buffer_.SetData(motor_command_tmp);
      
      //printDebug(exo);
}
