#pragma once

#include <atomic>
#include <string>

#include "../joint_reader/joint_reader.hpp"
#include "g1Robot.hpp"
#include "model/g1Values.hpp"

class G1Controller : public G1Robot {
 private:
  double time_;
  double control_dt_;
  double duration_;
  double right_elbow_cmd_;
  double elbow_ramp_rate_;

  std::atomic<double> right_elbow_target_rad_{0.0};
  JointReader joint_reader_;

 public:
  G1Controller(std::string networkInterface,
               std::string serial_device = "/dev/ttyACM0");

  void SetElbowTargetRad(double rad) { right_elbow_target_rad_.store(rad); }
  
  void AddElbowRad(double delta) { right_elbow_target_rad_.store(right_elbow_target_rad_.load() + delta); }

  double GetElbowTargetRad() const { return right_elbow_target_rad_.load(); }

  void Control() override;
};
