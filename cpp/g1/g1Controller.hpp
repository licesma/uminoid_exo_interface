#pragma once

#include <string>

#include "../joint_reader/joint_reader.hpp"
#include "../utils/bounds_loader.hpp"
#include "g1Robot.hpp"

class G1Controller : public G1Robot {
 private:
  double time_;
  double control_dt_;
  double duration_;

  JointReader joint_reader_;
  JointBounds bounds_;
  double toG1Angle(JointReading reading);

 public:
  G1Controller(std::string networkInterface,
               std::string serialDevice,
               bool isSimulation );

  void Control() override; 
};
