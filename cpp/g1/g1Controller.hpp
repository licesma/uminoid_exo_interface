#pragma once

#include <array>
#include <memory>
#include <string>

#include "upper_body_reader/upper_body_reader.hpp"
#include "../utils/bounds_loader.hpp"
#include "g1Robot.hpp"

class G1Controller : public G1Robot {
 private:
  double control_dt_;
  double max_target_velocity_;
  bool targets_initialized_;
  std::array<double, G1_NUM_MOTOR> commanded_targets_;

  std::unique_ptr<UpperBodyReader> joint_reader_;
  JointBounds bounds_;
  double toG1Angle(G1JointReading reading);

 public:
  G1Controller(std::string networkInterface,
               std::unique_ptr<UpperBodyReader> jointReader,
               bool isSimulation);

  void Control() override;
};
