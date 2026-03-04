#pragma once

#include "../utils/bounds_loader.hpp"
#include "constants.hpp"
#include "serial_manager.hpp"
#include "../g1/model/g1Enums.hpp"
#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <utility>

/** Snapshot of all 14 exoskeleton encoder readings, keyed by robot joint index. */
struct JointReading {
  G1JointIndex joint;
  double netAngle;
};

using ExoReadings = std::array<JointReading, 14>;

/*
 * Higher-level wrapper around SerialManager.
 * Eval() returns a stack-allocated snapshot of all 14 arm joints, ready to
 * iterate and write directly into a MotorCommand.
 */
class JointReader {
 public:
  explicit JointReader(const std::string& device_path,
                      double default_value = 0.0,
                      const std::string& bounds_path =
                          "../joint_reader/upperBodyReaderBounds.yaml")
      : serial_(device_path), bounds_(LoadBounds(bounds_path)) {}


  static G1JointIndex getG1JointIndex(ExoIndex j){
    return static_cast<G1JointIndex>(static_cast<int>(j) + 15);
  }

  ExoReadings Eval() const {
    const auto snapshot = serial_.Snapshot();
    
    auto getBoundedAngle = [&bounds = bounds_, &snapshot](ExoIndex j) {
      int exoIdx = static_cast<int>(j);
      int g1Idx = getG1JointIndex(j);
      auto [lower, upper] = bounds[g1Idx];
      double radian = (snapshot.data[exoIdx] - ENCODER_RESOLUTION/2.0)*ENCODER_PRECISION_RAD; 
      if(exoIdx == 0){
        std::cout<<"Bounds: ["<<lower<<", "<<upper<<"]"<<std::endl;
        std::cout<<"Snapshot:"<<radian<<std::endl;
        std::cout<<"Eval: "<<std::clamp(radian - lower, 0.0, upper - lower )<<std::endl;
      }

      return std::clamp(radian - lower, 0.0, upper - lower );
    };

    return {{
        {LeftShoulderPitch, getBoundedAngle(ExoIndex::LeftShoulderPitch)},
        {LeftShoulderRoll, getBoundedAngle(ExoIndex::LeftShoulderRoll)},
        {LeftShoulderYaw, getBoundedAngle(ExoIndex::LeftShoulderYaw)},
        {LeftElbow, getBoundedAngle(ExoIndex::LeftElbow)},
        {LeftWristRoll, getBoundedAngle(ExoIndex::LeftWristRoll)},
        {LeftWristPitch, getBoundedAngle(ExoIndex::LeftWristPitch)},
        {LeftWristYaw, getBoundedAngle(ExoIndex::LeftWristYaw)},
        {RightShoulderPitch, getBoundedAngle(ExoIndex::RightShoulderPitch)},
        {RightShoulderRoll, getBoundedAngle(ExoIndex::RightShoulderRoll)},
        {RightShoulderYaw, getBoundedAngle(ExoIndex::RightShoulderYaw)},
        {RightElbow, getBoundedAngle(ExoIndex::RightElbow)},
        {RightWristRoll, getBoundedAngle(ExoIndex::RightWristRoll)},
        {RightWristPitch, getBoundedAngle(ExoIndex::RightWristPitch)},
        {RightWristYaw, getBoundedAngle(ExoIndex::RightWristYaw)},
    }};
  }

 private:
  SerialManager serial_;
  JointBounds bounds_;
};
