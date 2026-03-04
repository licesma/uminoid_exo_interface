#pragma once

#include "bounds_loader.hpp"
#include "constants.hpp"
#include "serial_manager.hpp"
#include "../g1/model/g1Enums.hpp"
#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <utility>

/** Snapshot of all 14 exoskeleton encoder readings, keyed by robot joint index. */
using ExoReadings = std::array<std::pair<G1JointIndex, double>, 14>;

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

  ExoReadings Eval() const {
    const auto snapshot = serial_.Snapshot();
    
    auto getBoundedAngle = [&bounds = bounds_, &snapshot](ExoIndex j) {
      int index = static_cast<int>(j);
      auto [lower, upper] = bounds[index].second;
      double radian = (snapshot.data[index] - ENCODER_RESOLUTION/2.0)*ENCODER_PRECISION_RAD; 
      if(index == 0){
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
  ExoJointBounds bounds_;
};
