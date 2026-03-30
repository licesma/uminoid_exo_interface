#pragma once
#include "../g1/model/g1Enums.hpp"
#include "../g1/model/g1Structs.hpp"
#include <array>
#include <stdexcept>
#include <string>
#include <utility>
#include <yaml-cpp/yaml.h>

struct JointBounds {
  std::array<std::pair<double, double>, G1_NUM_MOTOR> data{};

  std::pair<double, double>& operator[](G1JointIndex idx) { return data[idx]; }
  const std::pair<double, double>& operator[](G1JointIndex idx) const { return data[idx]; }

};

/** Load joint bounds from a YAML file (e.g. upperBodyJointBounds.yaml). */
inline JointBounds LoadBounds(const std::string& path) {
  try {
    YAML::Node config = YAML::LoadFile(path);

    auto getBounds = [&config](const char* joint) {
      return std::pair<double, double>{config[joint]["lower"].as<double>(),
                                       config[joint]["upper"].as<double>()};
    };

    JointBounds bounds;
    bounds[LeftShoulderPitch]  = getBounds("left_shoulder_pitch");
    bounds[LeftShoulderRoll]   = getBounds("left_shoulder_roll");
    bounds[LeftShoulderYaw]    = getBounds("left_shoulder_yaw");
    bounds[LeftElbow]          = getBounds("left_elbow");
    bounds[LeftWristRoll]      = getBounds("left_wrist_roll");
    bounds[LeftWristPitch]     = getBounds("left_wrist_pitch");
    bounds[LeftWristYaw]       = getBounds("left_wrist_yaw");
    bounds[RightShoulderPitch] = getBounds("right_shoulder_pitch");
    bounds[RightShoulderRoll]  = getBounds("right_shoulder_roll");
    bounds[RightShoulderYaw]   = getBounds("right_shoulder_yaw");
    bounds[RightElbow]         = getBounds("right_elbow");
    bounds[RightWristRoll]     = getBounds("right_wrist_roll");
    bounds[RightWristPitch]    = getBounds("right_wrist_pitch");
    bounds[RightWristYaw]      = getBounds("right_wrist_yaw");
    return bounds;
  } catch (const YAML::Exception& e) {
    throw std::runtime_error(std::string("LoadBounds: ") + path + ": " +
                            e.what());
  }
}
