#include "bounds_loader.hpp"
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

JointBounds LoadBounds(const std::string& path) {
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
