#include "bounds_loader.hpp"
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

ExoJointBounds LoadBounds(const std::string& path) {
  try {
    YAML::Node config = YAML::LoadFile(path);

    auto getBounds = [&config](const char* joint) {
      return std::pair<double, double>{config[joint]["lower"].as<double>(),
                                       config[joint]["upper"].as<double>()};
    };

    return {{
      {LeftShoulderPitch, getBounds("left_shoulder_pitch")},
      {LeftShoulderRoll, getBounds("left_shoulder_roll")},
      {LeftShoulderYaw, getBounds("left_shoulder_yaw")},
      {LeftElbow, getBounds("left_elbow")},
      {LeftWristRoll, getBounds("left_wrist_roll")},
      {LeftWristPitch, getBounds("left_wrist_pitch")},
      {LeftWristYaw, getBounds("left_wrist_yaw")},
      {RightShoulderPitch, getBounds("right_shoulder_pitch")},
      {RightShoulderRoll, getBounds("right_shoulder_roll")},
      {RightShoulderYaw, getBounds("right_shoulder_yaw")},
      {RightElbow, getBounds("right_elbow")},
      {RightWristRoll, getBounds("right_wrist_roll")},
      {RightWristPitch, getBounds("right_wrist_pitch")},
      {RightWristYaw, getBounds("right_wrist_yaw")},
  }};
  } catch (const YAML::Exception& e) {
    throw std::runtime_error(std::string("LoadBounds: ") + path + ": " +
                            e.what());
  }
}
