#include "metadata_loader.hpp"
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

JointsReadingMetadata LoadMetadata(const std::string& path) {
  try {
    YAML::Node config = YAML::LoadFile(path);

    auto getMetadata = [&config](const char* joint) -> ReadingMetadata {
      bool alignedWithRobot = config[joint]["aligned_with_robot"].as<bool>();
      return {alignedWithRobot};
    };

    JointsReadingMetadata meta;
    meta[LeftShoulderPitch]  = getMetadata("left_shoulder_pitch");
    meta[LeftShoulderRoll]   = getMetadata("left_shoulder_roll");
    meta[LeftShoulderYaw]    = getMetadata("left_shoulder_yaw");
    meta[LeftElbow]          = getMetadata("left_elbow");
    meta[LeftWristRoll]      = getMetadata("left_wrist_roll");
    meta[LeftWristPitch]     = getMetadata("left_wrist_pitch");
    meta[LeftWristYaw]       = getMetadata("left_wrist_yaw");
    meta[RightShoulderPitch] = getMetadata("right_shoulder_pitch");
    meta[RightShoulderRoll]  = getMetadata("right_shoulder_roll");
    meta[RightShoulderYaw]   = getMetadata("right_shoulder_yaw");
    meta[RightElbow]         = getMetadata("right_elbow");
    meta[RightWristRoll]     = getMetadata("right_wrist_roll");
    meta[RightWristPitch]    = getMetadata("right_wrist_pitch");
    meta[RightWristYaw]      = getMetadata("right_wrist_yaw");
    return meta;
  } catch (const YAML::Exception& e) {
    throw std::runtime_error(std::string("LoadMetadata: ") + path + ": " +
                            e.what());
  }
}
