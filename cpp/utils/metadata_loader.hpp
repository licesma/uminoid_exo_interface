#pragma once
#include "../g1/model/g1Enums.hpp"
#include "../g1/model/g1Structs.hpp"
#include <array>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

struct ReadingMetadata {
  bool alignedWithRobot;
};

struct JointsReadingMetadata {
  std::array<ReadingMetadata, G1_NUM_MOTOR> aligned_with_robot{};

  ReadingMetadata& operator[](G1JointIndex idx) { return aligned_with_robot[idx]; }
  const ReadingMetadata& operator[](G1JointIndex idx) const { return aligned_with_robot[idx]; }
};

/** Load per-joint metadata from a YAML file (e.g. upperBodyReaderBounds.yaml). */
inline JointsReadingMetadata LoadMetadata(const std::string& path) {
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
