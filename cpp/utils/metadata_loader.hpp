#pragma once
#include "../g1/model/g1Enums.hpp"
#include "../g1/model/g1Structs.hpp"
#include <array>
#include <string>

struct ReadingMetadata {
  bool alignedWithRobot;
};

struct JointsReadingMetadata {
  std::array<ReadingMetadata, G1_NUM_MOTOR> aligned_with_robot{};

  ReadingMetadata& operator[](G1JointIndex idx) { return aligned_with_robot[idx]; }
  const ReadingMetadata& operator[](G1JointIndex idx) const { return aligned_with_robot[idx]; }
};

/** Load per-joint metadata from a YAML file (e.g. upperBodyReaderBounds.yaml). */
JointsReadingMetadata LoadMetadata(const std::string& path);
