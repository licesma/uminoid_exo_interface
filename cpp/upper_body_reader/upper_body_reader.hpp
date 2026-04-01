#pragma once

#include "../g1/model/g1Enums.hpp"
#include "../utils/bounds_loader.hpp"
#include "../utils/metadata_loader.hpp"
#include "arm_reader/arm_reader.hpp"
#include "constants.hpp"

#include <array>
#include <string>

#ifndef AS5600_BOUNDS_PATH
#define AS5600_BOUNDS_PATH \
  "../upper_body_reader/arm_reader/as5600/as5600_bounds.yaml"
#endif

#ifndef DYNAMIXEL_BOUNDS_PATH
#define DYNAMIXEL_BOUNDS_PATH \
  "../upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
#endif

struct G1JointReading {
  G1JointIndex joint;
  double netAngle;
  bool is_valid;
};

using UpperBodyReadings = std::array<G1JointReading, JOINT_COUNT>;

class UpperBodyReader {
 public:
  /** Construct with AS5600Arms (TCP relay). */
  explicit UpperBodyReader(
      const std::string& relay_address, double default_value = 0.0);

  /** Construct with DynamixelArms (USB/U2D2). Empty string disables that arm. */
  UpperBodyReader(
      const std::string& left_device, const std::string& right_device,
      int baudrate);

  ~UpperBodyReader() = default;

  static G1JointIndex getG1JointIndex(ExoIndex j) {
    return static_cast<G1JointIndex>(static_cast<int>(j) + 15);
  }

  void PrintRaw() const;
  UpperBodyReadings Eval() const;

  JointsReadingMetadata metadata;
  ArmReader left;
  ArmReader right;

 private:
  JointBounds bounds_;
};
