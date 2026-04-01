#pragma once

#include "../constants.hpp"

#include <array>
#include <cstdint>
#include <optional>

/** Data returned by a single-arm SkeletonArm. */
struct ArmLine {
  uint64_t timestamp;
  std::array<uint16_t, ARM_JOINT_COUNT> data;
};

/**
 * Abstract interface for reading joint data from one arm.
 * Implementations: AS5600Arm (TCP relay), DynamixelArm (USB/U2D2).
 */
class SkeletonArm {
 public:
  virtual ~SkeletonArm() = default;

  SkeletonArm(const SkeletonArm&) = delete;
  SkeletonArm& operator=(const SkeletonArm&) = delete;

  virtual bool IsOk() const = 0;
  virtual void Stop() = 0;

  /** Blocks until new data is available for this arm. */
  virtual std::optional<ArmLine> GetNextLine() = 0;

 protected:
  SkeletonArm() = default;
};
