#pragma once

#include "../constants.hpp"

#include <array>
#include <cstdint>
#include <optional>

/** Common data returned by any JointReader implementation. */
struct JointLine {
  uint64_t timestamp;
  std::array<uint16_t, JOINT_COUNT> data;
};

/**
 * Abstract interface for reading joint data.
 * Implementations: AS5600Reader (TCP relay), DynamixelReader (USB/U2D2).
 */
class JointReader {
 public:
  virtual ~JointReader() = default;

  JointReader(const JointReader&) = delete;
  JointReader& operator=(const JointReader&) = delete;

  virtual bool IsOk() const = 0;
  virtual void Stop() = 0;
  virtual std::optional<JointLine> GetNextLine() = 0;

 protected:
  JointReader() = default;
};
