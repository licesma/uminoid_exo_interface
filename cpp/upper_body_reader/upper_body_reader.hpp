#pragma once

#include "../g1/model/g1Enums.hpp"
#include "../utils/bounds_loader.hpp"
#include "constants.hpp"
#include "joint_reader/joint_reader.hpp"

#include <array>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

/** Snapshot of all 14 exoskeleton encoder readings, keyed by robot joint index. */
struct G1JointReading {
  G1JointIndex joint;
  double netAngle;
  bool is_valid;
};

using UpperBodyReadings = std::array<G1JointReading, 14>;

class UpperBodyReader {
 public:
  /** Construct with an AS5600Reader (TCP relay). */
  explicit UpperBodyReader(
      const std::string& relay_address, double default_value = 0.0,
      const std::string& bounds_path =
          "../upper_body_reader/joint_reader/as5600_bounds.yaml");

  /** Construct with a DynamixelReader (USB/U2D2). */
  UpperBodyReader(
      const std::string& device, int baudrate,
      const std::string& bounds_path =
          "../upper_body_reader/joint_reader/as5600_bounds.yaml");

  ~UpperBodyReader();

  static G1JointIndex getG1JointIndex(ExoIndex j) {
    return static_cast<G1JointIndex>(static_cast<int>(j) + 15);
  }

  void PrintRaw() const;

  JointLine Snapshot() const {
    std::lock_guard<std::mutex> lock(values_mutex_);
    return latest_;
  }

  std::optional<JointLine> WaitNextSnapshot();

  UpperBodyReadings Eval() const;

 private:
  void ReaderLoop();

  std::unique_ptr<JointReader> reader_;
  JointBounds bounds_;
  mutable std::condition_variable values_cv_;
  mutable std::mutex values_mutex_;
  JointLine latest_{};  // Guarded by values_mutex_
  uint64_t frame_seq_{0};  // Guarded by values_mutex_
  uint64_t consumed_seq_{0};  // Guarded by values_mutex_
  bool stopped_{false};  // Guarded by values_mutex_
  std::thread thread_;
};
