#pragma once

#include "../utils/bounds_loader.hpp"
#include "constants.hpp"
#include "../g1/model/g1Enums.hpp"
#include <array>
#include "serial_reader.hpp"
#include <string>
#include <mutex>
#include <thread>

/** Snapshot of all 14 exoskeleton encoder readings, keyed by robot joint index. */
struct G1JointReading {
  G1JointIndex joint;
  double netAngle;
  bool is_valid;
};

using G1UpperReadings = std::array<G1JointReading, 14>;

class JointManager {
 public:
  explicit JointManager(const std::string& relay_address,
                      double default_value = 0.0,
                      const std::string& bounds_path =
                          "../joint_reader/upperBodyReaderBounds.yaml")
      : bounds_(LoadBounds(bounds_path)), reader_(relay_address){
        if (!reader_.IsOk()) return;

        thread_ = std::thread(&JointManager::ReaderLoop, this);
      }
  ~JointManager();


  static G1JointIndex getG1JointIndex(ExoIndex j){
    return static_cast<G1JointIndex>(static_cast<int>(j) + 15);
  }

  void PrintRaw() const;

  SerialLine Snapshot() const {
    std::lock_guard<std::mutex> lock(values_mutex_);
    return latest_;
  }

  G1UpperReadings Eval() const;

 private:
  void ReaderLoop();

  SerialReader reader_;
  JointBounds bounds_;
  mutable std::mutex values_mutex_;
  SerialLine latest_{}; // Guarded by values_mutex_
  std::thread thread_;
};
