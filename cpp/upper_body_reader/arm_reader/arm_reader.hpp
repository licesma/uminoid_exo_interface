#pragma once

#include "skeleton_arm.hpp"

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

/**
 * Wraps a SkeletonArm with a read thread, mutex-protected latest value,
 * and blocking/non-blocking snapshot access.
 */
class ArmReader {
 public:
  /** Pass nullptr for an inactive arm. */
  explicit ArmReader(std::unique_ptr<SkeletonArm> arm = nullptr);
  ~ArmReader();

  ArmReader(const ArmReader&) = delete;
  ArmReader& operator=(const ArmReader&) = delete;

  bool IsOk() const { return arm_ && arm_->IsOk(); }

  /** Non-blocking: returns the latest reading. */
  ArmLine Snapshot() const;

  /** Blocking: waits until a new reading arrives since the last call. */
  std::optional<ArmLine> wait_for_next();

  /** Blocks on wait_for_next() in a loop, writing each reading to csv_path.
   *  Returns when the arm stops or stop_requested() returns true. */
  void collect_loop(const std::string& csv_path,
                    const std::function<bool()>& stop_requested);

  void Stop();

 private:
  void ReaderLoop();
  static std::string csv_header();
  static std::string format_line(const ArmLine& line);

  std::unique_ptr<SkeletonArm> arm_;
  std::thread thread_;

  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  ArmLine latest_{};
  uint64_t frame_seq_{0};
  uint64_t consumed_seq_{0};
  bool stopped_{false};
};
