#pragma once

#include "skeleton_arm.hpp"

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
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

  void Stop();

 private:
  void ReaderLoop();

  std::unique_ptr<SkeletonArm> arm_;
  std::thread thread_;

  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  ArmLine latest_{};
  uint64_t frame_seq_{0};
  uint64_t consumed_seq_{0};
  bool stopped_{false};
};
