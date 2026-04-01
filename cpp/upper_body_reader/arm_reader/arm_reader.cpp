#include "arm_reader.hpp"

ArmReader::ArmReader(std::unique_ptr<SkeletonArm> arm)
    : arm_(std::move(arm)), stopped_(!IsOk()) {
  if (!stopped_)
    thread_ = std::thread(&ArmReader::ReaderLoop, this);
}

ArmReader::~ArmReader() {
  Stop();
}

void ArmReader::Stop() {
  if (arm_) arm_->Stop();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

ArmLine ArmReader::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

std::optional<ArmLine> ArmReader::WaitSnapshot() {
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait(lock, [&] { return stopped_ || frame_seq_ > consumed_seq_; });

  if (frame_seq_ > consumed_seq_) {
    consumed_seq_ = frame_seq_;
    return latest_;
  }
  return std::nullopt;
}

void ArmReader::ReaderLoop() {
  while (arm_->IsOk()) {
    auto frame = arm_->GetNextLine();
    if (!frame) break;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ = *frame;
      ++frame_seq_;
    }
    cv_.notify_all();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
  }
  cv_.notify_all();
}
