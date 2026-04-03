#include "arm_reader.hpp"
#include "utils/csv_saver.hpp"

#include <string>

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

std::optional<ArmLine> ArmReader::wait_for_next() {
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait(lock, [&] { return stopped_ || frame_seq_ > consumed_seq_; });

  if (frame_seq_ > consumed_seq_) {
    consumed_seq_ = frame_seq_;
    return latest_;
  }
  return std::nullopt;
}

std::string ArmReader::csv_header() {
  std::string h = "timestamp,host_timestamp";
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i)
    h += ",joint_" + std::to_string(i);
  return h;
}

std::string ArmReader::format_line(const ArmLine& line) {
  std::string s = std::to_string(line.timestamp) + "," +
                  std::to_string(line.host_timestamp);
  for (const auto v : line.data) s += "," + std::to_string(v);
  return s;
}

void ArmReader::collect_loop(const std::string& csv_path,
                             const std::function<bool()>& stop_requested) {
  CsvSaver csv(csv_path, csv_header());

  while (!stop_requested()) {
    auto reading = wait_for_next();
    if (!reading) break;
    csv.write_line(format_line(*reading));
  }

  csv.close();
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
