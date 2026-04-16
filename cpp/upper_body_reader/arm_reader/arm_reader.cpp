#include "arm_reader.hpp"

#include <string>

namespace {

std::string csv_header() {
  std::string h = "collection_id,timestamp,host_timestamp";
  for (size_t i = 0; i < ARM_JOINT_COUNT; ++i)
    h += ",joint_" + std::to_string(i);
  return h;
}

std::string format_line(int collection_id, const ArmLine& line) {
  std::string s = std::to_string(collection_id) + "," +
                  std::to_string(line.timestamp) + "," +
                  std::to_string(line.host_timestamp);
  for (const auto v : line.data) s += "," + std::to_string(v);
  return s;
}

}  // namespace

ArmReader::ArmReader(std::unique_ptr<SkeletonArm> arm,
                     const std::string& csv_path,
                     const std::function<void(const std::string&)>& raise_error)
    : arm_(std::move(arm)),
      raise_error_(raise_error),
      csv_(csv_path.empty() ? CsvSaver{} : CsvSaver(csv_path, csv_header())) {
  if (!arm_) {
    stopped_ = true;
    return;
  }
  thread_ = std::thread(&ArmReader::read_loop, this);
}

ArmReader::~ArmReader() {
  stop();
}

void ArmReader::stop() {
  if (arm_) arm_->Stop();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
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


void ArmReader::collect_loop(const std::function<int()>& collection_id,
                             const std::function<bool()>& stop) {
  while (!stop()) {
    auto reading = wait_for_next();
    if (!reading) break;
    csv_.write_line(format_line(collection_id(), *reading));
  }

  csv_.close();
}

void ArmReader::read_loop() {
  while (auto frame = arm_->GetNextLine(raise_error_)) {
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
