#include "serial_manager.hpp"

#include <mutex>

SerialManager::SerialManager(const std::string& device_path)
    : reader_(device_path) {
  if (!reader_.IsOk()) return;
  thread_ = std::thread(&SerialManager::ReaderLoop, this);
}

SerialManager::~SerialManager() {
  reader_.Stop();
  if (thread_.joinable()) thread_.join();
}

void SerialManager::ReaderLoop() {
  while (reader_.IsOk()) {
    auto frame = reader_.GetNextLine();
    if (!frame) break;
    std::lock_guard<std::mutex> lock(values_mutex_);
    latest_ = *frame;
  }
}
