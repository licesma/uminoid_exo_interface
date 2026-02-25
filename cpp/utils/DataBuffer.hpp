#pragma once

#include <memory>
#include <shared_mutex>

/**
 * Thread-safe single-slot buffer. Writers call SetData(), readers call
 * GetData() which returns a shared_ptr to a snapshot (or nullptr if empty).
 */
template <typename T>
class DataBuffer {
 public:
  void SetData(const T &newData) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    data_ = std::make_shared<T>(newData);
  }

  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return data_ ? data_ : nullptr;
  }

  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    data_ = nullptr;
  }

 private:
  std::shared_ptr<T> data_;
  std::shared_mutex mutex_;
};
