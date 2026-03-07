#pragma once

#include "serial_reader.hpp"

#include <mutex>
#include <string>
#include <thread>

/**
 * Owns a SerialReader and a background thread that continuously calls
 * GetNextLine() and caches the latest SerialLine.
 * Snapshot() returns a coherent copy with no I/O — safe in a fast control loop.
 */
class SerialManager {
 public:
  explicit SerialManager(const std::string& relay_address);
  ~SerialManager();
  SerialManager(const SerialManager&) = delete;
  SerialManager& operator=(const SerialManager&) = delete;

  /** Returns a coherent snapshot of the latest frame. No I/O — safe in control loop. */
  SerialLine Snapshot() const {
    std::lock_guard<std::mutex> lock(values_mutex_);
    return latest_;
  }

  /** Print raw encoder values (0–4096) to stdout. */
  void PrintRaw() const;

 private:
  void ReaderLoop();

  SerialReader reader_;
  mutable std::mutex values_mutex_;
  SerialLine latest_{}; // Guarded by values_mutex_
  std::thread thread_;
};
