#pragma once

#include <atomic>
#include <string>
#include <thread>

/**
 * Reads lines from a serial port (9600 8N1), parses each line as a double,
 * and caches the last value. Eval() returns that cached value with no I/O —
 * safe to call from a fast control loop (e.g. every 2 ms).
 *
 * Protocol: send lines like "0.5" or "-0.1\n"; the last successfully parsed
 * value is returned by Eval(). Invalid lines are ignored.
 */
class SerialValueReader {
 public:
  explicit SerialValueReader(const std::string& device_path,
                             double default_value = 0.0);
  ~SerialValueReader();

  SerialValueReader(const SerialValueReader&) = delete;
  SerialValueReader& operator=(const SerialValueReader&) = delete;

  /** Returns the last value received over serial. No I/O — safe in control loop. */
  double Eval() const { return last_value_.load(); }

  /** Override the cached value (e.g. before any serial data has arrived). */
  void SetDefaultValue(double v) { last_value_.store(v); }

  /** True if the device was opened and the reader thread is running. */
  bool IsOk() const { return ok_; }

 private:
  void ReaderLoop();

  std::string device_path_;
  std::atomic<double> last_value_;
  std::atomic<bool> running_{true};
  bool ok_ = false;
  int fd_ = -1;
  std::thread thread_;
};
