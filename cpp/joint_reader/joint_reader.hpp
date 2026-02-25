#pragma once

#include "serial_value_reader.hpp"  // same directory
#include <string>

const double RADIAN_VAL = 0.001534;

/**
 * Higher-level wrapper around SerialValueReader (e.g. for encoder input).=
 */
class JointReader {
 public:
  explicit JointReader(const std::string& device_path,
                        double default_value = 0.0)
      : serial_(device_path, default_value) {}

  double Eval() const { return (serial_.Eval()-3400)*RADIAN_VAL; }

  bool IsOk() const { return serial_.IsOk(); }

 private:
  SerialValueReader serial_;
};
