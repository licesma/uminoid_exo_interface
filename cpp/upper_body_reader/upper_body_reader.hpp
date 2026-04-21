#pragma once

#include "arm_reader/arm_reader.hpp"

#include <functional>
#include <string>

class UpperBodyReader {
 public:
  /** Construct with AS5600Arms (TCP relay). */
  explicit UpperBodyReader(
      const std::string& relay_address, const std::string& recording_label = "",
      double default_value = 0.0);

  /** Construct with DynamixelArms (USB/U2D2). Empty string disables that arm. */
  UpperBodyReader(
      const std::string& left_device, const std::string& right_device,
      int baudrate, const std::string& recording_label = "",
      const std::function<void(const std::string&)>& raise_error = nullptr);

  ~UpperBodyReader() = default;

  void PrintRaw() const;

  void collect_loop(const std::function<int()>&  collection_id,
                    const std::function<bool()>& stop,
                    const std::function<bool()>& pause = [] { return false; });

  ArmReader left;
  ArmReader right;
};
