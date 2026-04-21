#pragma once

#include "arm_reader/arm_reader.hpp"
#include "upper_body_reader.hpp"

#include <functional>
#include <string>

class ExoUpperBodyReader : public UpperBodyReader {
 public:
  /** Construct with AS5600Arms (TCP relay). */
  explicit ExoUpperBodyReader(
      const std::string& relay_address, const std::string& recording_label = "",
      double default_value = 0.0);

  /** Construct with DynamixelArms (USB/U2D2). Empty string disables that arm. */
  ExoUpperBodyReader(
      const std::string& left_device, const std::string& right_device,
      int baudrate, const std::string& recording_label = "",
      const std::function<void(const std::string&)>& raise_error = nullptr);

  ~ExoUpperBodyReader() override = default;

  void PrintRaw() const;

  void collect_loop(const std::function<int()>&  collection_id,
                    const std::function<bool()>& stop,
                    const std::function<bool()>& pause = [] { return false; })
      override;

  ArmReader left;
  ArmReader right;
};
