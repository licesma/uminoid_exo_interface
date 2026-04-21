#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>

#include "g1Controller.hpp"
#include "upper_body_reader/arm_reader/arm_reader.hpp"

class G1TeleopController {
 public:
  G1TeleopController(
      std::string networkInterface, const std::string& relay_address,
      bool isSimulation, const std::string& recording_label = "",
      std::function<int()> collection_id_fn = [] { return 0; });
  G1TeleopController(
      std::string networkInterface, const std::string& left_device,
      const std::string& right_device, int baudrate, bool isSimulation,
      const std::string& recording_label = "",
      std::function<int()> collection_id_fn = [] { return 0; },
      const std::function<void(const std::string&)>& raise_error = nullptr);
  ~G1TeleopController();

 private:
  void run();
  void arm_listener_loop(ArmReader& reader, bool from_left);

  G1Controller controller_;
  ArmReader left_;
  ArmReader right_;
  std::atomic<bool> stop_requested_;
  std::thread control_thread_;
};
