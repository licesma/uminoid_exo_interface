#include <atomic>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>

#include "upper_body_reader/g1_upper_body_reader.hpp"

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }
}  // namespace

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: right_elbow_swing network_interface" << std::endl;
    exit(0);
  }

  YAML::Node config = YAML::LoadFile(CONFIG_PATH);
  const std::string collect_config_path =
      (std::filesystem::path(__FILE__).parent_path() / "collect_config.yaml")
          .lexically_normal()
          .string();
  YAML::Node collect_config = YAML::LoadFile(collect_config_path);
  std::string relay = config["relay"].as<std::string>();
  bool isSimulation = config["is_simulation"].as<bool>();

  std::string networkInterface = argv[1];
  // G1TeleopController can also be constructed from a relay address.
  const std::string left_device =
      collect_config["upper_body"]["left_device"].as<std::string>();
  const std::string right_device =
      collect_config["upper_body"]["right_device"].as<std::string>();
  const int baudrate = collect_config["upper_body"]["baudrate"].as<int>();

  if (left_device.empty() || right_device.empty()) {
    std::cerr << "teleoperate requires both left and right arm devices"
              << std::endl;
    return 1;
  }

  std::cout << "Starting Dynamixel reader on " << left_device << " + "
            << right_device << " at " << baudrate << " baud" << std::endl;

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  G1UpperBodyReader custom(networkInterface, left_device, right_device,
                           baudrate, isSimulation);
  custom.collect_loop([] { return 0; }, [] { return g_stop.load(); });

  return 0;
}
