#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <yaml-cpp/yaml.h>

#include "g1/g1Controller.hpp"

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: right_elbow_swing network_interface" << std::endl;
    exit(0);
  }

  YAML::Node config = YAML::LoadFile(CONFIG_PATH);
  std::string serialDevice = config["serial_device"].as<std::string>();
  bool isSimulation = config["is_simulation"].as<bool>();

  std::string networkInterface = argv[1];
  G1Controller custom(networkInterface, serialDevice, isSimulation);

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
