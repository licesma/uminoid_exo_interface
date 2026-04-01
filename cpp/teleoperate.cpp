#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include <yaml-cpp/yaml.h>

#include "g1/g1Controller.hpp"
#include "upper_body_reader/upper_body_reader.hpp"

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: right_elbow_swing network_interface" << std::endl;
    exit(0);
  }

  YAML::Node config = YAML::LoadFile(CONFIG_PATH);
  std::string relay = config["relay"].as<std::string>();
  bool isSimulation = config["is_simulation"].as<bool>();

  std::string networkInterface = argv[1];
  //auto jointReader = std::make_unique<UpperBodyReader>(relay);
  const std::string left_device = "/dev/ttyUSB0";
  const std::string right_device = "";
  const int baudrate = 1000000;

  std::cout << "Starting Dynamixel reader on " << left_device << " + "
            << right_device << " at " << baudrate << " baud" << std::endl;

  auto  jointReader = std::make_unique<UpperBodyReader>(left_device, right_device, baudrate);
  G1Controller custom(networkInterface, std::move(jointReader), isSimulation);

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
