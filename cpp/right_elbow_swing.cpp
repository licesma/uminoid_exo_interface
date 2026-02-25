#include <iostream>
#include <string>

#include "g1/g1Controller.hpp"

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: right_elbow_swing network_interface" << std::endl;
    exit(0);
  }
  std::string networkInterface = argv[1];
  G1Controller custom(networkInterface);

  const double step_rad = 0.1;
  std::cout << "Right elbow: start 0 rad. 'u' = +0.1 rad, 'd' = -0.1 rad.\n";

  std::string input;
  while (std::cin >> input) {
    if (input == "up" || input == "u") {
      custom.AddElbowRad(step_rad);
      std::cout << "moving 0.1 rad up (target: " << custom.GetElbowTargetRad() << " rad)\n";
    } else if (input == "d") {
      custom.AddElbowRad(-step_rad);
      std::cout << "moving 0.1 rad down (target: " << custom.GetElbowTargetRad() << " rad)\n";
    }
  }
  return 0;
}
