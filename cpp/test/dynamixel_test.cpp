#include "upper_body_reader/exo_upper_body_reader.hpp"

#include <cstdlib>
#include <iostream>
#include <optional>

int main(int argc, char const* argv[]) {
  const std::string left_device = argc >= 2 ? argv[1] : "/dev/ttyUSB0";
  const std::string right_device = argc >= 3 ? argv[2] : "/dev/ttyUSB1";
  const int baudrate = argc >= 4 ? std::atoi(argv[3]) : 1000000;

  std::cout << "Starting Dynamixel reader on " << left_device << " + "
            << right_device << " at " << baudrate << " baud" << std::endl;

  ExoUpperBodyReader reader(left_device, right_device, baudrate);

  while (true) {
    const auto left = reader.left.wait_for_next();
    const auto right = reader.right.wait_for_next();
    if (!left && !right){
      break;
    }

    if (left) {
      std::cout << "L[" << left->timestamp << "]:";
      for (const auto v : left->data) std::cout << ' ' << v;
    }
    if (right) {
      std::cout << "  R[" << right->timestamp << "]:";
      for (const auto v : right->data) std::cout << ' ' << v;
    }
    std::cout << std::endl;
  }

  return EXIT_SUCCESS;
}
