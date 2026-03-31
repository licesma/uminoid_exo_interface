#include "upper_body_reader/upper_body_reader.hpp"

#include <cstdlib>
#include <iostream>

#ifndef READER_BOUNDS_PATH
#define READER_BOUNDS_PATH "../upper_body_reader/joint_reader/as5600_bounds.yaml"
#endif

int main(int argc, char const* argv[]) {
  const std::string device = argc >= 2 ? argv[1] : "/dev/ttyUSB0";
  const int baudrate = argc >= 3 ? std::atoi(argv[2]) : 1000000;

  std::cout << "Starting Dynamixel reader on " << device << " at " << baudrate
            << " baud" << std::endl;

  UpperBodyReader reader(device, baudrate, READER_BOUNDS_PATH);

  while (true) {
    const auto snapshot = reader.WaitNextSnapshot();
    if (!snapshot) break;

    std::cout << snapshot->timestamp;
    for (const auto value : snapshot->data) std::cout << ' ' << value;
    std::cout << std::endl;
  }

  return EXIT_SUCCESS;
}
