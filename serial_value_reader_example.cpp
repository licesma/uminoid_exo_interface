#include "serial_value_reader.hpp"

#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char* argv[]) {
  const char* device = (argc >= 2) ? argv[1] : "/dev/ttyACM0";
  SerialValueReader reader(device, 0.0);

  if (!reader.IsOk()) {
    std::cerr << "Failed to open " << device << std::endl;
    return 1;
  }

  std::cout << "Reading values from " << device
            << ". Send lines like 0.5 or -0.1. Ctrl+C to stop.\n";

  while (true) {
    double v = reader.Eval();
    std::cout << "Eval() = " << v << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
