#include "upper_body_reader/upper_body_reader.hpp"

#include <cstdlib>
#include <iostream>

int main(int argc, char const* argv[]) {
  const std::string relay_address = argc >= 2 ? argv[1] : "127.0.0.1:5000";

  std::cout << "Starting AS5600 reader on " << relay_address << std::endl;

  UpperBodyReader reader(relay_address);

  while (true) {
    const auto left = reader.left.wait_for_next();
    const auto right = reader.right.wait_for_next();
    if (!left && !right) break;

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
