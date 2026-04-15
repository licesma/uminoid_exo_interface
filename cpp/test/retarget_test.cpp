#include "hand_retarget/inspire/inspire_retargeter.hpp"

#include <atomic>
#include <csignal>
#include <iostream>

static std::atomic<bool> running{true};
static void signal_handler(int) { running.store(false); }

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    const std::string left_device  = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    const std::string right_device = argc > 2 ? argv[2] : "/dev/ttyUSB1";

    std::cout << "Retarget loop: left_hand=" << left_device
              << " right_hand=" << right_device << std::endl;

    InspireRetargeter retargeter(left_device, right_device);
    retargeter.retarget_loop([] { return !running.load(); });

    return 0;
}
