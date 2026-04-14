#include "hand_retarget/inspire/inspire_retargeter.hpp"
#include "manus/manus_reader.hpp"

#include <atomic>
#include <csignal>
#include <filesystem>
#include <iostream>

static std::atomic<bool> running{true};
static void signal_handler(int) { running.store(false); }

static std::string bounds_path() {
#ifdef BOUNDS_PATH
    return BOUNDS_PATH;
#else
    return (std::filesystem::path(__FILE__).parent_path() / ".." / "hand_retarget" /
            "inspire" / "inspire_retarget_bounds.yaml")
        .lexically_normal()
        .string();
#endif
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    const std::string left_device  = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    const std::string right_device = argc > 2 ? argv[2] : "/dev/ttyUSB1";
    const std::string left_addr    = argc > 3 ? argv[3] : "tcp://localhost:8002";
    const std::string right_addr   = argc > 4 ? argv[4] : "tcp://localhost:8003";

    std::cout << "Retarget loop: left_hand=" << left_device
              << " right_hand=" << right_device
              << " left_manus=" << left_addr
              << " right_manus=" << right_addr << std::endl;

    InspireRetargeter retargeter(left_device, right_device, bounds_path());
    ManusReader reader(left_addr, right_addr);
    retargeter.retarget_loop(reader, [] { return !running.load(); });

    reader.stop();
    return 0;
}
