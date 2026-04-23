#include "hand_retarget/inspire/usb_inspire_retargeter.hpp"

#include <atomic>
#include <csignal>
#include <iostream>

static std::atomic<bool> running{true};
static void signal_handler(int) { running.store(false); }

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    const uint8_t left_id  = argc > 1 ? static_cast<uint8_t>(std::stoi(argv[1])) : 1;
    const uint8_t right_id = argc > 2 ? static_cast<uint8_t>(std::stoi(argv[2])) : 2;

    std::cout << "Retarget loop: left_id=" << int(left_id)
              << " right_id=" << int(right_id) << std::endl;

    UsbInspireRetargeter retargeter(/*left_enabled=*/true,  left_id,
                                    /*right_enabled=*/true, right_id);
    retargeter.retarget_loop([] { return !running.load(); });

    return 0;
}
