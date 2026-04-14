#include "hand_retarget/inspire/inspire_retargeter.hpp"
#include "manus/manus_reader.hpp"
#include "utils/time.hpp"

#include <atomic>
#include <csignal>
#include <iostream>
#include <thread>

static std::atomic<bool> running{true};
static void signal_handler(int) { running.store(false); }

int main(int argc, char** argv) {
    const std::string left_device  = argc > 1 ? argv[1] : "/dev/ttyUSB1";
    const std::string right_device = argc > 2 ? argv[2] : "/dev/ttyUSB2";
    const std::string recording_name = argc > 3 ? argv[3] : "";

    InspireRetargeter inspire(left_device, right_device);
    ManusReader manus;

    std::signal(SIGINT, signal_handler);
    std::cout << "Manus -> Inspire retarget running";
    if (!recording_name.empty()) {
        std::cout << " with recording \"" << recording_name << "\"";
    }
    std::cout << ". Press q to quit." << std::endl;

    std::thread([&manus] {
        char key = 0;
        while (running.load() && std::cin.get(key)) {
            if (key == 'q' || key == 'Q') {
                running.store(false);
                manus.stop();
                break;
            }
        }
    }).detach();

    inspire.retarget_loop(
        manus,
        [] { return !running.load(); },
        recording_name
    );

    manus.stop();
    return 0;
}
