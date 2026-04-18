#include "inspire.h"
#include "SerialPort.h"

#include <atomic>
#include <csignal>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

static std::atomic<bool> running{true};
static void signal_handler(int) { running.store(false); }
using InspireFinger = Eigen::Matrix<double, 6, 1>;

int main(int argc, char** argv) {
    const std::string device = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    std::cout << "Inspire hand on " << device << std::endl;

    auto serial = std::make_shared<SerialPort>(device);
    inspire::InspireHand hand(serial, 1);

    std::signal(SIGINT, signal_handler);

    // Order: pinky, ring, middle, index, thumb_bend, thumb_rotation
    InspireFinger target;
    target << 1.0, 1.0, 1.0, 0.5, 1.0, 1.0;

    hand.SetPosition(target);

    while (running.load()) {
        InspireFinger pos;
        int err = hand.GetPosition(pos);
        if (err == 0) {
            std::cout << "pos: " << pos.transpose() << std::endl;
        } else {
            std::cout << "read error: " << err << std::endl;
        }
        usleep(200000);
    }

    return 0;
}
