#include "manus/manus_reader.hpp"

#include <csignal>
#include <iostream>

static std::atomic<bool> running{true};

static void signal_handler(int) { running.store(false); }

static void print_finger(const char* name, const ManusFinger& f) {
    std::cout << "  " << name
              << ": pt=" << f.pinky_to_thumb
              << " pb=" << f.palm_to_back
              << " wt=" << f.wrist_to_tip;
}

static void print_hand(const char* label, const ManusHand& h) {
    std::cout << label;
    print_finger("thumb", h.thumb);
    print_finger("index", h.index);
    print_finger("middle", h.middle);
    print_finger("ring", h.ring);
    print_finger("pinky", h.pinky);
}

int main(int argc, char const* argv[]) {
    const std::string left_addr  = argc >= 2 ? argv[1] : "tcp://localhost:8002";
    const std::string right_addr = argc >= 3 ? argv[2] : "tcp://localhost:8003";

    std::cout << "Manus reader: left=" << left_addr << " right=" << right_addr << std::endl;

    std::signal(SIGINT, signal_handler);

    ManusReader reader(left_addr, right_addr);

    while (running.load()) {
        auto [left, right] = reader.wait_for_next();

        if (left) {
            print_hand("L:", *left);
        }
        if (right) {
            print_hand("  R:", *right);
        }
        std::cout << std::endl;
    }

    reader.stop();
    return 0;
}
