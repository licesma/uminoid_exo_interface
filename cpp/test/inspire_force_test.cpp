/**
 * @file inspire_force_test.cpp
 * @brief Live readout of fingertip forces from the Inspire RH56 hand(s).
 *
 * Reads inspire_force_test_config.yaml to decide whether to talk to the
 * hand(s) directly over USB serial or via the G1 DDS bridge. Prints a
 * refreshed force table in-place — press a fingertip to see the
 * corresponding column rise.
 *
 * Usage:
 *   ./inspire_force_test [path/to/config.yaml]
 */

#include "hand_retarget/inspire/inspire_port_resolver.hpp"
#include "inspire.h"
#include "SerialPort.h"

#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

using Force6 = Eigen::Matrix<double, 6, 1>;

static std::atomic<bool> running{true};
static void signal_handler(int) { running.store(false); }

static const char* FINGER_NAMES[6] = {
    "pinky", "ring", "middle", "index", "thumb_b", "thumb_r",
};

struct Config {
    std::string source;              // "usb" or "g1"
    std::string network_interface;   // g1 only
    bool left_enabled, right_enabled;
    uint8_t left_id, right_id;
    double print_rate_hz;
};

static Config load_config(const std::string& path) {
    YAML::Node y = YAML::LoadFile(path);
    Config c;
    c.source             = y["inspire"]["source"].as<std::string>();
    c.network_interface  = y["inspire"]["network_interface"]
                               ? y["inspire"]["network_interface"].as<std::string>()
                               : std::string("");
    c.left_enabled       = y["inspire"]["left"]["enabled"].as<bool>();
    c.right_enabled      = y["inspire"]["right"]["enabled"].as<bool>();
    c.left_id            = y["inspire"]["left"]["id"].as<int>();
    c.right_id           = y["inspire"]["right"]["id"].as<int>();
    c.print_rate_hz      = y["print_rate_hz"] ? y["print_rate_hz"].as<double>() : 10.0;
    return c;
}

// Abstract reader: returns force (N) for left and right. std::nullopt if
// the side is disabled or the read failed on this tick.
class ForceReader {
public:
    virtual ~ForceReader() = default;
    virtual std::pair<std::optional<Force6>, std::optional<Force6>> read() = 0;
};

class UsbForceReader : public ForceReader {
public:
    UsbForceReader(const Config& c) {
        auto raise = [](const std::string& m) { std::cerr << m << std::endl; };
        auto ports = inspire_port_resolver::resolve(
            c.left_enabled  ? std::optional<uint8_t>{c.left_id}  : std::nullopt,
            c.right_enabled ? std::optional<uint8_t>{c.right_id} : std::nullopt,
            raise);

        if (c.left_enabled) {
            left_serial_ = std::make_shared<SerialPort>(ports.left_device, raise);
            left_hand_.emplace(left_serial_, c.left_id);
        }
        if (c.right_enabled) {
            right_serial_ = std::make_shared<SerialPort>(ports.right_device, raise);
            right_hand_.emplace(right_serial_, c.right_id);
        }
    }

    std::pair<std::optional<Force6>, std::optional<Force6>> read() override {
        auto poll = [](std::optional<inspire::InspireHand>& h) -> std::optional<Force6> {
            if (!h) return std::nullopt;
            Force6 f;
            return h->GetForce(f) == 0 ? std::optional<Force6>{f} : std::nullopt;
        };
        return {poll(left_hand_), poll(right_hand_)};
    }

private:
    SerialPort::SharedPtr left_serial_, right_serial_;
    std::optional<inspire::InspireHand> left_hand_, right_hand_;
};

class G1ForceReader : public ForceReader {
public:
    G1ForceReader(const Config& c) : left_enabled_(c.left_enabled), right_enabled_(c.right_enabled) {
        unitree::robot::ChannelFactory::Instance()->Init(0, c.network_interface);
        state_.states().resize(12);
        sub_ = std::make_shared<
            unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>>(
            "rt/inspire/state");
        sub_->InitChannel([this](const void* m) {
            std::lock_guard<std::mutex> lk(mtx_);
            state_ = *static_cast<const unitree_go::msg::dds_::MotorStates_*>(m);
        });
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::pair<std::optional<Force6>, std::optional<Force6>> read() override {
        std::lock_guard<std::mutex> lk(mtx_);
        // Slot layout matches G1InspireRetargeter: right 0-5, left 6-11.
        auto extract = [&](int offset, bool enabled) -> std::optional<Force6> {
            if (!enabled) return std::nullopt;
            Force6 f;
            for (int i = 0; i < 6; ++i) f(i) = state_.states()[offset + i].tau_est();
            return f;
        };
        return {extract(6, left_enabled_), extract(0, right_enabled_)};
    }

private:
    bool left_enabled_, right_enabled_;
    std::mutex mtx_;
    unitree_go::msg::dds_::MotorStates_ state_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>> sub_;
};

static void print_row(const std::string& label, const std::optional<Force6>& f) {
    std::cout << std::left << std::setw(8) << label;
    if (!f) {
        for (int i = 0; i < 6; ++i) std::cout << std::right << std::setw(10) << "  --   ";
    } else {
        std::cout << std::fixed << std::setprecision(2);
        for (int i = 0; i < 6; ++i) std::cout << std::right << std::setw(10) << (*f)(i);
    }
    std::cout << "\n";
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    const std::string default_cfg =
        (std::filesystem::path(__FILE__).parent_path() / "inspire_force_test_config.yaml")
            .lexically_normal().string();
    const std::string cfg_path = argc > 1 ? argv[1] : default_cfg;
    const Config cfg = load_config(cfg_path);

    std::cout << "Inspire force test  [source=" << cfg.source
              << "  left=" << cfg.left_enabled << "  right=" << cfg.right_enabled
              << "]\n";

    std::unique_ptr<ForceReader> reader;
    if (cfg.source == "usb")       reader = std::make_unique<UsbForceReader>(cfg);
    else if (cfg.source == "g1")   reader = std::make_unique<G1ForceReader>(cfg);
    else {
        std::cerr << "Unknown source: " << cfg.source << " (expected 'usb' or 'g1')\n";
        return 1;
    }

    const auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / cfg.print_rate_hz));

    while (running.load()) {
        auto [left, right] = reader->read();

        // Clear screen + home cursor so the table updates in place.
        std::cout << "\033[2J\033[H";
        std::cout << "Force (N)  [Ctrl+C to exit]\n\n";
        std::cout << std::left << std::setw(8) << "";
        for (int i = 0; i < 6; ++i)
            std::cout << std::right << std::setw(10) << FINGER_NAMES[i];
        std::cout << "\n";
        if (cfg.left_enabled)  print_row("left", left);
        if (cfg.right_enabled) print_row("right", right);
        std::cout << std::flush;

        std::this_thread::sleep_for(period);
    }

    std::cout << "\nDone.\n";
    return 0;
}
