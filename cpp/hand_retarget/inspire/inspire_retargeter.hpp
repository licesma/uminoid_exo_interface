#pragma once

#include "hand_retarget/inspire/inspire_port_resolver.hpp"
#include "inspire.h"
#include "SerialPort.h"
#include "manus/manus_reader.hpp"

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <optional>
#include <string>
#include <yaml-cpp/yaml.h>

#include "utils/csv_saver.hpp"
#include "utils/type.hpp"

using InspirePose = Eigen::Matrix<double, 6, 1>;

class InspireRetargeter {
public:
    // Resolves which /dev/ttyUSB* port hosts which hand by probing HAND_IDs.
    // Pass left_enabled/right_enabled = false to skip a side entirely (no
    // port probe, no SetVelocity/SetPosition calls, no error if missing).
    InspireRetargeter(
        bool left_enabled,  uint8_t left_id,
        bool right_enabled, uint8_t right_id,
        const std::string& recording_label = "",
        const std::function<void(const std::string&)>& raise_error = nullptr
    );

    void retarget_loop(
        const std::function<bool()>& stop,
        const std::function<int()>&  collection_id = [] { return 0; },
        const std::function<bool()>& pause        = [] { return false; }
    );

private:
    // Delegated-to by the public constructor; lets us resolve port paths first.
    InspireRetargeter(
        const inspire_port_resolver::Assignment& ports,
        bool left_enabled,  uint8_t left_id,
        bool right_enabled, uint8_t right_id,
        const std::string& recording_label,
        const std::function<void(const std::string&)>& raise_error
    );

    opt<InspirePose> retarget(const opt<ManusHand>& hand, HandSide side) const;
    struct FingerBounds { double low, high; };
    struct HandBounds {
        FingerBounds index, middle, ring, pinky, thumb;
    };

    static double scale(float value, double low, double high);
    static HandBounds load_bounds(const YAML::Node& node);

    HandBounds left_bounds_;
    HandBounds right_bounds_;
    SerialPort::SharedPtr left_serial_;
    SerialPort::SharedPtr right_serial_;
    std::optional<inspire::InspireHand> left_hand_;
    std::optional<inspire::InspireHand> right_hand_;
    std::string recording_label_;
    ManusReader manus_;
    CsvSaver hand_csv_;
};
