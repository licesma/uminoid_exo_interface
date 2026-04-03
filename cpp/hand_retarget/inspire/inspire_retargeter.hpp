#pragma once

#include "inspire.h"
#include "SerialPort.h"
#include "manus/manus_hand.hpp"

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <string>
#include <yaml-cpp/yaml.h>

#include "utils/csv_saver.hpp"
#include "utils/type.hpp"

using InspirePose = Eigen::Matrix<double, 6, 1>;

class ManusReader;

class InspireRetargeter {
public:
    InspireRetargeter(
        const std::string& left_device,
        const std::string& right_device,
        const std::string& bounds_path,
        uint8_t id = 1
    );

    void retarget_loop(
        ManusReader& reader,
        const std::function<bool()>& stop_requested,
        const std::string& recording_name = ""
    );

private:
    opt<InspirePose> retarget(const opt<ManusHand>& hand, HandSide side) const;
    struct FingerBounds { double low, high; };
    struct HandBounds {
        FingerBounds index, middle, ring, pinky, thumb;
    };

    static double scale(float value, double low, double high);
    static CsvSaver make_recording_csv(const std::string& recording_name);
    static HandBounds load_bounds(const YAML::Node& node);

    HandBounds left_bounds_;
    HandBounds right_bounds_;
    SerialPort::SharedPtr left_serial_;
    SerialPort::SharedPtr right_serial_;
    inspire::InspireHand left_hand_;
    inspire::InspireHand right_hand_;
    CsvSaver hand_csv_;
};
