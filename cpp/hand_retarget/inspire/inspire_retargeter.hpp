#pragma once

#include "inspire.h"
#include "SerialPort.h"
#include "manus/manus_reader.hpp"

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <string>
#include <yaml-cpp/yaml.h>

#include "utils/csv_saver.hpp"
#include "utils/type.hpp"

using InspirePose = Eigen::Matrix<double, 6, 1>;

class InspireRetargeter {
public:
    InspireRetargeter(
        const std::string& left_device,
        const std::string& right_device,
        const std::string& recording_label,
        uint8_t id = 1
    );

    void retarget_loop(
        const std::function<int()>& collection_id,
        const std::function<bool()>& stop
    );

private:
    opt<InspirePose> retarget(const opt<ManusHand>& hand, HandSide side) const;
    struct FingerBounds { double low, high; };
    struct HandBounds {
        FingerBounds index, middle, ring, pinky, thumb;
    };

    static double scale(float value, double low, double high);
    CsvSaver make_recording_csv(int collection_id) const;
    static HandBounds load_bounds(const YAML::Node& node);

    HandBounds left_bounds_;
    HandBounds right_bounds_;
    SerialPort::SharedPtr left_serial_;
    SerialPort::SharedPtr right_serial_;
    inspire::InspireHand left_hand_;
    inspire::InspireHand right_hand_;
    std::string recording_label_;
    ManusReader manus_;
    CsvSaver hand_csv_;
};
