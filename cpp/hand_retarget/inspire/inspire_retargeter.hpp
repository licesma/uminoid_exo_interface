#pragma once

#include "inspire.h"
#include "SerialPort.h"
#include "manus/manus_hand.hpp"

#include <eigen3/Eigen/Dense>
#include <string>
#include <yaml-cpp/yaml.h>

class InspireRetargeter {
public:
    InspireRetargeter(const std::string& device, const std::string& bounds_path, uint8_t id = 1);

    Eigen::Matrix<double, 6, 1> retarget(const ManusHand& hand, HandSide side) const;
    void send(const Eigen::Matrix<double, 6, 1>& target);

private:
    struct FingerBounds { double low, high; };
    struct HandBounds {
        FingerBounds index, middle, ring, pinky, thumb;
    };

    static double scale(float value, double low, double high);
    static HandBounds load_bounds(const YAML::Node& node);

    HandBounds left_bounds_;
    HandBounds right_bounds_;
    SerialPort::SharedPtr serial_;
    inspire::InspireHand hand_;
};
