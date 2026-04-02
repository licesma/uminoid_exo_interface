#include "hand_retarget/inspire/inspire_retargeter.hpp"

#include <algorithm>

InspireRetargeter::InspireRetargeter(const std::string& device, const std::string& bounds_path, uint8_t id)
    : serial_(std::make_shared<SerialPort>(device, B115200, 200)),
      hand_(serial_, id)
{
    YAML::Node config = YAML::LoadFile(bounds_path);
    left_bounds_  = load_bounds(config["left"]);
    right_bounds_ = load_bounds(config["right"]);
}

double InspireRetargeter::scale(float value, double low, double high) {
    return std::clamp((value - low) / (high - low), 0.0, 1.0);
}

InspireRetargeter::HandBounds InspireRetargeter::load_bounds(const YAML::Node& node) {
    HandBounds b;
    b.index  = {node["index"]["low"].as<double>(),  node["index"]["high"].as<double>()};
    b.middle = {node["middle"]["low"].as<double>(), node["middle"]["high"].as<double>()};
    b.ring   = {node["ring"]["low"].as<double>(),   node["ring"]["high"].as<double>()};
    b.pinky  = {node["pinky"]["low"].as<double>(),  node["pinky"]["high"].as<double>()};
    b.thumb  = {node["thumb"]["low"].as<double>(),  node["thumb"]["high"].as<double>()};
    return b;
}

Eigen::Matrix<double, 6, 1> InspireRetargeter::retarget(const ManusHand& hand, HandSide side) const {
    const auto& bounds = (side == HandSide::LEFT) ? left_bounds_ : right_bounds_;

    Eigen::Matrix<double, 6, 1> out;
    out(0) = scale(hand.pinky.wrist_to_tip,   bounds.pinky.low,  bounds.pinky.high);
    out(1) = scale(hand.ring.wrist_to_tip,    bounds.ring.low,   bounds.ring.high);
    out(2) = scale(hand.middle.wrist_to_tip,  bounds.middle.low, bounds.middle.high);
    out(3) = scale(hand.index.wrist_to_tip,   bounds.index.low,  bounds.index.high);
    out(4) = scale(hand.thumb.pinky_to_thumb, bounds.thumb.low,  bounds.thumb.high);
    out(5) = 1.0;  // thumb_rotation
    return out;
}

void InspireRetargeter::send(const Eigen::Matrix<double, 6, 1>& target) {
    hand_.SetPosition(target);
}
