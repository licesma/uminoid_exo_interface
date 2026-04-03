#pragma once

#include "inspire.h"
#include "SerialPort.h"
#include "manus/manus_hand.hpp"

#include <cstdint>
#include <condition_variable>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <yaml-cpp/yaml.h>


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
    struct LoggedHandSample {
        int64_t hand_timestamp;
        opt<InspirePose> left_target;
        opt<InspirePose> right_target;
    };

    static double scale(float value, double low, double high);
    static std::ofstream get_recording_csv(const std::string& recording_name);
    static HandBounds load_bounds(const YAML::Node& node);
    void record_loop(const std::string& recording_name);

    HandBounds left_bounds_;
    HandBounds right_bounds_;
    SerialPort::SharedPtr left_serial_;
    SerialPort::SharedPtr right_serial_;
    inspire::InspireHand left_hand_;
    inspire::InspireHand right_hand_;
    std::ofstream hand_csv_;
    std::deque<LoggedHandSample> queued_samples_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    bool writer_done_{false};
};
