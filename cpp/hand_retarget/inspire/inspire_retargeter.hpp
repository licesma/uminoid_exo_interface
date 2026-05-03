#pragma once

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

struct InspireFeedback {
    opt<InspirePose> actual;
    opt<InspirePose> force;
};


class InspireRetargeter {
public:
    InspireRetargeter(
        bool left_enabled, bool right_enabled,
        const std::string& recording_label,
        const std::function<void(const std::string&)>& raise_error
    );
    virtual ~InspireRetargeter() = default;

    void retarget_loop(
        const std::function<bool()>& stop,
        const std::function<int()>&  collection_id = [] { return 0; },
        const std::function<bool()>& pause        = [] { return false; }
    );

protected:
    virtual void send(
        const opt<InspirePose>& left_target,
        const opt<InspirePose>& right_target
    ) = 0;


    virtual std::pair<InspireFeedback, InspireFeedback> read_feedback() {
        return {{}, {}};
    }

    bool left_enabled_;
    bool right_enabled_;

private:
    struct FingerBounds { double low, high; };
    struct HandBounds {
        FingerBounds index, middle, ring, pinky, thumb;
    };

    opt<InspirePose> retarget(const opt<ManusHand>& hand, HandSide side) const;

    static double scale(float value, double low, double high);
    static HandBounds load_bounds(const YAML::Node& node);

    HandBounds left_bounds_;
    HandBounds right_bounds_;
    std::string recording_label_;
    ManusReader manus_;
    CsvSaver hand_csv_;
};
