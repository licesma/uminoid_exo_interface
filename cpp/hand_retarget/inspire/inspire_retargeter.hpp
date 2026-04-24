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

// Per-hand readback sampled alongside the commanded target.
//   actual: measured finger positions, same scale as commands ([0, 1]).
//   force:  per-finger fingertip force in Newtons (RH56 0.5 N resolution).
// Either may be std::nullopt if the transport can't supply it (e.g. the
// G1-bridged variant has no force channel).
struct InspireFeedback {
    opt<InspirePose> actual;
    opt<InspirePose> force;
};

/**
 * Abstract inspire-hand retargeter. Reads Manus glove poses, maps them to
 * per-finger inspire targets, and forwards them to a concrete transport.
 *
 * Implementations:
 *   - UsbInspireRetargeter: writes directly to the hands over USB serial.
 *   - G1InspireRetargeter:  publishes DDS commands picked up by the
 *                           inspire_g1 bridge running on the G1.
 */
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
    // Send commanded targets to the actual hand(s). A std::nullopt target
    // means "no command for this side this tick" (e.g. missing glove sample
    // or disabled side).
    virtual void send(
        const opt<InspirePose>& left_target,
        const opt<InspirePose>& right_target
    ) = 0;

    // Read back measured state (actual angles + fingertip forces) for logging.
    // Default: no feedback available. Overridden by transports that can poll
    // the hand directly.
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
