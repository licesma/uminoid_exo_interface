#include "hand_retarget/inspire/inspire_retargeter.hpp"
#include "manus/manus_hand.hpp"
#include "utils/repo_constants.hpp"

#include "utils/time.hpp"
#include "utils/type.hpp"

#include <algorithm>
#include <filesystem>
#include <optional>

namespace {

static const std::string BOUNDS_PATH =
    (std::filesystem::path(__FILE__).parent_path() / "inspire_retarget_bounds.yaml")
        .lexically_normal()
        .string();

constexpr int INSPIRE_SERIAL_TIMEOUT_MS = 200;
constexpr int INSPIRE_MAX_CONSECUTIVE_TIMEOUTS = 5;


std::string csv_header() {
    return "collection_id,host_timestamp,"
           "left_pinky,left_ring,left_middle,left_index,left_thumb_bend,left_thumb_rotation,"
           "right_pinky,right_ring,right_middle,right_index,right_thumb_bend,right_thumb_rotation";
}

std::string format_line(int collection_id,
                        int64_t timestamp,
                        const opt<InspirePose>& left,
                        const opt<InspirePose>& right) {
    std::string s = std::to_string(collection_id) + "," + std::to_string(timestamp);
    auto append_pose = [&](const opt<InspirePose>& p) {
        for (int i = 0; i < 6; ++i)
            s += p ? ("," + std::to_string((*p)(i))) : ",null";
    };
    append_pose(left);
    append_pose(right);
    return s;
}

}  // namespace

InspireRetargeter::InspireRetargeter(
    uint8_t left_id,
    uint8_t right_id,
    const std::string& recording_label,
    const std::function<void(const std::string&)>& raise_error
)
    : InspireRetargeter(inspire_port_resolver::resolve(left_id, right_id, raise_error),
                        left_id, right_id, recording_label, raise_error)
{}

InspireRetargeter::InspireRetargeter(
    const inspire_port_resolver::Assignment& ports,
    uint8_t left_id,
    uint8_t right_id,
    const std::string& recording_label,
    const std::function<void(const std::string&)>& raise_error
)
    : left_serial_(std::make_shared<SerialPort>(ports.left_device, raise_error)),
      right_serial_(std::make_shared<SerialPort>(ports.right_device, raise_error)),
      left_hand_(left_serial_, left_id),
      right_hand_(right_serial_, right_id),
      recording_label_(recording_label),
      manus_(manus_defaults::LEFT_ADDRESS, manus_defaults::RIGHT_ADDRESS, raise_error)
{
    YAML::Node config = YAML::LoadFile(BOUNDS_PATH);
    left_bounds_  = load_bounds(config["left"]);
    right_bounds_ = load_bounds(config["right"]);

    // SetVelocity errors propagate via raise_error wired into SerialPort.
    left_hand_.SetVelocity(1000, 1000, 1000, 1000, 1000, 1000);
    right_hand_.SetVelocity(1000, 1000, 1000, 1000, 1000, 1000);

    if (!recording_label_.empty()) {
        const std::string dir = repo_constants::DATA_DIR + "/" + recording_label_;
        hand_csv_ = CsvSaver(dir + "/inspire_hand.csv", csv_header());
    }
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

opt<InspirePose> InspireRetargeter::retarget(const opt<ManusHand>& hand, HandSide side) const {
    if(hand){
        const auto& bounds = (side == HandSide::LEFT) ? left_bounds_ : right_bounds_;

        InspirePose out;
        out(0) = scale(hand->pinky.wrist_to_tip,   bounds.pinky.low,  bounds.pinky.high);
        out(1) = scale(hand->ring.wrist_to_tip,    bounds.ring.low,   bounds.ring.high);
        out(2) = scale(hand->middle.wrist_to_tip,  bounds.middle.low, bounds.middle.high);
        out(3) = scale(hand->index.wrist_to_tip,   bounds.index.low,  bounds.index.high);
        out(4) = scale(hand->thumb.pinky_to_thumb, bounds.thumb.low,  bounds.thumb.high);
        out(5) = 1.0;  // thumb_rotation
        return out;
    }
   return std::nullopt;
}

void InspireRetargeter::retarget_loop(
    const std::function<bool()>& stop,
    const std::function<int()>&  collection_id,
    const std::function<bool()>& pause
) {
    while (auto pose = manus_.wait_for_next(stop)) {
        auto& [left, right] = *pose;

        opt<InspirePose> left_target  = retarget(*left,  HandSide::LEFT);
        opt<InspirePose> right_target = retarget(*right, HandSide::RIGHT);

        // SetPosition errors propagate via raise_error wired into SerialPort.
        left_target && left_hand_.SetPosition(*left_target);
        right_target && right_hand_.SetPosition(*right_target);

        if (!pause() && hand_csv_) {
            hand_csv_.write_line(
                format_line(collection_id(), Time::ts(), left_target, right_target));
        }
    }

    hand_csv_.close();
}
