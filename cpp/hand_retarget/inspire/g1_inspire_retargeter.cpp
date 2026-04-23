#include "hand_retarget/inspire/g1_inspire_retargeter.hpp"

#include <chrono>
#include <thread>

G1InspireRetargeter::G1InspireRetargeter(
    bool left_enabled,
    bool right_enabled,
    const std::string& network_interface,
    const std::string& recording_label,
    const std::function<void(const std::string&)>& raise_error
)
    : InspireRetargeter(left_enabled, right_enabled, recording_label, raise_error)
{
    // ChannelFactory is a singleton; Init is idempotent for repeated calls
    // with the same interface (other components like g1 may have already
    // initialized it).
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    cmd_pub_ = std::make_shared<
        unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>>(
        "rt/inspire/cmd");
    cmd_pub_->InitChannel();

    cmd_.cmds().resize(kTotalHandJoints);
    // Start in the open position for any slot we never end up commanding.
    for (int i = 0; i < kTotalHandJoints; ++i)
        cmd_.cmds()[i].q() = 1.0f;

    state_.states().resize(kTotalHandJoints);
    state_sub_ = std::make_shared<
        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>>(
        "rt/inspire/state");
    state_sub_->InitChannel([this](const void* message) {
        std::lock_guard<std::mutex> lock(state_mtx_);
        state_ = *static_cast<const unitree_go::msg::dds_::MotorStates_*>(message);
    });

    // Give DDS discovery a moment before the first publish.
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void G1InspireRetargeter::send(
    const opt<InspirePose>& left_target,
    const opt<InspirePose>& right_target
) {
    auto fill = [&](int offset, const opt<InspirePose>& target) {
        if (!target) return;
        for (int i = 0; i < kFingersPerHand; ++i)
            cmd_.cmds()[offset + i].q() = static_cast<float>((*target)(i));
    };
    fill(kRightHandOffset, right_target);
    fill(kLeftHandOffset,  left_target);

    cmd_pub_->Write(cmd_);
}
