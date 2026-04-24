#pragma once

#include "hand_retarget/inspire/inspire_retargeter.hpp"

#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <functional>
#include <memory>
#include <mutex>
#include <string>

/**
 * Inspire retargeter that drives the hand(s) through the G1 via DDS.
 *
 * Publishes 12-joint MotorCmds_ on rt/inspire/cmd (right-hand slots 0-5,
 * left-hand slots 6-11) and subscribes to the echoed state on
 * rt/inspire/state. The actual serial bridge (inspire_g1) must be running
 * on the robot; see run_inspire.md.
 */
class G1InspireRetargeter : public InspireRetargeter {
public:
    G1InspireRetargeter(
        bool left_enabled,
        bool right_enabled,
        const std::string& network_interface,
        const std::string& recording_label = "",
        const std::function<void(const std::string&)>& raise_error = nullptr
    );

protected:
    void send(const opt<InspirePose>& left_target,
              const opt<InspirePose>& right_target) override;

    std::pair<InspireFeedback, InspireFeedback> read_feedback() override;

private:
    // Finger order per slot: pinky, ring, middle, index, thumb_bend, thumb_rotation.
    static constexpr int kFingersPerHand   = 6;
    static constexpr int kRightHandOffset  = 0;
    static constexpr int kLeftHandOffset   = 6;
    static constexpr int kTotalHandJoints  = 12;

    std::shared_ptr<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>>   cmd_pub_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>> state_sub_;

    std::mutex                           state_mtx_;
    unitree_go::msg::dds_::MotorStates_  state_;
    unitree_go::msg::dds_::MotorCmds_    cmd_;
};
