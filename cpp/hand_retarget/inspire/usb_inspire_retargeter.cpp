#include "hand_retarget/inspire/usb_inspire_retargeter.hpp"

UsbInspireRetargeter::UsbInspireRetargeter(
    bool left_enabled,  uint8_t left_id,
    bool right_enabled, uint8_t right_id,
    const std::string& recording_label,
    const std::function<void(const std::string&)>& raise_error
)
    : UsbInspireRetargeter(
          inspire_port_resolver::resolve(
              left_enabled  ? std::optional<uint8_t>{left_id}  : std::nullopt,
              right_enabled ? std::optional<uint8_t>{right_id} : std::nullopt,
              raise_error),
          left_enabled,  left_id,
          right_enabled, right_id,
          recording_label, raise_error)
{}

UsbInspireRetargeter::UsbInspireRetargeter(
    const inspire_port_resolver::Assignment& ports,
    bool left_enabled,  uint8_t left_id,
    bool right_enabled, uint8_t right_id,
    const std::string& recording_label,
    const std::function<void(const std::string&)>& raise_error
)
    : InspireRetargeter(left_enabled, right_enabled, recording_label, raise_error),
      left_serial_(left_enabled   ? std::make_shared<SerialPort>(ports.left_device,  raise_error) : nullptr),
      right_serial_(right_enabled ? std::make_shared<SerialPort>(ports.right_device, raise_error) : nullptr)
{
    if (left_enabled)  left_hand_.emplace(left_serial_,  left_id);
    if (right_enabled) right_hand_.emplace(right_serial_, right_id);

    // SetVelocity errors propagate via raise_error wired into SerialPort.
    if (left_hand_)  left_hand_->SetVelocity(1000, 1000, 1000, 1000, 1000, 1000);
    if (right_hand_) right_hand_->SetVelocity(1000, 1000, 1000, 1000, 1000, 1000);
}

void UsbInspireRetargeter::send(
    const opt<InspirePose>& left_target,
    const opt<InspirePose>& right_target
) {
    // SetPosition errors propagate via raise_error wired into SerialPort.
    if (left_hand_  && left_target)  left_hand_->SetPosition(*left_target);
    if (right_hand_ && right_target) right_hand_->SetPosition(*right_target);
}

std::pair<InspireFeedback, InspireFeedback> UsbInspireRetargeter::read_feedback() {
    auto poll = [](std::optional<inspire::InspireHand>& hand) {
        InspireFeedback fb;
        if (!hand) return fb;
        InspirePose q, f;
        if (hand->GetPosition(q) == 0) fb.actual = q;
        if (hand->GetForce(f)    == 0) fb.force  = f;
        return fb;
    };
    return {poll(left_hand_), poll(right_hand_)};
}
