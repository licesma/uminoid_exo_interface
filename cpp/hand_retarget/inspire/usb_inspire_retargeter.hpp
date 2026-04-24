#pragma once

#include "hand_retarget/inspire/inspire_port_resolver.hpp"
#include "hand_retarget/inspire/inspire_retargeter.hpp"
#include "inspire.h"
#include "SerialPort.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

/**
 * Inspire retargeter that talks directly to the hand(s) over USB serial.
 * Auto-resolves which /dev/ttyUSB* port hosts which hand by probing HAND_IDs.
 */
class UsbInspireRetargeter : public InspireRetargeter {
public:
    UsbInspireRetargeter(
        bool left_enabled,  uint8_t left_id,
        bool right_enabled, uint8_t right_id,
        const std::string& recording_label = "",
        const std::function<void(const std::string&)>& raise_error = nullptr
    );

protected:
    void send(const opt<InspirePose>& left_target,
              const opt<InspirePose>& right_target) override;

    std::pair<InspireFeedback, InspireFeedback> read_feedback() override;

private:
    // Delegated-to by the public constructor; lets us resolve port paths first.
    UsbInspireRetargeter(
        const inspire_port_resolver::Assignment& ports,
        bool left_enabled,  uint8_t left_id,
        bool right_enabled, uint8_t right_id,
        const std::string& recording_label,
        const std::function<void(const std::string&)>& raise_error
    );

    SerialPort::SharedPtr left_serial_;
    SerialPort::SharedPtr right_serial_;
    std::optional<inspire::InspireHand> left_hand_;
    std::optional<inspire::InspireHand> right_hand_;
};
