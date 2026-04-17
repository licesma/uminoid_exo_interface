#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace inspire_port_resolver {

struct Assignment {
    std::string left_device;
    std::string right_device;
};

// Scans every /dev/ttyUSB* port for an Inspire hand and returns the
// (left, right) port assignment based on the HAND_ID each hand reports.
//
// Use set_inspire_id.py (run once per hand) to assign distinct HAND_IDs to
// the left and right hands before relying on this resolver.
//
// On failure, invokes `raise_error` (if provided) with a descriptive message
// and returns an empty Assignment. Never throws.
Assignment resolve(
    uint8_t left_id,
    uint8_t right_id,
    const std::function<void(const std::string&)>& raise_error = nullptr
);

}  // namespace inspire_port_resolver
