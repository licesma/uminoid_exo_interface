#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

namespace inspire_port_resolver {

struct Assignment {
    std::string left_device;   // empty if left is disabled
    std::string right_device;  // empty if right is disabled
};

// Scans every /dev/ttyUSB* port for an Inspire hand and returns the
// (left, right) port assignment based on the HAND_ID each hand reports.
//
// Pass std::nullopt for a side to skip it (the returned device will be empty
// and the resolver won't require that hand to be plugged in).
//
// Use set_inspire_id.py (run once per hand) to assign distinct HAND_IDs to
// the left and right hands before relying on this resolver.
//
// On failure, invokes `raise_error` (if provided) with a descriptive message
// and returns an empty Assignment. Never throws.
Assignment resolve(
    std::optional<uint8_t> left_id,
    std::optional<uint8_t> right_id,
    const std::function<void(const std::string&)>& raise_error = nullptr
);

}  // namespace inspire_port_resolver
