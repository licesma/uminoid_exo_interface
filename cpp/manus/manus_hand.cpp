#include "manus_hand.hpp"

ManusFinger::ManusFinger(const float* arr, HandSide side) {
    pinky_to_thumb = arr[0] * (side == HandSide::RIGHT ? -1.0f : 1.0f);
    palm_to_back   = arr[1];
    wrist_to_tip   = arr[2];
}

ManusHand::ManusHand(const std::array<std::array<float, 3>, 25>& landmarks, HandSide side)
    : thumb (landmarks[THUMB_IDX].data(),  side),
      index (landmarks[INDEX_IDX].data(),  side),
      middle(landmarks[MIDDLE_IDX].data(), side),
      ring  (landmarks[RING_IDX].data(),   side),
      pinky (landmarks[PINKY_IDX].data(),  side)
{}

