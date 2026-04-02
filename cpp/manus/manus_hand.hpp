#include <array>

enum class HandSide { LEFT, RIGHT };


struct ManusFinger {
    float pinky_to_thumb = 0.0f;
    float palm_to_back   = 0.0f;
    float wrist_to_tip   = 0.0f;

    ManusFinger() = default;
    ManusFinger(const float* arr, HandSide side);
};

struct ManusHand {
    ManusFinger thumb;
    ManusFinger index;
    ManusFinger middle;
    ManusFinger ring;
    ManusFinger pinky;

    ManusHand() = default;
    ManusHand(const std::array<std::array<float, 3>, 25>& landmarks, HandSide side);

private:
    static constexpr int THUMB_IDX  = 24;
    static constexpr int INDEX_IDX  = 4;
    static constexpr int MIDDLE_IDX = 9;
    static constexpr int RING_IDX   = 19;
    static constexpr int PINKY_IDX  = 14;
};