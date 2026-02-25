#pragma once

#include "g1Structs.hpp"  // same directory

// Stiffness for all G1 joints
inline const std::array<float, G1_NUM_MOTOR> Kp{
    60, 60, 60, 100, 40, 40,      // legs
    60, 60, 60, 100, 40, 40,      // legs
    60, 40, 40,                   // waist
    40, 40, 40, 40,  40, 40, 40,  // arms
    40, 40, 40, 40,  40, 40, 40   // arms
};

// Damping for all G1 joints
inline const std::array<float, G1_NUM_MOTOR> Kd{
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1,              // waist
    1, 1, 1, 1, 1, 1, 1,  // arms
    1, 1, 1, 1, 1, 1, 1   // arms
};
