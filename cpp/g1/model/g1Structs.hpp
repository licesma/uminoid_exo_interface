#pragma once

#include <array>
#include <cstdint>

constexpr int G1_NUM_MOTOR = 29;

// Stiffness (Kp) and damping (Kd) gains for all 29 G1 joints
inline constexpr std::array<float, G1_NUM_MOTOR> Kp{
    60, 60, 60, 100, 40, 40,      // left leg
    60, 60, 60, 100, 40, 40,      // right leg
    60, 40, 40,                   // waist
    40, 40, 40, 40,  40, 40, 40,  // left arm
    40, 40, 40, 40,  40, 40, 40   // right arm
};

inline constexpr std::array<float, G1_NUM_MOTOR> Kd{
    1, 1, 1, 2, 1, 1,     // left leg
    1, 1, 1, 2, 1, 1,     // right leg
    1, 1, 1,              // waist
    1, 1, 1, 1, 1, 1, 1,  // left arm
    1, 1, 1, 1, 1, 1, 1   // right arm
};

struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
};

struct MotorCommand {
  std::array<float, G1_NUM_MOTOR> q_target = {};
  std::array<float, G1_NUM_MOTOR> dq_target = {};
  std::array<float, G1_NUM_MOTOR> kp = {};
  std::array<float, G1_NUM_MOTOR> kd = {};
  std::array<float, G1_NUM_MOTOR> tau_ff = {};
};

struct MotorState {
  uint64_t host_timestamp = 0;
  std::array<float, G1_NUM_MOTOR> q = {};
  std::array<float, G1_NUM_MOTOR> dq = {};
};
