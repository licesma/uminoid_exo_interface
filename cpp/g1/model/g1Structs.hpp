#pragma once

#include <array>
#include <cstdint>

constexpr int G1_NUM_MOTOR = 29;

// Per-joint values (gains, initial pose) live in g1Values.hpp.

struct ImuState {
  std::array<float, 4> quat = {};              // (w, x, y, z), straight from the IDL
  std::array<float, 3> angular_velocity = {};
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
