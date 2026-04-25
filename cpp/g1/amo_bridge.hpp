#pragma once

// AmoBridge: ZMQ link to the Python AMO sidecar.
//
//   state  : C++ PUB (bind)    -> Python SUB (connect)   tcp://127.0.0.1:5555
//   action : C++ SUB (connect) <- Python PUB (bind)      tcp://127.0.0.1:5556
//
// Wire format must match amo_sidecar/proto.py byte-for-byte.

#include "../utils/DataBuffer.hpp"
#include "model/g1Enums.hpp"
#include "model/g1Structs.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>

#include <zmq.hpp>

// AMO 23-DoF *observation* order (from play_amo.py dof_names) -> G1JointIndex.
// Used when packing the state frame the sidecar consumes. AMO has no wrists,
// so the 8 arm slots (15..22) skip directly from elbow to shoulder of the
// other side.
inline constexpr std::array<G1JointIndex, 23> AMO_OBSERVATION = {
    LeftHipPitch,    LeftHipRoll,    LeftHipYaw,    LeftKnee,
    LeftAnklePitch,  LeftAnkleRoll,
    RightHipPitch,   RightHipRoll,   RightHipYaw,   RightKnee,
    RightAnklePitch, RightAnkleRoll,
    WaistYaw,        WaistRoll,      WaistPitch,
    LeftShoulderPitch,  LeftShoulderRoll,  LeftShoulderYaw,  LeftElbow,
    RightShoulderPitch, RightShoulderRoll, RightShoulderYaw, RightElbow,
};

// AMO 15-DoF *action* order -> G1JointIndex. These are the only joints AMO
// commands; arms are owned by the exo-driven path in g1Controller.
inline constexpr std::array<G1JointIndex, 15> AMO_ACTION = {
    LeftHipPitch,    LeftHipRoll,    LeftHipYaw,    LeftKnee,
    LeftAnklePitch,  LeftAnkleRoll,
    RightHipPitch,   RightHipRoll,   RightHipYaw,   RightKnee,
    RightAnklePitch, RightAnkleRoll,
    WaistYaw,        WaistRoll,      WaistPitch,
};

inline constexpr int AMO_LOWER_DOF = static_cast<int>(AMO_ACTION.size());

// High-level commands the sidecar consumes (mirrors AmoCommands in
// humanoid_env_headless.py: vx, yaw_target, vy, height, torso_yaw,
// torso_pitch, torso_roll).
struct AmoCommand {
  double vx = 0.0;
  double yaw_target = 0.0;
  double vy = 0.0;
  double height = 0.0;
  double torso_yaw = 0.0;
  double torso_pitch = 0.0;
  double torso_roll = 0.0;
};

// What the sidecar returns: 15 absolute PD targets for legs + waist
// (already includes default_dof_pos[:15] offset).
struct AmoAction {
  uint64_t seq = 0;       // echoes the state.seq used to compute this action
  uint64_t ts_ns = 0;     // sidecar publish time, monotonic ns
  uint64_t receive_ts_ns = 0;// when we received it (set by AmoBridge)
  std::array<double, AMO_LOWER_DOF> q_target{};
};

class AmoBridge {
 public:
  struct Config {
    std::string state_endpoint  = "tcp://127.0.0.1:5555"; // PUB binds here
    std::string action_endpoint = "tcp://127.0.0.1:5556"; // SUB connects here
    // An action older than this is treated as stale and ignored.
    int stale_action_ms = 60;
  };

  AmoBridge();
  explicit AmoBridge(Config cfg);
  ~AmoBridge();
  AmoBridge(const AmoBridge&) = delete;
  AmoBridge& operator=(const AmoBridge&) = delete;

  // Pack and PUB one state frame. Cheap; safe to call from a high-rate thread.
  // Quaternion order is (w, x, y, z). seq must increase monotonically.
  void publish_state(uint64_t seq,
                     const MotorState& motor_state,
                     const ImuState& imu_state,
                     const std::array<float, 4>& quat_wxyz,
                     const AmoCommand& cmd);

  // Returns the most recent action if it is fresher than stale_action_ms.
  // nullopt if none yet, or stale.
  std::optional<AmoAction> latest_action() const;

  // TODO(amo-bridge): remove these debug counters once the sidecar is
  // validated end-to-end and we trust the wire format / connection. They
  // exist purely to diagnose "is the sidecar talking to me / are we in sync".
  uint64_t debug_successful_action()  const { return debug_successful_action_.load(); }
  uint64_t debug_bad_frame_counts()   const { return debug_bad_frame_counts_.load(); }

 private:
  void receive_loop_();

  Config cfg_;
  zmq::context_t ctx_;
  zmq::socket_t  state_pub_;
  zmq::socket_t  action_sub_;

  DataBuffer<AmoAction> action_buffer_;
  // TODO(amo-bridge): remove once sidecar integration is trusted.
  std::atomic<uint64_t> debug_successful_action_{0};
  std::atomic<uint64_t> debug_bad_frame_counts_{0};

  std::atomic<bool> running_{false};
  std::thread receive_thread_;
};
