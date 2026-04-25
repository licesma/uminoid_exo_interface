#include "amo_bridge.hpp"

#include "amo_zmq_helper.hpp"

#include <chrono>
#include <cstring>
#include <iostream>

namespace {

// Wire format (must match amo_sidecar/proto.py):
//   state  : uint64 seq, uint64 ts_ns,
//            float64 q[23], dq[23], quat[4], ang_vel[3], cmds[7]
//   action : uint64 seq, uint64 ts_ns, float64 q_target[15]
constexpr size_t STATE_FRAME_SIZE  = 8 + 8 + 8 * (23 + 23 + 4 + 3 + 7);  // 496
constexpr size_t ACTION_FRAME_SIZE = 8 + 8 + 8 * 15;                     // 136
static_assert(STATE_FRAME_SIZE == 496);
static_assert(ACTION_FRAME_SIZE == 136);

inline uint64_t now_ns() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

template <typename T>
inline void put(uint8_t* buf, size_t& off, T value) {
  std::memcpy(buf + off, &value, sizeof(T));
  off += sizeof(T);
}

template <typename T>
inline T get(const uint8_t* buf, size_t& off) {
  T value;
  std::memcpy(&value, buf + off, sizeof(T));
  off += sizeof(T);
  return value;
}

// Layout-aware packer for the C++ -> Python state frame.
size_t pack_state_frame(uint8_t* buf,
                        uint64_t seq,
                        const MotorState& motor_state,
                        const ImuState& imu_state,
                        const std::array<float, 4>& quat_wxyz,
                        const AmoCommand& cmd) {
  size_t off = 0;
  put<uint64_t>(buf, off, seq);
  put<uint64_t>(buf, off, now_ns());

  for (int i = 0; i < 23; ++i)
    put<double>(buf, off, static_cast<double>(motor_state.q.at(AMO_OBSERVATION[i])));
  for (int i = 0; i < 23; ++i)
    put<double>(buf, off, static_cast<double>(motor_state.dq.at(AMO_OBSERVATION[i])));
  for (int i = 0; i < 4; ++i)
    put<double>(buf, off, static_cast<double>(quat_wxyz[i]));
  for (int i = 0; i < 3; ++i)
    put<double>(buf, off, static_cast<double>(imu_state.omega[i]));

  put<double>(buf, off, cmd.vx);
  put<double>(buf, off, cmd.yaw_target);
  put<double>(buf, off, cmd.vy);
  put<double>(buf, off, cmd.height);
  put<double>(buf, off, cmd.torso_yaw);
  put<double>(buf, off, cmd.torso_pitch);
  put<double>(buf, off, cmd.torso_roll);

  return off;
}

// Layout-aware unpacker for the Python -> C++ action frame.
// Caller must have already verified that the frame is ACTION_FRAME_SIZE bytes.
AmoAction unpack_action_frame(const uint8_t* buf) {
  size_t off = 0;
  AmoAction act;
  act.seq   = get<uint64_t>(buf, off);
  act.ts_ns = get<uint64_t>(buf, off);
  for (int i = 0; i < AMO_LOWER_DOF; ++i) {
    act.q_target[i] = get<double>(buf, off);
  }
  return act;
}

bool is_stale(const AmoAction& action, int stale_threshold_ms) {
  const uint64_t age_ns = now_ns() - action.receive_ts_ns;
  const uint64_t stale_ns =
      static_cast<uint64_t>(stale_threshold_ms) * 1'000'000ULL;
  return age_ns > stale_ns;
}

}  // namespace

AmoBridge::AmoBridge() : AmoBridge(Config{}) {}

AmoBridge::AmoBridge(Config cfg)
    : cfg_(std::move(cfg)),
      ctx_(1),
      state_pub_(amo_zmq::make_pub_bound(ctx_, cfg_.state_endpoint)),
      action_sub_(amo_zmq::make_sub_conflate(ctx_, cfg_.action_endpoint)) {
  running_.store(true);
  receive_thread_ = std::thread(&AmoBridge::receive_loop_, this);
}

AmoBridge::~AmoBridge() {
  running_.store(false);
  // Closing sockets unblocks poll() with ETERM in the receive thread.
  try { action_sub_.close(); } catch (...) {}
  try { state_pub_.close();  } catch (...) {}
  if (receive_thread_.joinable()) receive_thread_.join();
}

void AmoBridge::publish_state(uint64_t seq,
                              const MotorState& motor_state,
                              const ImuState& imu_state,
                              const std::array<float, 4>& quat_wxyz,
                              const AmoCommand& cmd) {
  uint8_t buf[STATE_FRAME_SIZE];
  const size_t written =
      pack_state_frame(buf, seq, motor_state, imu_state, quat_wxyz, cmd);
  if (written != STATE_FRAME_SIZE) {
    std::cerr << "[amo_bridge] BUG: packed " << written
              << " bytes, expected " << STATE_FRAME_SIZE << "\n";
    return;
  }
  amo_zmq::send_dontwait(state_pub_, buf, STATE_FRAME_SIZE);
}

std::optional<AmoAction> AmoBridge::latest_action() const {
  auto act = action_buffer_.GetData();
  if (!act) return std::nullopt;
  if (is_stale(*act, cfg_.stale_action_ms)) return std::nullopt;
  return *act;
}

void AmoBridge::receive_loop_() {
  amo_zmq::run_receive_loop(action_sub_, running_,
      [this](const uint8_t* buf, size_t size) {
        if (size != ACTION_FRAME_SIZE) {
          debug_bad_frame_counts_.fetch_add(1);  // TODO(amo-bridge): remove
          return;
        }
        AmoAction act = unpack_action_frame(buf);
        act.receive_ts_ns = now_ns();
        action_buffer_.SetData(act);
        debug_successful_action_.fetch_add(1);  // TODO(amo-bridge): remove
      });
}
