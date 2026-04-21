#include "g1TeleopController.hpp"

#include "upper_body_reader/arm_reader/as5600/as5600_arm.hpp"
#include "upper_body_reader/arm_reader/dynamixel/dynamixel_arm.hpp"

#include <stdexcept>

#ifndef READER_BOUNDS_PATH
#define READER_BOUNDS_PATH \
  "../upper_body_reader/arm_reader/as5600/as5600_bounds.yaml"
#endif

#ifndef DYNAMIXEL_BOUNDS_PATH
#define DYNAMIXEL_BOUNDS_PATH \
  "../upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
#endif

G1TeleopController::G1TeleopController(
    std::string networkInterface, const std::string& relay_address,
    bool isSimulation, const std::string& recording_label,
    std::function<int()> collection_id_fn)
    : controller_(networkInterface, isSimulation,
                  LoadMetadata(READER_BOUNDS_PATH),
                  LoadBounds(READER_BOUNDS_PATH), recording_label,
                  std::move(collection_id_fn)),
      left_(std::make_unique<AS5600Arm>(relay_address, 0)),
      right_(std::make_unique<AS5600Arm>(relay_address, ARM_JOINT_COUNT)),
      stop_requested_(false) {
  control_thread_ = std::thread(&G1TeleopController::run, this);
}

G1TeleopController::G1TeleopController(
    std::string networkInterface, const std::string& left_device,
    const std::string& right_device, int baudrate, bool isSimulation,
    const std::string& recording_label,
    std::function<int()> collection_id_fn,
    const std::function<void(const std::string&)>& raise_error)
    : controller_(networkInterface, isSimulation,
                  LoadMetadata(DYNAMIXEL_BOUNDS_PATH),
                  LoadBounds(DYNAMIXEL_BOUNDS_PATH), recording_label,
                  std::move(collection_id_fn)),
      left_(left_device.empty()
                ? nullptr
                : std::make_unique<DynamixelArm>(left_device, baudrate,
                                                 raise_error),
            "", raise_error),
      right_(right_device.empty()
                 ? nullptr
                 : std::make_unique<DynamixelArm>(right_device, baudrate,
                                                  raise_error),
             "", raise_error),
      stop_requested_(false) {
  if (left_device.empty() || right_device.empty()) {
    const std::string msg =
        "[G1TeleopController] Both left and right arm devices are required";
    if (raise_error) raise_error(msg);
    throw std::invalid_argument(msg);
  }

  control_thread_ = std::thread(&G1TeleopController::run, this);
}

G1TeleopController::~G1TeleopController() {
  stop_requested_.store(true);
  left_.stop();
  right_.stop();
  if (control_thread_.joinable()) control_thread_.join();
}

void G1TeleopController::run() {
  if (!controller_.initialize_targets_from_robot_state(
          [this] { return stop_requested_.load(); })) {
    return;
  }

  std::thread left_listener_thread(
      &G1TeleopController::arm_listener_loop, this, std::ref(left_), true);
  std::thread right_listener_thread(
      &G1TeleopController::arm_listener_loop, this, std::ref(right_), false);

  left_listener_thread.join();
  right_listener_thread.join();
}

void G1TeleopController::arm_listener_loop(ArmReader& reader, bool from_left) {
  while (!stop_requested_.load()) {
    auto fresh_line = reader.wait_for_next();
    if (!fresh_line) break;

    controller_.process_arm_sample(*fresh_line, from_left);
  }
}
