#include "g1_upper_body_reader.hpp"

#include "arm_reader/as5600/as5600_arm.hpp"
#include "arm_reader/dynamixel/dynamixel_arm.hpp"

#include <stdexcept>
#include <thread>

#ifndef READER_BOUNDS_PATH
#define READER_BOUNDS_PATH \
  "../upper_body_reader/arm_reader/as5600/as5600_bounds.yaml"
#endif

#ifndef DYNAMIXEL_BOUNDS_PATH
#define DYNAMIXEL_BOUNDS_PATH \
  "../upper_body_reader/arm_reader/dynamixel/dynamixel_bounds.yaml"
#endif

G1UpperBodyReader::G1UpperBodyReader(
    std::string networkInterface, const std::string& relay_address,
    bool isSimulation, const std::string& recording_label,
    const std::function<void(const std::string&)>& raise_error)
    : controller_(networkInterface, isSimulation,
                  LoadMetadata(READER_BOUNDS_PATH),
                  LoadBounds(READER_BOUNDS_PATH), recording_label,
                  raise_error),
      left_(std::make_unique<AS5600Arm>(relay_address, 0)),
      right_(std::make_unique<AS5600Arm>(relay_address, ARM_JOINT_COUNT)) {}

G1UpperBodyReader::G1UpperBodyReader(
    std::string networkInterface, const std::string& left_device,
    const std::string& right_device, int baudrate, bool isSimulation,
    const std::string& recording_label,
    const std::function<void(const std::string&)>& raise_error)
    : controller_(networkInterface, isSimulation,
                  LoadMetadata(DYNAMIXEL_BOUNDS_PATH),
                  LoadBounds(DYNAMIXEL_BOUNDS_PATH), recording_label,
                  raise_error),
      left_(left_device.empty()
                ? nullptr
                : std::make_unique<DynamixelArm>(left_device, baudrate,
                                                 raise_error),
            "", raise_error),
      right_(right_device.empty()
                 ? nullptr
                 : std::make_unique<DynamixelArm>(right_device, baudrate,
                                                  raise_error),
             "", raise_error) {
  if (left_device.empty() || right_device.empty()) {
    const std::string msg =
        "[G1UpperBodyReader] Both left and right arm devices are required";
    if (raise_error) raise_error(msg);
    throw std::invalid_argument(msg);
  }
}

void G1UpperBodyReader::collect_loop(
    const std::function<int()>&  collection_id,
    const std::function<bool()>& stop,
    const std::function<bool()>& pause) {
  if (!controller_.initialize_targets_from_robot_state(stop)) return;

  std::thread left_thread(&G1UpperBodyReader::arm_listener_loop, this,
                          std::ref(left_), true, std::cref(collection_id),
                          std::cref(stop), std::cref(pause));
  std::thread right_thread(&G1UpperBodyReader::arm_listener_loop, this,
                           std::ref(right_), false, std::cref(collection_id),
                           std::cref(stop), std::cref(pause));

  left_thread.join();
  right_thread.join();
}

void G1UpperBodyReader::handle_key(char key) {
  controller_.handle_key(key);
}

void G1UpperBodyReader::arm_listener_loop(
    ArmReader& reader, bool from_left,
    const std::function<int()>&  collection_id,
    const std::function<bool()>& stop,
    const std::function<bool()>& pause) {
  while (!stop()) {
    auto fresh_line = reader.wait_for_next();
    if (!fresh_line) break;

    controller_.process_arm_sample(*fresh_line, from_left, collection_id(),
                                   !pause());
  }
}
