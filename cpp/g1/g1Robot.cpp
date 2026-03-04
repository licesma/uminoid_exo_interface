#include "g1Robot.hpp"

#include <cstring>
#include <iostream>
#include <unistd.h>

#include "../utils/crc32.hpp"

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

G1Robot::G1Robot(std::string networkInterface, bool isSimulation)
    : mode_pr_(Mode::PR),
      mode_machine_(0),
      counter_(0) {
  ChannelFactory::Instance()->Init(isSimulation ? 1 : 0, networkInterface);

  msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
  msc_->SetTimeout(5.0f);
  msc_->Init();
  std::string form, name;
  while (msc_->CheckMode(form, name), !name.empty()) {
    if (msc_->ReleaseMode())
      std::cout << "Failed to switch to Release Mode\n";
    sleep(5);
  }

  lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
  lowcmd_publisher_->InitChannel();
  lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
  lowstate_subscriber_->InitChannel(std::bind(&G1Robot::LowStateHandler, this, std::placeholders::_1), 1);
  imutorso_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
  imutorso_subscriber_->InitChannel(std::bind(&G1Robot::imuTorsoHandler, this, std::placeholders::_1), 1);
  command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &G1Robot::LowCommandWriter, this);
}

void G1Robot::StartControlThread() {
  control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, &G1Robot::ControlDispatch, this);
}

void G1Robot::ControlDispatch() {
  Control();
}

void G1Robot::imuTorsoHandler(const void *message) {
  IMUState_ imu_torso = *(const IMUState_ *)message;
  (void)imu_torso;
}

void G1Robot::LowStateHandler(const void *message) {
  LowState_ low_state = *(const LowState_ *)message;
  if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
    std::cout << "[ERROR] CRC Error" << std::endl;
    return;
  }

  MotorState ms_tmp;
  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    ms_tmp.q.at(i) = low_state.motor_state()[i].q();
    ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
    if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll)
      std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
  }
  motor_state_buffer_.SetData(ms_tmp);

  ImuState imu_tmp;
  imu_tmp.omega = low_state.imu_state().gyroscope();
  imu_tmp.rpy = low_state.imu_state().rpy();
  imu_state_buffer_.SetData(imu_tmp);

  memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
  gamepad_.update(rx_.RF_RX);

  if (mode_machine_ != low_state.mode_machine()) {
    if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
    mode_machine_ = low_state.mode_machine();
  }

  if (++counter_ % 500 == 0) {
    counter_ = 0;
  }
}

void G1Robot::LowCommandWriter() {
  LowCmd_ dds_low_command;
  dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
  dds_low_command.mode_machine() = mode_machine_;

  const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
  if (mc) {
    for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
      dds_low_command.motor_cmd().at(i).mode() = 1;
      dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
      dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
      dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
      dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);
      dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);
    }

    dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
    lowcmd_publisher_->Write(dds_low_command);
  }
}
