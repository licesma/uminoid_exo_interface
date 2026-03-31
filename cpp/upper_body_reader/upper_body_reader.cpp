#include "upper_body_reader.hpp"

#include "joint_reader/as5600_reader.hpp"
#include "joint_reader/dynamixel_reader.hpp"

#include "../utils/circular_math.hpp"
#include "constants.hpp"

#include <iostream>

static G1JointReading invalidReading(G1JointIndex joint) {
  return {joint, -1, false};
}

UpperBodyReader::UpperBodyReader(const std::string& relay_address,
                                 double default_value,
                                 const std::string& bounds_path)
    : bounds_(LoadBounds(bounds_path)) {
  reader_ = std::make_unique<AS5600Reader>(relay_address);
  if (!reader_->IsOk()) return;

  thread_ = std::thread(&UpperBodyReader::ReaderLoop, this);
}

UpperBodyReader::UpperBodyReader(const std::string& device, int baudrate,
                                 const std::string& bounds_path)
    : bounds_(LoadBounds(bounds_path)) {
  reader_ = std::make_unique<DynamixelReader>(device, baudrate);
  if (!reader_->IsOk()) return;

  thread_ = std::thread(&UpperBodyReader::ReaderLoop, this);
}

UpperBodyReader::~UpperBodyReader() {
  reader_->Stop();
  if (thread_.joinable()) thread_.join();
}

void UpperBodyReader::PrintRaw() const {
  const auto s = Snapshot();

  auto p = [&](int i) -> std::string {
    char buf[12];
    std::snprintf(buf, sizeof(buf), "  %5u |", s.data[i]);
    return buf;
  };

  std::string out;
  out += "\033[H\033[J";
  out += "__________________________________________________________________________\n";
  out += " L_sh_pit | L_sh_rol | L_sh_yaw |  L_elbow | L_wr_rol | L_wr_pit | L_wr_yaw\n";
  out += p(0) + p(1) + p(2) + p(3) + p(4) + p(5) + p(6) + "\n";
  out += " R_sh_pit | R_sh_rol | R_sh_yaw |  R_elbow | R_wr_rol | R_wr_pit | R_wr_yaw\n";
  out += p(7) + p(8) + p(9) + p(10) + p(11) + p(12) + p(13) + "\n";
  std::cout << out << std::flush;
}

void UpperBodyReader::ReaderLoop() {
  while (reader_->IsOk()) {
    auto frame = reader_->GetNextLine();
    if (!frame) break;
    std::lock_guard<std::mutex> lock(values_mutex_);
    latest_ = *frame;
  }
}

UpperBodyReadings UpperBodyReader::Eval() const {
  const auto snapshot = Snapshot();

  auto getReadingValue = [&bounds = bounds_, &snapshot](ExoIndex j) {
    int exoIdx = static_cast<int>(j);
    G1JointIndex g1Idx = getG1JointIndex(j);
    auto [lower, upper] = bounds[g1Idx];
    int bit_value = snapshot.data[exoIdx];
    double value = (bit_value - ENCODER_RESOLUTION / 2.0) * ENCODER_PRECISION_RAD;

    if (bit_value == 5000) return invalidReading(g1Idx);

    if (circularDistance(upper, lower) < circularDistance(value, lower))
      return invalidReading(g1Idx);
    return G1JointReading{g1Idx, circularDistance(value, lower), true};
  };

  UpperBodyReadings joint_values;
  for (auto joint : EXO_JOINT_INDICES)
    joint_values[static_cast<int>(joint)] = getReadingValue(joint);
  return joint_values;
}
