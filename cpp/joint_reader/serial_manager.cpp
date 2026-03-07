#include "serial_manager.hpp"

#include <cstdio>
#include <iostream>
#include <mutex>
#include <string>

SerialManager::SerialManager(const std::string& relay_address)
    : reader_(relay_address) {
  if (!reader_.IsOk()) return;
  thread_ = std::thread(&SerialManager::ReaderLoop, this);
}

SerialManager::~SerialManager() {
  reader_.Stop();
  if (thread_.joinable()) thread_.join();
}

void SerialManager::PrintRaw() const {
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

void SerialManager::ReaderLoop() {
  while (reader_.IsOk()) {
    auto frame = reader_.GetNextLine();
    if (!frame) break;
    std::lock_guard<std::mutex> lock(values_mutex_);
    latest_ = *frame;
  }
}
