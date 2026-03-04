#pragma once
#include "../g1/model/g1Enums.hpp"
#include "../g1/model/g1Structs.hpp"
#include <array>
#include <string>
#include <utility>

struct JointBounds {
  std::array<std::pair<double, double>, G1_NUM_MOTOR> data{};

  std::pair<double, double>& operator[](G1JointIndex idx) { return data[idx]; }
  const std::pair<double, double>& operator[](G1JointIndex idx) const { return data[idx]; }

};

/** Load joint bounds from a YAML file (e.g. upperBodyJointBounds.yaml). */
JointBounds LoadBounds(const std::string& path);
