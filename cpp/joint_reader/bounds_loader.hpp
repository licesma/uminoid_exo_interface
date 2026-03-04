#pragma once
#include "../g1/model/g1Enums.hpp"
#include <array>
#include <string>
#include <utility>

/** Bounds for all 14 joints, same format as ExoReadings: array of (joint, (lower, upper)). */
using ExoJointBounds = std::array<std::pair<G1JointIndex, std::pair<double, double>>, 14>;

/** Load joint bounds from upperBodyReaderBounds.yaml. */
ExoJointBounds LoadBounds(const std::string& path);
