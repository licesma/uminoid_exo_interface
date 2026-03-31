#pragma once

/*
################################################################################
#  ⚠️  KEEP IN SYNC  ⚠️
#  This file MUST be kept in sync with: ../../joint_reader_constants.py
################################################################################
*/

#include <cmath>
#include <array>

constexpr int JOINT_COUNT = 14;
constexpr int ENCODER_RESOLUTION = 4096;
constexpr double ENCODER_PRECISION = 1.0 / ENCODER_RESOLUTION;
constexpr double ENCODER_PRECISION_RAD = ENCODER_PRECISION * 2.0 * M_PI;



/**
 * Encoder index in the CSV stream sent over serial.
 * Order matches JOINT_NAMES in joints.py:
 *   left arm (shoulder_pitch → wrist_yaw), then right arm.
 */
 enum class ExoIndex : int {
    LeftShoulderPitch  = 0,
    LeftShoulderRoll   = 1,
    LeftShoulderYaw    = 2,
    LeftElbow          = 3,
    LeftWristRoll      = 4,
    LeftWristPitch     = 5,
    LeftWristYaw       = 6,
    RightShoulderPitch = 7,
    RightShoulderRoll  = 8,
    RightShoulderYaw   = 9,
    RightElbow         = 10,
    RightWristRoll     = 11,
    RightWristPitch    = 12,
    RightWristYaw      = 13,
  };

  const std::array<ExoIndex, JOINT_COUNT> EXO_JOINT_INDICES = {
    ExoIndex::LeftShoulderPitch,
    ExoIndex::LeftShoulderRoll, 
    ExoIndex::LeftShoulderYaw, 
    ExoIndex::LeftElbow, 
    ExoIndex::LeftWristRoll, 
    ExoIndex::LeftWristPitch, 
    ExoIndex::LeftWristYaw, 
    ExoIndex::RightShoulderPitch, 
    ExoIndex::RightShoulderRoll, 
    ExoIndex::RightShoulderYaw, 
    ExoIndex::RightElbow, 
    ExoIndex::RightWristRoll, 
    ExoIndex::RightWristPitch, 
    ExoIndex::RightWristYaw,
  };
  