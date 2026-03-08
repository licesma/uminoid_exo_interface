#pragma once

#include "../utils/bounds_loader.hpp"
#include "constants.hpp"
#include "serial_manager.hpp"
#include "../g1/model/g1Enums.hpp"
#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <utility>
#include <cmath>

/** Snapshot of all 14 exoskeleton encoder readings, keyed by robot joint index. */
struct JointReading {
  G1JointIndex joint;
  double netAngle;
  bool is_valid;
};

using ExoReadings = std::array<JointReading, 14>;

/*
 * Higher-level wrapper around SerialManager.
 * Eval() returns a stack-allocated snapshot of all 14 arm joints, ready to
 * iterate and write directly into a MotorCommand.
 */



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

class JointReader {
 public:
  explicit JointReader(const std::string& relay_address,
                      double default_value = 0.0,
                      const std::string& bounds_path =
                          "../joint_reader/upperBodyReaderBounds.yaml")
      : serial_(relay_address), bounds_(LoadBounds(bounds_path)) {}


  static G1JointIndex getG1JointIndex(ExoIndex j){
    return static_cast<G1JointIndex>(static_cast<int>(j) + 15);
  }

  void PrintRaw() const { serial_.PrintRaw(); }

  static void print(SerialLine line){
    std::cout<<"[";
    for(int j = 0; j < 14; j++){
      std::cout<<line.data[j]<<", ";
    }
    std::cout<<"]"<<std::endl;
  }

  static double circularDistance(double high, double low){
    return low < high ? high - low : 2*M_PI + high - low;
  }

  static JointReading invalidReading(G1JointIndex joint){
    return {joint, -1, false};
  }

  ExoReadings Eval() const {
    const auto snapshot = serial_.Snapshot();
    
    auto getReadingValue = [&bounds = bounds_, &snapshot](ExoIndex j) {
      int exoIdx = static_cast<int>(j);
      G1JointIndex g1Idx = getG1JointIndex(j);
      auto [lower, upper] = bounds[g1Idx];
      int bit_value = snapshot.data[exoIdx];



      if(bit_value == 5000)      return invalidReading(g1Idx);

      double value = (bit_value - ENCODER_RESOLUTION/2.0)*ENCODER_PRECISION_RAD;
      //print(snapshot);
      if(j == ExoIndex::LeftWristYaw && circularDistance(upper, lower) < circularDistance(value, lower)){
        std::cout<<"Bounds: ["<<lower<<", "<<upper<<"]"<<std::endl;
         
        std::cout<<"Snapshot:"<<snapshot.data[exoIdx]<<std::endl;
        std::cout<<"Value:"<<value<<std::endl;
        std::cout<<"Val to Low"<<circularDistance(value, lower)<<std::endl;
        std::cout<<"High to Low:"<<circularDistance(upper, lower)<<std::endl;
        std::cout<<"is Outside:"<<std::to_string(circularDistance(upper, lower) < circularDistance(value, lower))<<std::endl; 
        std::cout<<"__________________________________________________"<<std::endl;
        std::cout<<"Low to Val"<<circularDistance(lower ,value)<<std::endl;
        std::cout<<"Val to High"<<circularDistance(value, upper);
      }

      if(circularDistance(upper, lower) < circularDistance(value, lower))  return invalidReading(g1Idx);
      
      return JointReading{g1Idx, circularDistance(value, lower), true};
      
    };

    ExoReadings joint_values;

    for(auto joint: EXO_JOINT_INDICES){
      joint_values[static_cast<int>(joint)] = getReadingValue(joint);
    }

    return joint_values;
  }

 private:
  SerialManager serial_;
  JointBounds bounds_;
};
