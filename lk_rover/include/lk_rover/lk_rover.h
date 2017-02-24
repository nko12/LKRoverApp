#ifndef LK_ROVER_H
#define LK_ROVER_H

#include <array>

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

constexpr int kNumWheels = 4;

class LKRover: public hardware_interface::RobotHW {
public:
  LKRover();
  ~LKRover();

  void write();
  void read();
private:
  std::array<double, kNumWheels> wheelEfforts;
  std::array<double, kNumWheels> wheelPoss;
  std::array<double, kNumWheels> wheelVels;

  hardware_interface::VelocityJointInterface vji;
  hardware_interface::JointStateInterface jsi;
};

#endif
