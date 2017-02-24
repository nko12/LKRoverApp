#ifndef LK_ROVER_H
#define LK_ROVER_H

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

const int kNumWheels = 4;

class LKRover: public hardware_interface::RobotHW {
public:
  LKRover();
  ~LKRover();

  void write();
  void read();
private:
  double wheelEfforts[kNumWheels];
  double wheelPoss[kNumWheels];
  double wheelVels[kNumWheels];

  hardware_interface::JointStateHandle stateHandles[kNumWheels];
  hardware_interface::JointHandle commandHandles[kNumWheels];

  hardware_interface::VelocityJointInterface vji;
  hardware_interface::JointStateInterface jsi;
};

#endif
