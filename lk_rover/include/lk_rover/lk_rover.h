#ifndef LK_ROVER_H
#define LK_ROVER_H

#include <array>
#include <memory>

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

#include "lk_rover/common.h"
#include "lk_rover/lk_hw.h"

class LKRover: public hardware_interface::RobotHW {
public:
  LKRover(std::shared_ptr<LKHW> hw_);
  ~LKRover();

  void write();
  void read();
private:
  std::array<double, kNumWheels> wheelPoss;
  std::array<double, kNumWheels> wheelVels;
  std::array<double, kNumWheels> wheelAccels;

  std::array<double, kNumWheels> wheelPwms;

  hardware_interface::VelocityJointInterface vji;
  hardware_interface::JointStateInterface jsi;

  std::shared_ptr<LKHW> hw;
  ros::Time lastTime;
};

#endif
