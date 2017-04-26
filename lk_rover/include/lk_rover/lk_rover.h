#ifndef LK_ROVER_H
#define LK_ROVER_H

#include <array>
#include <memory>

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

#include "lk_rover/common.h"
#include "lk_rover/lk_hw.h"

struct ActuatorConfig {
  double min, max, gain;
};

struct ActuatorConfigs {
  ActuatorConfig left, right;
};

/// A helper class for converting a single PWM/encoder pair into two
/// PWMs/encoders when the two actuators need to be kept in precise synchronization
/// This basically is used to simulate a "virtual" actuator for ROS to actuate,
/// which will then be used to control two physical actuators
/// ActuatorConfig is used to allow for some amount of configuration
class TwinJoints {
public:
  const ActuatorConfig a, b;

  TwinJoints(const ActuatorConfig &a_, const ActuatorConfig &b_);
  ~TwinJoints() {}

  double getProcessedEncoder(double inA, double inB);
  void getProcessedPwms(double pwm, double &outA, double &outB);
private:
  double lastA, lastB;
};

class LKRover: public hardware_interface::RobotHW {
public:
  LKRover(std::shared_ptr<LKHW> hw_);
  ~LKRover();

  void write();
  void read();
private:
  TwinJoints virtualDump, virtualLadder;

  std::array<double, kNumWheels> wheelPoss;
  std::array<double, kNumWheels> wheelVels;
  std::array<double, kNumWheels> wheelAccels;
  double dumpPos, dumpVel, dumpAccel;
  double ladderPos, ladderVel, ladderAccel;

  std::array<double, kNumWheels> wheelPwms;
  double dumpPwm, ladderPwm;

  double spin;

  hardware_interface::VelocityJointInterface vji;
  hardware_interface::PositionJointInterface pji;
  hardware_interface::JointStateInterface jsi;

  std::shared_ptr<LKHW> hw;
  ros::Time lastTime;
};

#endif
