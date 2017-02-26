#ifndef LK_GAZEBO_HW_H
#define LK_GAZEBO_HW_H

#include "ros/ros.h"
#include "ros/console.h"

#include <array>

#include "lk_rover/lk_hw.h"

class GazeboHW: public LKHW {
public:
  GazeboHW();
  bool init(ros::NodeHandle &nh);
  void setPWMs(const std::array<double, kNumWheels>&);
  void getCount(std::array<double, kNumWheels>&);
private:
  ros::ServiceClient mj, cj, gj;

  std::array<double, kNumWheels> curEfforts;
  std::array<int, kNumWheels> numEfforts;
};

#endif
