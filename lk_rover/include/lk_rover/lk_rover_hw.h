#ifndef LK_ROVER_HW_H
#define LK_ROVER_HW_H

#include "ros.h"

#include "lk_rover/lk_hw.h"

class LKRoverHW: public LKHW {
public:
  LKRoverHW(ros::NodeHandle nh);

  void setPWMs(const std::array<double, kNumWheels>&);
  void getCount(std::array<double, kNumWheels>&);
  void waitForSerial();
};

#endif
