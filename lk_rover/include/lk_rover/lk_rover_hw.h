#ifndef LK_ROVER_HW_H
#define LK_ROVER_HW_H

#include <thread>

#include "ros.h"

#include "lk_rover/AllPWMs.h"
#include "lk_rover/AllEncoders.h"

#include "lk_rover/lk_hw.h"

class LKRoverHW: public LKHW {
public:
  LKRoverHW(ros::NodeHandle nh);

  void setPWMs(const std::array<double, kNumWheels>&);
  void getCount(std::array<double, kNumWheels>&);
  void waitForSerial();
private:
  void encoderCb(const lk_rover::AllEncoders&);
  ros::NodeHandle nh;
  ros::Publisher pubPwm;
  ros::Subscriber subEncoders;

  std::mutex encoderLock;
  lk_rover::AllEncoders lastEncoderData;
};

#endif
