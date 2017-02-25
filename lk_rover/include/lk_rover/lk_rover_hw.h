#ifndef LK_ROVER_HW_H
#define LK_ROVER_HW_H

#include "lk_rover/lk_hw.h"

class LKRoverHW: public LKHW {
public:
  void setPWMs(const std::array<double, kNumWheels>&);
  void getCount(std::array<double, kNumWheels>&);
};

#endif
