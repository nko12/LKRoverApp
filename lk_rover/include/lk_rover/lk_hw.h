#ifndef LK_HW_H
#define LK_HW_H

#include <array>

#include "lk_rover/lk_rover.h"

class LKHW {
public:
  virtual void setPWMs(const std::array<double, kNumWheels>&);
  virtual void getCount(std::array<double, kNumWheels>&);
};

#endif
