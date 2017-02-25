#include "lk_rover/lk_rover.h"

const char* kWheelNames[] = {
  "base_to_left_front_wheel",
  "base_to_left_back_wheel",
  "base_to_right_front_wheel",
  "base_to_right_back_wheel"
};

LKRover::LKRover(std::shared_ptr<LKHW> hw_):
    wheelAccels{}, wheelPoss{}, wheelVels{}, wheelPwms{},
    hw(hw_), lastTime(ros::Time::now()) {
  for (int i = 0; i < kNumWheels; ++i) {
    auto jsh = hardware_interface::JointStateHandle(
        kWheelNames[i], &wheelPoss[i], &wheelVels[i], &wheelAccels[i]);
    auto jh = hardware_interface::JointHandle(jsh, &wheelPwms[i]);

    jsi.registerHandle(jsh);
    vji.registerHandle(jh);
  }
  registerInterface(&vji);
  registerInterface(&jsi);
}

LKRover::~LKRover() {
}

void LKRover::write() {
  hw->setPWMs(wheelPwms);
}

void LKRover::read() {
  std::array<double, kNumWheels> oldPos = wheelPoss;
  std::array<double, kNumWheels> oldVels = wheelVels;

  hw->getCount(wheelPoss);
  ros::Time curTime = ros::Time::now();
  for (int i = 0; i < kNumWheels; i++) {
    wheelVels[i] = (wheelPoss[i] - oldPos[i])/(curTime - lastTime).toSec();
  }
  for (int i = 0; i < kNumWheels; i++) {
    wheelAccels[i] = (wheelVels[i] - oldVels[i])/(curTime - lastTime).toSec();
  }
  lastTime = curTime;
}
