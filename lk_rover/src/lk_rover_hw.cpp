#include "lk_rover/AllPWMs.h"
#include "lk_rover/AllEncoders.h"
#include "lk_rover/lk_rover_hw.h"

void LKRoverHW::setPWMs(const std::array<double, kNumWheels>& pwms,
    double dumpA, double dumpB, double ladderA, double ladderB, double spin) {
  // TODO: transmit PWM values to the wheels
}

void LKRoverHW::getCount(std::array<double, kNumWheels>& encoderValsInRadians,
    double &dumpA, double& dumpB, double &ladderA, double &ladderB) {
  // TODO: get data from the wheel encoders and store in the array and other things
}

void LKRoverHW::waitForSerial() {
}
