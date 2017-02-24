#include "lk_rover.h"

const char* kWheelNames[] = {
  "left_front",
  "left_back",
  "right_front",
  "right_back"
};

LKRover::LKRover(): wheelEfforts{0}, wheelPoss{0}, wheelVels{0} {
  for (int i = 0; i < kNumWheels; ++i) {
    stateHandles[i] = hardware_interface::JointStateHandle(
        kWheelNames[i], &wheelPoss[i], &wheelVels[i], &wheelEfforts[i]);
    commandHandles[i] = hardware_interface::JointHandle(
        stateHandles[i], &wheelEfforts[i]);

    jsi.registerHandle(stateHandles[i]);
    vji.registerHandle(commandHandles[i]);
  }
  registerInterface(&vji);
  registerInterface(&jsi);
}

LKRover::~LKRover() {
}

void LKRover::write() {
  // TODO: write commands to the Arduino
}

void LKRover::read() {
  // TODO: make sure the wheelCounts have the latest data from the Arduino
}
