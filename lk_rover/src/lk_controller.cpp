#include "lk_rover/lk_controller.h"

LKController::LKController(ros::NodeHandle &nh_, ros::NodeHandle &nhPrivate_): nh(nh_), nhPrivate(nhPrivate_) {
  // TODO: subscribe to whatever that's needed, set up publishers for the actuators
}

LKController::~LKController() {
}

void LKController::doStuff() {
  // TODO: logic goes here
}
