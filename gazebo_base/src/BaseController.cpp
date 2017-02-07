#include "ros/ros.h"
#include "ros/console.h"

#include "BaseController.h"
#include "PIDController.h"

BaseController::BaseController(ros::ServiceClient &mj, ros::ServiceClient &cj): moveJoints(mj), clearJoints(cj) {
  for (int i = 0; i < kNumJoints; i++) {
    pidControllers[i] = PIDController(this, i);
  }
}

void BaseController::ControllerCallback(const geometry_msgs::Twist& t) {
  // TODO: parse command

  pidControl();
}

void BaseController::LinkStatesCallback(const gazebo_msgs::LinkStates& ls) {
  static int c = 0;
  ++c;

  for (auto i = 0; i < ls.name.size(); i++) {
    for (auto j = 0; j < kNumJoints; j++) {
      auto name = kLinkNames[j];
      if (ls.name[i].compare(name) == 0) {

        if (c % 64 == 0) {
        ROS_INFO("%s: %f %f %f, %f %f %f %f\n\t%f %f %f, %f %f %f",
            ls.name[i].c_str(), 
            ls.pose[i].position.x, ls.pose[i].position.y, ls.pose[i].position.z, 
            ls.pose[i].orientation.x, ls.pose[i].orientation.y, 
            ls.pose[i].orientation.z, ls.pose[i].orientation.w, 
            ls.twist[i].linear.x, ls.twist[i].linear.y, ls.twist[i].linear.z,
            ls.twist[i].angular.x, ls.twist[i].angular.y, ls.twist[i].angular.z);
        }


        // got link state
        // TODO: parse twist data
        break;
      }
    }
  }

  pidControl();
}

void BaseController::ModelStatesCallback(const gazebo_msgs::ModelStates& ms) {
  static int c = 0;
  ++c;

  for (auto i = 0; i < ms.name.size(); i++) {
  }

  pidControl();
}

void BaseController::pidControl() {
  for (auto &pc: pidControllers) {
    pc.pidControl();
  }
}
