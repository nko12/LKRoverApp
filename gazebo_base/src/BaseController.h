#ifndef BASE_CONTROLLER_H_
#define BASE_CONTROLLER_H_

#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"

#include <array>

#include "common.h"
#include "PIDController.h"

class BaseController {
  friend class PIDController;

protected:
  std::array<PIDController, kNumJoints> pidControllers;
  geometry_msgs::Pose curModelPose;

  ros::ServiceClient &moveJoints, &clearJoints;

public:
  BaseController(ros::ServiceClient &mj, ros::ServiceClient &cj);

  void ControllerCallback(const geometry_msgs::Twist& t);
  void LinkStatesCallback(const gazebo_msgs::LinkStates& ls);
  void ModelStatesCallback(const gazebo_msgs::ModelStates& ms);

protected:
  void pidControl();
};

#endif
