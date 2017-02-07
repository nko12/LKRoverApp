#include "ros/ros.h"
#include "ros/console.h"

#include "PIDController.h"

#include "BaseController.h"

void PIDController::pidControl() {
  desiredForce = 0.2f;
  setForce();
}

void PIDController::setForce() {
  if (std::abs(desiredForce - curForce) > 0.001f) {
    if (nForceApplications > kMaxForceNum) {
      // clear forces
      clearForce();
      curForce = 0.0f;
    }
    auto deltaForce = desiredForce - curForce;
    if (addForce(deltaForce)) {
      ++nForceApplications;
      curForce += deltaForce;
    } else {
      ROS_WARN("BaseController::addForce(%d, %f) failed", idx, deltaForce);
    }
  }
}

void PIDController::clearForce() {
  gazebo_msgs::JointRequest cjf;
  cjf.request.joint_name = kJointNames[idx];
  if (!parent -> clearJoints.call(cjf)) {
    ROS_WARN("BaseController::clearForce(%d) failed", idx);
  }
}

bool PIDController::addForce(float f) {
  gazebo_msgs::ApplyJointEffort aje;
  aje.request.joint_name = kJointNames[idx];
  aje.request.effort = f;
  aje.request.duration = ros::Duration(-0.1f);
  if (!parent -> moveJoints.call(aje)) {
    return false;
  } else if (!aje.response.success) {
    ROS_WARN("BaseController::addForce(%d, %f): aje request returned with %s",
        idx, f, aje.response.status_message.c_str());
  }
  return aje.response.success;
}



