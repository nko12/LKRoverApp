#include <algorithm>
#include "gazebo_msgs/ApplyJointEffort.h"
#include "gazebo_msgs/GetJointProperties.h"
#include "gazebo_msgs/JointRequest.h"

#include "lk_rover/common.h"
#include "lk_rover/lk_rover.h"
#include "lk_rover/gazebo_hw.h"

constexpr double kForceTol = 0.001;
constexpr int kMaxForceNum = 8;

GazeboHW::GazeboHW(): curEfforts{}, numEfforts{} {}

bool GazeboHW::init(ros::NodeHandle &nh) {
  mj = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort", true);
  cj = nh.serviceClient<gazebo_msgs::JointRequest>("/gazebo/clear_joint_forces", true);
  gj = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties", true);

  if (!mj) {
    ROS_ERROR("unable to connect to /gazebo/apply_joint_effort");
    return false;
  }
  if (!cj) {
    ROS_ERROR("unable to connect to /gazebo/clear_joint_forces");
    return false;
  }
  if (!gj) {
    ROS_ERROR("unable to connect to /gazebo/get_joint_properties");
    return false;
  }
  return true;
}

void GazeboHW::setPWMs(const std::array<double, kNumWheels>& newEfforts,
    double dumpA, double dumpB, double ladderA, double ladderB, double spin) {
  // TODO: set up the nonwheel actuators
  ROS_INFO("GazeboHW::setPWMs %f %f %f %f",
      newEfforts[0], newEfforts[1], newEfforts[2], newEfforts[3]);
  for (int i = 0; i < kNumWheels; ++i) {
    double clippedEfforts = newEfforts[i];
    if (newEfforts[i] > kMaxTorque) {
      clippedEfforts = kMaxTorque;
    } else if (newEfforts[i] < -kMaxTorque) {
      clippedEfforts = -kMaxTorque;
    }
    if (std::abs(clippedEfforts - curEfforts[i]) > kForceTol) {
      if (numEfforts[i] > kMaxForceNum) {
        gazebo_msgs::JointRequest cjf;
        cjf.request.joint_name = kWheelNames[i];
        if (!cj.call(cjf)) {
          ROS_WARN("GazeboHW::setPWMs: clear force %d failed", i);
          continue;
        }
        curEfforts[i] = 0.0;
      }
      const auto deltaForce = clippedEfforts - curEfforts[i];
      gazebo_msgs::ApplyJointEffort aje;
      aje.request.joint_name = kWheelNames[i];
      aje.request.effort = deltaForce;
      aje.request.duration = ros::Duration(-0.1f);
      if (!mj.call(aje)) {
        continue;
      } else if (!aje.response.success) {
        ROS_WARN("GazeboHW::setPWMs: %d aje request failed with %s",
            i, aje.response.status_message.c_str());
        continue;
      }
      curEfforts[i] += deltaForce;
      ++numEfforts[i];
    }
  }
}

void GazeboHW::getCount(std::array<double, kNumWheels>& count,
    double &dumpA, double& dumpB, double &ladderA, double &ladderB) {
  // TODO: set up the nonwheel actuators
  for (auto i = 0; i < kNumWheels; ++i) {
    gazebo_msgs::GetJointProperties gjp;

    gjp.request.joint_name = kWheelNames[i];
    if (!gj.call(gjp)) {
      ROS_WARN("GazeboHW::getCount get joints %d call failed", i);
      continue;
    }
    if (!gjp.response.success) {
      ROS_WARN("GazeboHW::getCount get joints %d call failed: %s", i,
          gjp.response.status_message.c_str());
      continue;
    }
    // ROS_INFO("position %d, %f", i, gjp.response.position[0]);
    count[i] = gjp.response.position[0];
  }
}

