#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"

#include "gazebo_msgs/ApplyJointEffort.h"
#include "gazebo_msgs/JointRequest.h"
#include "gazebo_msgs/SpawnModel.h"

#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include "main.h"

const char* kJointNames[kNumJoints] = {
    kJointLeftFrontName,
    kJointLeftBackName,
    kJointRightFrontName,
    kJointRightBackName
};
const char* kLinkNames[kNumJoints] = {
    kLinkLeftFrontName,
    kLinkLeftBackName,
    kLinkRightFrontName,
    kLinkRightBackName
};



int main(int argc, char **argv) {
  ros::init(argc, argv, "gazebo_base");

  auto nh = ros::NodeHandle();

  // set up private node handler to get parameters, as per
  // http://answers.ros.org/question/11098/roscpp-relative-parameter/
  auto nhPrivate = ros::NodeHandle("~");

  // wait for gazebo to come up
  {
    ROS_INFO("waiting for gazebo...");
    int timeout_count = 5;
    int timeout_time = 5;
    while (timeout_count > 0) {
      if (ros::service::waitForService("/gazebo/spawn_urdf_model", timeout_time)) {
        break;
      }
      timeout_count--;
      ROS_INFO("/gazebo/spawn_urdf_model connection timed out, retry %d", 5-timeout_count);
    }
    if (timeout_count <= 0) {
      ROS_ERROR("unable to connect to gazebo");
      return -5;
    }
  }

  ROS_INFO("spawning model...");
  // spawn the robot model in gazebo
  if (!SpawnModel(nh, nhPrivate)) {
    ROS_ERROR("unable to spawn model");
    return -1;
  }

  auto aje_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort", true); 
  auto cjf_client = nh.serviceClient<gazebo_msgs::JointRequest>("/gazebo/clear_joint_forces", true); 

  if (!aje_client) {
    ROS_ERROR("unable to connect to /gazebo/apply_joint_effort");
    return -3;
  }
  if (!cjf_client) {
    ROS_ERROR("unable to connect to /gazebo/clear_joint_forces");
    return -4;
  }

  auto bc = BaseController(aje_client, cjf_client);

  auto base_controller_sub = nh.subscribe("cmd_vel", 1, &BaseController::ControllerCallback, &bc);

  // subscribe to link and model state data to do PID control
  auto link_state = nh.subscribe("/gazebo/link_states", 1, &BaseController::LinkStatesCallback, &bc);
  if (!link_state) {
    ROS_ERROR("unable to subscribe to link_states");
    return -2;
  }

  auto model_state = nh.subscribe("/gazebo/model_states", 1, &BaseController::ModelStatesCallback, &bc);
  if (!model_state) {
    ROS_ERROR("unable to subscribe to model_states");
    return -7;
  }

  auto r = ros::Rate(100);

  ros::spin();

  ros::shutdown();

  while (ros::ok()) {
    r.sleep();
  }

  return 0;
}

bool SpawnModel(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate) {
  // copy the model into a string to pass into the model spawner
  std::string model_path = "";
  std::stringstream model;

  if (!nhPrivate.getParam("model_path", model_path)) {
    ROS_ERROR("no model path specified");
    ROS_ERROR("path: %s", model_path.c_str());
    return -3;
  }
  ROS_INFO("opening model %s", model_path.c_str());
  {
    auto model_file = std::ifstream(model_path.c_str());
    model << model_file.rdbuf();
  }

  // spawn model
  std::string model_name = "";
  {
    auto model_spawner = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    gazebo_msgs::SpawnModel sm;
    sm.request.model_name = "tesbot";
    sm.request.model_xml = model.str();
    sm.request.robot_namespace = "tesbot";
    // sm.request.initial_pose;

    if (model_spawner.call(sm)) {
      if (sm.response.success) {
        ROS_INFO("robot spawn successful");
        model_name = sm.request.model_name;
      } else {
        ROS_ERROR("spawn attempt failed");
        ROS_ERROR("error message: %s", sm.response.status_message.c_str());
        return false;
      }
    } else {
      ROS_ERROR("unable to connect to model spawner");
      return false;
    }
  }
  return true;
}

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
