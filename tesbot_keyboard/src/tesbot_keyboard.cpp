#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/LinkStates.h"

#include "gazebo_msgs/ApplyJointEffort.h"
#include "gazebo_msgs/JointRequest.h"
#include "gazebo_msgs/SpawnModel.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

int count = 0;

void PositionCallback(const gazebo_msgs::ModelStates& ms) {
  // log model positions
  // only log every 16th
  ++count;

  if ((count % 64) == 0) {
    int idx = -1;
    for (int i = 0; i < ms.name.size(); ++i) {
      if (ms.name[i] == "tesbot") {
        idx = i;
        ROS_INFO("pose: position %f %f %f orientation %f %f %f %f\n\ttwist: linear %f %f %f angular %f %f %f",
            ms.pose[i].position.x, ms.pose[i].position.y, ms.pose[i].position.z,
            ms.pose[i].orientation.x, ms.pose[i].orientation.y, ms.pose[i].orientation.z, ms.pose[i].orientation.w, 
            ms.twist[i].linear.x, ms.twist[i].linear.y, ms.twist[i].linear.z,
            ms.twist[i].angular.x, ms.twist[i].angular.y, ms.twist[i].angular.z);
        break;
      }
    }

    if (idx == -1) {
      ROS_INFO("model not found");
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tesbot_keyboard_node");

  auto nh = ros::NodeHandle();

  // set up private node handler to get parameters, as per
  // http://answers.ros.org/question/11098/roscpp-relative-parameter/
  auto nhPrivate = ros::NodeHandle("~");

  // sleep for some amount of time to ensure Gazebo is up and running
  int sleep_time = 5;
  nhPrivate.param<int>("sleep", sleep_time, 5);

  std::this_thread::sleep_for(std::chrono::seconds(sleep_time));

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
    ROS_INFO("spawning robot..");
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
        return -2;
      }
    } else {
      ROS_ERROR("unable to connect to model spawner");
      return -1;
    }
  }

  std::mutex lock;
  auto reader_running = true;

  auto model_state = nh.subscribe("/gazebo/model_states", 8, PositionCallback);
  if (!model_state) {
    ROS_ERROR("unable to subscribe to model_states");
    return -4;
  }

  auto reading_thread = std::thread([&]() {
    auto aje_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort", true); 
    auto cjf_client = nh.serviceClient<gazebo_msgs::JointRequest>("/gazebo/clear_joint_forces", true); 

    if (!aje_client) {
      ROS_ERROR("unable to connect to /gazebo/apply_joint_effort");
      std::lock_guard<std::mutex> guard(lock);
      reader_running = false;
      return;
    }
    if (!cjf_client) {
      ROS_ERROR("unable to connect to /gazebo/clear_joint_forces");
      std::lock_guard<std::mutex> guard(lock);
      reader_running = false;
      return;
    }


    const auto left_joints = std::vector<std::string>{
      "base_to_left_front_wheel",
      "base_to_left_back_wheel"
    };
    const auto right_joints = std::vector<std::string>{
      "base_to_right_front_wheel",
      "base_to_right_back_wheel"
    };


    auto c = std::cin.get();
    while (c != 'q') {
      auto valid = false,
           stop = false,
           left_forwards = true,
           right_forwards = true;

      switch (c) {
      case 'w':
      // go forwards
        stop = false;
        valid = true;
        left_forwards = true;
        right_forwards = true;
        ROS_INFO("back");
        break;
      case 's':
      // go back
        stop = false;
        valid = true;
        left_forwards = false;
        right_forwards = false;
        ROS_INFO("forwards");
        break;
      case 'a':
      // go left
        stop = false;
        valid = true;
        left_forwards = false;
        right_forwards = true;
        ROS_INFO("left");
        break;
      case 'd':
      // go right
        stop = false;
        valid = true;
        left_forwards = true;
        right_forwards = false;
        ROS_INFO("right");
        break;
      case '\r':
      case '\n':
      case ' ':
        break;
      default:
      // stop
        valid = true;
        stop = true;
        ROS_INFO("stop");
        break;
      }

      if (valid) {
        if (stop) {
          gazebo_msgs::JointRequest cjf;
          for (auto &left_joint: left_joints) {
            cjf.request.joint_name = left_joint;

            if (!cjf_client.call(cjf)) {
              ROS_ERROR("joint clear call failed");
            }
          }
          for (auto &right_joint: right_joints) {
            cjf.request.joint_name = right_joint;

            if (!cjf_client.call(cjf)) {
              ROS_ERROR("joint clear call failed");
            }
          }
        } else {
          gazebo_msgs::ApplyJointEffort aje;
          for (auto &left_joint: left_joints) {
            aje.request.joint_name = left_joint;
            aje.request.effort = left_forwards ? 2.0f : -2.0f;
            aje.request.duration = ros::Duration(-1.0f);

            if (aje_client.call(aje)) {
              if (!aje.response.success) {
                ROS_ERROR("joint effort request failed %s: %s",
                    aje.request.joint_name.c_str(), aje.response.status_message.c_str());
              }
            } else {
              ROS_ERROR("joint effort call failed");
            }

          }
          for (auto &right_joint: right_joints) {
            aje.request.joint_name = right_joint;
            aje.request.effort = right_forwards ? 2.0f : -2.0f;
            aje.request.duration = ros::Duration(-1.0f);

            if (aje_client.call(aje)) {
              if (!aje.response.success) {
                ROS_ERROR("joint effort request failed %s: %s",
                    aje.request.joint_name.c_str(), aje.response.status_message.c_str());
              }
            } else {
              ROS_ERROR("joint effort call failed");
            }
          }
        }
      }

      c = std::cin.get();
    }

    {
      std::lock_guard<std::mutex> guard(lock);
      reader_running = false;
    }

  });

  auto r = ros::Rate(100);

  bool still_running;
  {
    std::lock_guard<std::mutex> guard(lock);
    still_running = reader_running;
  }

  while (still_running) {
    ros::spinOnce();
    r.sleep();
    {
      std::lock_guard<std::mutex> guard(lock);
      still_running = reader_running;
    }
  }

  ros::shutdown();

  while (ros::ok()) {
    r.sleep();
  }

  return 0;
}
