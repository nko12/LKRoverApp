#include <fstream>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include "gazebo_msgs/SpawnModel.h"

#include "controller_manager/controller_manager.h"

#include "lk_rover/lk_rover.h"
#include "lk_rover/lk_rover_hw.h"
#include "lk_rover/gazebo_hw.h"

bool SpawnModel(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate);

int main(int argc, char** argv) {
  ros::init(argc, argv, "lk_rover_base");

  auto nh = ros::NodeHandle();
  auto nhPrivate = ros::NodeHandle("~");

  bool useGazebo = false;
  nhPrivate.param<bool>("use_gazebo", useGazebo, false);

  // TODO: make this less repetitive
  if (useGazebo) {
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

    ROS_INFO("spawning model...");
    // spawn the robot model in gazebo
    if (!SpawnModel(nh, nhPrivate)) {
      ROS_ERROR("unable to spawn model");
      return -1;
    }

    auto gazeboHW = std::make_shared<GazeboHW>();
    if(!gazeboHW->init(nh)) {
      ROS_ERROR("gazebo hw init failed");
      return -8;
    }
    LKRover robot(gazeboHW);
    controller_manager::ControllerManager cm(&robot, nh);
    cm.loadController("lk_velocity_controller");

    auto r = ros::Rate(100);
    auto curTime = ros::Time::now();
    while (true) {
      r.sleep();
      robot.read();
      cm.update(curTime, r.cycleTime());
      robot.write();
    }
  } else {
    LKRover robot(std::make_shared<LKRoverHW>());
    controller_manager::ControllerManager cm(&robot, nh);
    cm.loadController("lk_velocity_controller");

    auto r = ros::Rate(100);
    auto curTime = ros::Time::now();
    while (true) {
      r.sleep();
      robot.read();
      cm.update(curTime, r.cycleTime());
      robot.write();
    }
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
