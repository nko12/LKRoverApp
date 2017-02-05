#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

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

bool SpawnModel(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate);

constexpr char kLeftBackName[] = "base_to_left_back_wheel";
constexpr char kLeftFrontName[] = "base_to_left_front_wheel";
constexpr char kRightBackName[] = "base_to_right_back_wheel";
constexpr char kRightFrontName[] = "base_to_right_front_wheel";


class BaseController {
public:
  float desiredVels[4];
  float curVels[4];
  float curForces[4];

  // so the way Gazebo works means that every time you apply a force it generates
  // a new force that acts on the object
  // this means it'll gradually slow down, so this is here to figure out when
  // to consolidate forces to reduce the workload on gazebo
  int nForceApplications[4];

  static const char* kNameOrdering[4];

  ros::ServiceClient *moveJoints, *clearJoints;

  BaseController(): moveJoints(nullptr), clearJoints(nullptr) {
    for (int i = 0; i < 4; i++) {
      desiredVels[i] = curVels[i] = curForces[i] = 0.0f;
      nForceApplications[i] = 0;
    }
  }

  void ControllerCallback(const geometry_msgs::Twist& t) {
    if (!isReady()) {
      return;
    }

    // TODO: parse command


    pidControl();
  }

  void LinkStatesCallback(const gazebo_msgs::LinkStates& ls) {
    float leftBack, leftFront, rightBack, rightFront;
    for (auto i = 0; i < ls.name.size(); i++) {
      for (auto j = 0; j < 4; j++) {
        if (ls.name[i].compare(BaseController::kNameOrdering[i]) == 0) {
          // got link state
          // TODO: parse twist data
          break;
        }
      }
    }

    pidControl();
  }
private:
  bool isReady() {
    return moveJoints != nullptr && clearJoints != nullptr;
  }

  void pidControl() {
    if (!isReady()) {
      return;
    }
    // TODO: PID calculations or whatever
  }
};

const char* BaseController::kNameOrdering[4] = {
    kLeftFrontName,
    kLeftBackName,
    kRightFrontName,
    kRightBackName
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "gazebo_base");

  auto nh = ros::NodeHandle();

  // set up private node handler to get parameters, as per
  // http://answers.ros.org/question/11098/roscpp-relative-parameter/
  auto nhPrivate = ros::NodeHandle("~");

  auto bc = BaseController();

  auto base_controller_sub = nh.subscribe("cmd_vel", 1, &BaseController::ControllerCallback, &bc);

  // wait for gazebo to come up
  {
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

  // spawn the robot model in gazebo
  if (!SpawnModel(nh, nhPrivate)) {
    ROS_ERROR("unable to spawn model");
    return -1;
  }

  // subscribe to link state data to do PID control
  auto link_state = nh.subscribe("/gazebo/link_states", 1, &BaseController::LinkStatesCallback, &bc);
  if (!link_state) {
    ROS_ERROR("unable to subscribe to link_states");
    return -2;
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

  bc.moveJoints = &aje_client;
  bc.clearJoints = &cjf_client;

  auto r = ros::Rate(100);

  while (true) {
    ros::spinOnce();
    r.sleep();
  }

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
        return false;
      }
    } else {
      ROS_ERROR("unable to connect to model spawner");
      return false;
    }
  }
  return true;
}


