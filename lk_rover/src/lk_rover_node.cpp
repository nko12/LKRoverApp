#include <atomic>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ros/ros.h"
#include "ros/console.h"

#include "std_msgs/Bool.h"

#include "gazebo_msgs/SpawnModel.h"

#include "controller_manager/controller_manager.h"
#include "controller_manager_msgs/SwitchController.h"

#include "lk_rover/lk_controller.h"
#include "lk_rover/lk_rover.h"
#include "lk_rover/lk_rover_hw.h"
#include "lk_rover/gazebo_hw.h"

std::atomic<bool> receivedHeartbeat(false);
void teleopHeartCb(const std_msgs::Bool &b) {
  receivedHeartbeat.store(true);
};

bool SpawnModel(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate);

int main(int argc, char** argv) {
  ros::init(argc, argv, "lk_rover_base");

  auto nh = ros::NodeHandle();
  auto nhPrivate = ros::NodeHandle("~");

  bool useGazebo = false;
  nhPrivate.param<bool>("use_gazebo", useGazebo, false);

  std::shared_ptr<LKHW> hw;
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
    hw = gazeboHW;
  } else {
    hw = std::make_shared<LKRoverHW>(nh);
    // wait for the rosserial link to connect
    dynamic_cast<LKRoverHW*>(hw.get())->waitForSerial();
  }

  ActuatorConfigs dumpConfigs = {0}, ladderConfigs = {0};

  bool configFail = false;
  if (!nh.getParam("dump/left/min", dumpConfigs.left.min)) {
    ROS_ERROR("unable to find dump left min");
    configFail = true;
  }
  if (!nh.getParam("dump/left/max", dumpConfigs.left.max)) {
    ROS_ERROR("unable to find dump left max");
    configFail = true;
  }
  if (!nh.getParam("dump/left/gain", dumpConfigs.left.gain)) {
    ROS_ERROR("unable to find dump left gain");
    configFail = true;
  }
  if (!nh.getParam("dump/right/min", dumpConfigs.right.min)) {
    ROS_ERROR("unable to find dump right min");
    configFail = true;
  }
  if (!nh.getParam("dump/right/max", dumpConfigs.right.max)) {
    ROS_ERROR("unable to find dump left max");
    configFail = true;
  }
  if (!nh.getParam("dump/right/gain", dumpConfigs.right.gain)) {
    ROS_ERROR("unable to find dump right gain");
    configFail = true;
  }
  if (!nh.getParam("dump/diff_gain", dumpConfigs.diffGain)) {
    ROS_ERROR("unable to find dump diff gain");
    configFail = true;
  }
  if (!nh.getParam("dump/length", dumpConfigs.length)) {
    ROS_ERROR("unable to find dump diff length");
    configFail = true;
  }
  ROS_INFO("dump params: %lf %lf %lf %lf %lf %lf %lf", 
    dumpConfigs.left.min, dumpConfigs.left.max, dumpConfigs.left.gain,
    dumpConfigs.right.min, dumpConfigs.right.max, dumpConfigs.right.gain,
    dumpConfigs.diffGain);

  if (!nh.getParam("ladder/left/min", ladderConfigs.left.min)) {
    ROS_ERROR("unable to find ladder left min");
    configFail = true;
  }
  if (!nh.getParam("ladder/left/max", ladderConfigs.left.max)) {
    ROS_ERROR("unable to find ladder left max");
    configFail = true;
  }
  if (!nh.getParam("ladder/left/gain", ladderConfigs.left.gain)) {
    ROS_ERROR("unable to find ladder left gain");
    configFail = true;
  }
  if (!nh.getParam("ladder/right/min", ladderConfigs.right.min)) {
    ROS_ERROR("unable to find ladder right min");
    configFail = true;
  }
  if (!nh.getParam("ladder/right/max", ladderConfigs.right.max)) {
    ROS_ERROR("unable to find ladder left max");
    configFail = true;
  }
  if (!nh.getParam("ladder/right/gain", ladderConfigs.right.gain)) {
    ROS_ERROR("unable to find ladder right gain");
    configFail = true;
  }
  if (!nh.getParam("ladder/diff_gain", ladderConfigs.diffGain)) {
    ROS_ERROR("unable to find ladder diff gain");
    configFail = true;
  }
  if (!nh.getParam("ladder/length", ladderConfigs.length)) {
    ROS_ERROR("unable to find ladder diff length");
    configFail = true;
  }
  ROS_INFO("ladder params: %lf %lf %lf %lf %lf %lf %lf", 
    ladderConfigs.left.min, ladderConfigs.left.max, ladderConfigs.left.gain,
    ladderConfigs.right.min, ladderConfigs.right.max, ladderConfigs.right.gain,
    ladderConfigs.diffGain);

  if (configFail) exit(-1);

  LKRover robot(hw, dumpConfigs, ladderConfigs);
  controller_manager::ControllerManager cm(&robot, nh);
  cm.loadController("lk_velocity_controller");
  cm.loadController("lk_dump_controller");
  cm.loadController("lk_ladder_controller");
  cm.loadController("lk_spin_controller");
  cm.loadController("lk_flap_controller");

  bool teleopMode = false;

  std::atomic<bool> running(true);
  std::atomic<bool> killMotors(false);

  auto heartSub = nh.subscribe("heartbeat", 1, teleopHeartCb);

  std::thread controlThread([&]() {
    auto r = ros::Rate(50);
    auto curTime = ros::Time::now();
    while (running) {
      r.sleep();
      robot.read();
      if (!killMotors) {
        cm.update(curTime, r.cycleTime());
      } else {
        robot.killMotors();
      }
      robot.write();
      // ROS_INFO("control loop");
    }
  });

  auto toStart = std::vector<std::string>{
    "lk_velocity_controller",
    "lk_dump_controller",
    "lk_ladder_controller",
    "lk_spin_controller",
    "lk_flap_controller",
  };
  auto toStop = std::vector<std::string>{};
  cm.switchController(
      toStart,
      toStop,
      2); // STRICT

  auto master = LKController(nh, nhPrivate);

  auto r = ros::Rate(100);
  const int kTimeoutTime = 20;
  auto teleopTimeout = kTimeoutTime;
  while (ros::ok()) {
    r.sleep();
    if (!teleopMode) {
      master.doStuff();
    } else {
      // teleop mode;
      if (!receivedHeartbeat) {
        --teleopTimeout;
      } else {
        teleopTimeout = kTimeoutTime;
        killMotors.store(false);
      }
      if (teleopTimeout == 0) {
        ROS_WARN("teleop connection loss detected; killing motors");
        killMotors.store(true);
      }
    }
    if (receivedHeartbeat) {
      teleopMode = true;
    }
    receivedHeartbeat.store(false);
    ros::spinOnce();
  }

  running.store(false);
  controlThread.join();

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
