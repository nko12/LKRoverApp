#include "ros/ros.h"
#include "ros/console.h"

#include "controller_manager/controller_manager.h"

#include "lk_rover/lk_rover.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lk_rover_base");

  auto nh = ros::NodeHandle();
  auto nhPrivate = ros::NodeHandle("~");

  LKRover robot;
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

  return 0;
}
