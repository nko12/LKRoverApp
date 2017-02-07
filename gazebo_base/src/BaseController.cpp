#include "ros/ros.h"
#include "ros/console.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "geometry_msgs/Pose.h"

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
  for (auto i = 0; i < ls.name.size(); i++) {
    for (auto j = 0; j < kNumJoints; j++) {
      auto name = kLinkNames[j];
      if (ls.name[i].compare(name) == 0) {
        tf2::Transform tmp;
        tf2::fromMsg(ls.pose[i], tmp);
        wheelPoses[j] = curModelPose.inverseTimes(tmp);


        geometry_msgs::Pose poseTmp;
        tf2::toMsg(wheelPoses[j], poseTmp);

#if 0
        ROS_INFO("%s: %f %f %f, %f %f %f %f",
            ls.name[i].c_str(),
            poseTmp.position.x,
            poseTmp.position.y,
            poseTmp.position.z,
            poseTmp.orientation.x,
            poseTmp.orientation.y,
            poseTmp.orientation.z,
            poseTmp.orientation.w);
#endif

        // got link state
        break;
      }
    }
  }

  pidControl();
}

void BaseController::ModelStatesCallback(const gazebo_msgs::ModelStates& ms) {
  for (auto i = 0; i < ms.name.size(); i++) {
    if (ms.name[i].compare(kRobotName)) {
      tf2::fromMsg(ms.pose[i], curModelPose);
    }
  }

  pidControl();
}

void BaseController::pidControl() {
  for (auto &pc: pidControllers) {
    pc.pidControl();
  }
}
