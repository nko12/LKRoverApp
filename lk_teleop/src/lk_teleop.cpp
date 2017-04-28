#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <linux/joystick.h>

#include <algorithm>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
// lots of code copied from http://www.cs.uleth.ca/~holzmann/C/system/ttyraw.c
struct termios origTermios;

int main(int argc, char **argv) {
  ros::init(argc, argv, "lk_teleop");

  auto nh = ros::NodeHandle();
  auto nhPriv = ros::NodeHandle("~");

  ros::Publisher pubBase = nh.advertise<geometry_msgs::Twist>("/lk_velocity_controller/cmd_vel", 1);
  ros::Publisher pubLadderSpin = nh.advertise<std_msgs::Float64>("/ladder_spin", 1);
  ros::Publisher pubBucketLift = nh.advertise<std_msgs::Float64>("/lk_dump_controller/command", 1);
  ros::Publisher pubLadderLift = nh.advertise<std_msgs::Float64>("/lk_ladder_controller/command", 1);
  ros::Publisher pubFlapLift = nh.advertise<std_msgs::Float64>("/flap_lift", 1);

  int joy_fd = open("/dev/input/js0", O_RDONLY);
  if (joy_fd == -1) {
    fprintf(stderr, "unable to open controller\n");
    exit(-1);
  }

  int numAxes = 0, numButtons = 0;
  char joystickName[80];
	ioctl(joy_fd, JSIOCGAXES, &numAxes);
	ioctl(joy_fd, JSIOCGBUTTONS, &numButtons);
	ioctl(joy_fd, JSIOCGNAME(80), &joystickName);

  int *axes = new int[numAxes];
  char *buttons = new char[numButtons];

  ROS_INFO("Found joystick %s with %d axes and %d buttons", joystickName, numAxes, numButtons);

  struct js_event js;
  float bucketPos = 0.0;
  float ladderPos = 0.0;
  float flapPos = 0.0; 

  fd_set set;
  struct timeval timeout;


  while (!buttons[6]) {
    // printf("loop\n");

    FD_ZERO(&set);
    FD_SET(joy_fd, &set);
    timeout.tv_sec = 0;
    timeout.tv_usec = 20000;
    int rv = select(joy_fd + 1, &set, NULL, NULL, &timeout);
    if (rv > 0) {
	    if (read(joy_fd, &js, sizeof(js)) < 0) {
	      fprintf(stderr, "unable to read controller\n");
	      exit(-2);
	    }
	    switch (js.type & ~JS_EVENT_INIT) {
	      case JS_EVENT_AXIS:
		axes[js.number] = js.value;
		if(js.number == 0 || js.number == 1){
			float forceLin = axes[1];
			float forceAng = axes[0];
			if(forceLin <= 0.1*std::abs(forceAng) && forceLin >= -std::abs(forceAng))
				forceLin = 0;
			if(forceAng <= 0.1*std::abs(forceLin) && forceAng >= -std::abs(forceLin))
				forceAng = 0;
		        geometry_msgs::Twist msg = {};
		        msg.linear.x = forceLin/(-40*32767.0);
		        msg.angular.z = forceAng/(-40*32767.0);	
			pubBase.publish(msg);
		}
		if(js.number == 5){
			float force = axes[5];
		        std_msgs::Float64 msg;

			if(force >= 0.0)
		        	msg.data = (force/(32767.0));
			else
				msg.data = (0.0);
			pubLadderSpin.publish(msg);
		}
		if(js.number == 2){
			float force = axes[2];
			std_msgs::Float64 msg;
			if(force >= 0)
				msg.data = (-force/(32767.0));
			else
				msg.data = (0.0);
			pubLadderSpin.publish(msg);
		}
		
	break;
	      case JS_EVENT_BUTTON:
		buttons[js.number] = js.value;
		if(js.number == 8){
			float pos = buttons[8];
			std_msgs::Float64 msg;
			msg.data = pos;
			pubFlapLift.publish(msg);		
		}	

		break;
	    }
#if 0
	    for(int i = 0; i < numAxes; ++i)
	      printf("a %d:%6d ", i, axes[i]);
	    for(int i = 0; i < numButtons; ++i)
	      printf("b %d: %d ", i, buttons[i]);
	    printf("  \r");
	    fflush(stdout);
#endif
    } else if (rv == 0) {
       // printf("timeout\n");
       if (buttons[0] || buttons[1] || buttons[2] || buttons[3]) {
          if (buttons[1] || buttons[3]) {
            bucketPos += 0.01*(buttons[3] - buttons[1]);
            if (bucketPos > 1.00) bucketPos = 1.00;
            if (bucketPos < 0.00) bucketPos = 0.00;
            std_msgs::Float64 msg;
            msg.data = bucketPos;
            pubBucketLift.publish(msg);
          }
          if (buttons[2] || buttons[0]) {
            ladderPos += 0.01*(buttons[2] - buttons[0]);
            if (ladderPos > 1.00) ladderPos = 1.00;
            if (ladderPos < 0.00) ladderPos = 0.00;
            std_msgs::Float64 msg;
            msg.data = ladderPos;
            pubLadderLift.publish(msg);
          }
	if (buttons[6]){
	//stop
		close(joy_fd);
		return 0;
	}
	if (buttons[7]){
	//start
		
	}
       }
    } else {
      fprintf(stderr, "select() error\n");
    }
    ros::spinOnce();
  }

  close(joy_fd);

  return 0;
}
