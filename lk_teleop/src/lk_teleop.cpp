#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
// lots of code copied from http://www.cs.uleth.ca/~holzmann/C/system/ttyraw.c
struct termios origTermios;

int main(int argc, char **argv) {
  ros::init(argc, argv, "lk_teleop");

  auto nh = ros::NodeHandle();
  auto nhPriv = ros::NodeHandle("~");

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

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
  while (1) {
    if (read(joy_fd, &js, sizeof(js)) < 0) {
      fprintf(stderr, "unable to read controller\n");
      exit(-2);
    }
    switch (js.type & ~JS_EVENT_INIT) {
      case JS_EVENT_AXIS:
        axes[js.number] = js.value;
        break;
      case JS_EVENT_BUTTON:
        buttons[js.number] = js.value;
        break;
    }
		for(int i = 0; i < numAxes; ++i)
			printf("a %d:%6d ", i, axes[i]);
		for(int i = 0; i < numButtons; ++i)
			printf("b %d: %d ", i, buttons[i]);
		printf("  \r");
		fflush(stdout);
  }

  close(joy_fd);

  return 0;
}
