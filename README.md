# LKRoverApp
The primary code for controlling the Lunar Knights Mars Mining Robot.

The Lunar Knights application will likely contain a few different packages.

  1. An executable that drives the robot and listens for commands from the client, which runs on the robot server computer.
  2. An executable that is used to control the robot, which runs on the client computer.
  3. A library that utilizes communication between the client and the server itself. This would be ROS itself.
  4. A library that handles any artificial intelligence of the robot. This would probably be a library that comes with ROS.
  
The code for the robot will likely be written in 3 languages: C, C++ and Python.
  1. We can use C for direct manipulation of the components connected to the Arduino (such as motors driving the wheels).
  2. We can use Python as an easier way to deal with making a GUI that controls the robot. GUI in C++ is generally not fun.
  3. We can use C++ for everything else that isn't GUI or direct manipulation of the robot hardware.
  
# Install dependencies
```

sudo apt-get install ros-kinetic-diff_drive_controller

sudo apt-get install ros-kinetic-controller_manager

```
# How to install

```
# create a folder to put the code in
mkdir -p ~/catkin_ws

# copy the repository into ~/catkin_ws/src
git clone https://github.com/Algias/LKRoverApp.git src/

# set up the catkin workspace
cd ~/catkin_ws/src
catkin_init_workspace

# build the project
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
catkin_make
```

# How to use tesbot
```
# get the environment for our catkin workspace
source ~/catkin_ws/devel/setup.bash

# launch the tesbot_keyboard node
roslaunch lk_rover gazebo_test

# on the separate terminal start up the Gazebo client
gzclient

#Send a velocity command through twist message with requested velocities.
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: -0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

## Useful references
http://wiki.ros.org/roscpp/Overview/Services
http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
http://wiki.ros.org/roscpp/Overview/Messages
http://gazebosim.org/tutorials?tut=ros_comm&cat=connect_ros
http://docs.ros.org/kinetic/api/gazebo_msgs/html/srv/JointRequest.html

