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
roslaunch tesbot_keyboard tesbot_keyboard.launch

# on the separate terminal start up the Gazebo client
gzclient
```

## Useful references
http://wiki.ros.org/roscpp/Overview/Services
http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
http://wiki.ros.org/roscpp/Overview/Messages
http://gazebosim.org/tutorials?tut=ros_comm&cat=connect_ros
http://docs.ros.org/kinetic/api/gazebo_msgs/html/srv/JointRequest.html

# URDF Notes
urdf file is located in katkin_ws/src/tesbot_description/urdf

launch file is located in katkin_ws/src/tesbot_gazebo/launch
world file located in katkin_ws/src/tesbot_gazebo/worlds

To run in Gazebo:

requirements to run:
ros kinetic
gazebo and gazebo package for ros kinetic (have to install both seperately)

The following are some of the guides that i used to make the urdf/ make it useable in gazebo

guide to install gazebo ros package: http://gazebosim.org/tutorials?tut=ros_installing
guide to learning urdf: http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch
guide to useing the urdf in gazebo: http://gazebosim.org/tutorials?tut=ros_roslaunch


currently to get tesbot to render in gazebo it requires 2 steps first launching the gazebo instance and secondly spawning the


to launch gazebo (2 commands):

> . ~/katkin_ws/devel/setup.bash
> roslaunch tesbot_gazebo tesbot.launch


the setup bash only needs to be run once in the command line (so if restarting gazebo instances
you can just skip the setup.bash)
launches gazebo for specified instance for tesbot (currently an empty/ flat world)


to input urdf (1 command):

> rosrun gazebo_ros spawn_model -file 'tesbot_description'/urdf/tesbot.urdf -urdf -x 0 -y 0 -z1
-model tesbot

this command should be run while inside the katkin/src file path
you may possibly need to run this command in another terminal
working on finding the commands to allow the launcher to automaticly spawn in the urdf file

if you have any questions about this setup or how to run it feel free to ask me and i will do my best to help
- Wyatt
