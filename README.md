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

# How to use

```
# create a folder to put the code in
mkdir -p ~/catkin_ws

# copy the repository into ~/catkin_ws/src
git clone https://github.com/Algias/LKRoverApp.git src/

cd ~/catkin_ws/src
# set up the catkin workspace
catkin_init_workspace

# build the project
cd ~/catkin_ws
catkin_make
```
