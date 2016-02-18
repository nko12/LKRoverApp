# LKRoverServer
The server application for controlling the Lunar Knights Mars Mining Robot.

The server application will likely contain a few different packages.

  1. The main executable that drives the robot and listens for commands from the client.
  2. A library that utilizes communication between the client and the server itself.
  3. A library that handles any artificial intelligence of the robot.
  
The server will more than likely utilize the server capabilities of ROS. All server code will be written in C++ or Python (more than likely C++), as this is a requirement of ROS itself.
