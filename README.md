Simulation Demo: https://www.youtube.com/watch?v=qSjgmY7hYsY

This is the source code for using ROS with an FRC Robot.

These steps are identical for desktop PC's, laptops, Raspberry PI, NVidia Jetson.
The "catkin_ws" folder is here simply for viewing. This will probably be changed later.
All code is automatically extracted placed in its appropriate location using "./SetupFRCRobot.sh".

Step 1: Make sure you have Ubuntu 16.04 installed.

Step 1a: Install NVidia Jetpack 3.3 (NVidia Developer membership required): https://developer.nvidia.com/embedded/dlc/jetpack-install-guide-3_3

Step 2: Install ROS Kinetic using the following guide: http://wiki.ros.org/kinetic/Installation/Ubuntu

Step 3: Run './SetupFRCRobot.sh'

Step 4: Run 'roslaunch frc_robot main_control.launch sim:=true visualize:=true slam:=true teleop:=true' and that will bring up a simulation.
Note: You need to hold down the "A" button on the Xbox controller to enable driving control.
You can hold down "B" for turbo speed.


The options include the following and can be typed in similar to above:
- auton: Enables autonomous nagivation and driving.
- robot: FOR THE REAL ROBOT. This brings up all sensors and the bridge to the roboRIO.
- sim: Brings up a simulation.
- slam: Enables SLAM (Simeltaneous Localization and Mapping)
- teleop: Enables teleop control through either GUI or any joystick such as an XBox controller.



------------------------------------------------------------------------------------------------------------------------




So far, everything is in simulation. Work is being done to bring it to an actual robot.
The end goal is the user should not be expected to know ROS or to even have to change anything on the Jetson.
The goal will be to create a Jetson image that can be used out-of-the-box, as well as a C++ class on the RoboRio that handles all interfacing with the Jetson.
All that the user will have to do is give it a few parameters such as drivetrain info and location of sensors (kinect and lidar) on the robot.



First on the to-do list is stuff that needs to be done to get the system operating at a basic level on a real robot:

1) Implement a network tables interface for the Jetson.
Create a ROS node that interfaces the RoboRio to the Jetson as essentially a 1-to-1 translation from network tabeles to ROS topics.

Interface will be like this:
RoboRio Software (RoboRio) <--Network Tables--> ROS Converter Node (Jetson) <-- ROS Topics --> ROS Software (Jetson)

2) Implement RoboRio software to interface with the Jetson.
This will be a Class that will send and receive data.
The goal will be for the user to create a class object that contains functions for everything the user could need.

These functions will boil down to (stupidly simplified):
- Map the field, or navigate only (requires existing map)
- Send IMU and Wheel data
- Set navigation/targeting goal
- Get velocity commands
- Get various data and statuses

3) Send IMU and wheel data from RoboRio to Jetson.

4) Take velocity commands from Jetson and make the robot move at that velocity.




Below is some of the data that will be passed between the RoboRio and Jetson.
Note that the robot will be assumed to be working in a plane, which means no Z movement and no pitch or roll.

Data from RoboRio -> Jetson:
- IMU Data: XYZ Angle, XYZ Angular Velocity, XYZ Acceleration
- Wheel Data: XY Linear Velocity, Z Angular Velocity
- Commands (with parameters): Track Target, SLAM commands (mapping, navigation goal, etc)

Data from Jetson -> RoboRio:
- Velocity Commands: XY Linear Velocity, Z Angular Velocity
- Odometry: XYZ Coordinates, XYZ Angle, XYZ Linear Velocity, XYZ Angular Velocity
- Target tracking data
- Commands being executed w/ parameters
- Various Statuses




Below here are features to be added once everythig works at a basic level:

1) Create a complete image for the Jetson that can be flashed and used out-of-the-box.

2) Implement a ROS node that does vision tracking. The user will have to write the code for the actual processing.
Maybe it can be done in Python so that the code doesn't have to be compiled.

For simplification, all camera data will be published over ROS topics.
These will include images, and depth data where applicable (such as from a Kinect or stereo camera).
The image will be processed in its own ROS node and output target coordinates.
A stereo camera may require some of the available GPU power.
The Jetson has CUDA cores, which will significantly boost image processing capabilities.

3) If the user wants to change the characteristics of the robot in ROS, they need to modify the URDF and YAML files in './catkin_ws/src/frc_robot' and upload those to the Jetson.
There are a ton of parameters contained in the YAML and Launch files, so they will need to be set to values that can be used in a variety of robots, then fine tuning done to fit a particular robot.

It may be helpful to make it so that some simpler parameters can be changed on the Jetson from the RoboRio to allow users to have some control over things without too much hassle.
Such parameters include:
- Drivetrain (Holonomic/Non-Holonomic, max acceleration and velocity)
- Robot Dimensions (for a more complete URDF beyond a simple square chassis, the user will have to do that)
- Sensor locations on Robot (kinect, lidar, camera, etc)

4) Implementing deep learning inference will be a strech goal. The goal of this will be to train the Jetson on targets and field elements relevent to a particular game.

5) Implement more advanced commands from the RoboRIO to the Jetson for SLAM and Navigation.




Recommended tutorials for current content (this is the only stuff that's immediately important):
- http://wiki.ros.org/ROS/Tutorials
- http://wiki.ros.org/urdf/Tutorials


Tutorials for future development:
- http://gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros
- http://wiki.ros.org/rtabmap_ros/Tutorials
- http://wiki.ros.org/navigation/Tutorials
- https://roscon.ros.org/2015/presentations/robot_localization.pdf


More relevant content:

- http://docs.ros.org/kinetic/api/robot_localization/html/index.html
- http://wiki.ros.org/taraxl-ros-package
- http://wiki.ros.org/rplidar
- http://wiki.ros.org/rtabmap_ros
- http://wiki.ros.org/freenect_launch


This stuff is more advanced:
- https://github.com/dusty-nv/ros_deep_learning
- https://developer.nvidia.com/embedded/twodaystoademo

# FRC-ROS
