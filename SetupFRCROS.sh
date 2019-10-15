#!/bin/bash

cwd=$PWD

echo "Currernt working directory:" $cwd


# Auto Install ROS

ros_install=ros-kinetic-desktop-full
pip_install=python-pip

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt-get install ${ros_install} ${pip_install}
sudo rosdep init
rosdep update

#install network tables for ntbridge
python -m pip install --user pynetworktables==2018.2.0

# Check ~/.bashrc for sourcing

mkdir -p ~/catkin_ws/src

if grep -q 'source /opt/ros/kinetic/setup.bash' ~/.bashrc; then
    echo "~/.bashrc line 1 found"
else
    echo -e "\e[93m~/.bashrc line NOT found. Adding line to EOF.\e[39m"
    echo 'source /opt/ros/kinetic/setup.bash' >> ~/.bashrc
fi

if grep -q 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc; then
    echo "~/.bashrc line 2 found"
else
    echo -e "\e[93m~/.bashrc line NOT found. Adding line to EOF.\e[39m"
    echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
fi

sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Copy packages

rm -rf ~/catkin_ws/src/frc_robot/
cp -R $cwd/catkin_ws/src/frc_robot ~/catkin_ws/src/

rm -rf ~/catkin_ws/src/networktable_bridge/
cp -R $cwd/catkin_ws/src/networktable_bridge ~/catkin_ws/src/

rm -rf ~/catkin_ws/src/swerve_controller/
cp -R $cwd/catkin_ws/src/swerve_controller ~/catkin_ws/src/

# Extract supporting packages

file=$cwd/common-sensors.tar.gz
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/catkin_ws/src"
    tar -xf "$file" -C ~/catkin_ws/src
fi

file=$cwd/rplidar_ros.tar.gz
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/catkin_ws/src"
    tar -xf "$file" -C ~/catkin_ws/src
fi


mkdir -p ~/.gazebo/models
file=$cwd/kinect_ros.tar.gz
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/.gazebo/models"
    tar -xf "$file" -C ~/.gazebo/models
fi

# Modification to the lidar to extend the range to that of the RPLidar (12 meters).
file=$cwd/hokuyo_04lx_laser.gazebo.xacro
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/catkin_ws/src/common-sensors/common_sensors/urdf/sensors/"
    cp "$file" ~/catkin_ws/src/common-sensors/common_sensors/urdf/sensors/
fi

# Kinect IR Calibration
file=$cwd/depth_B00364725109104B.yaml
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/.ros/camera_info/"
    cp "$file" ~/.ros/camera_info/
fi

# Kinect Camera Calibration
file=$cwd/rgb_B00364725109104B.yaml
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/.ros/camera_info/"
    cp "$file" ~/.ros/camera_info/
fi

# Auto Install Dependencies

rosdep install --from-paths ~/catkin_ws/src --ignore-src --rosdistro=kinetic -y


# Build everything

source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash



# Check dependencies

cd ~/catkin_ws/src/frc_robot
roswtf
cd ~/catkin_ws/src/networktable_bridge
roswtf

