#!/bin/bash

cwd=$PWD

echo "Currernt working directory:" $cwd


release=$(lsb_release -cs)  # Get codename
ros_version="none"


if [ $release == "focal" ]; then    # Ubuntu 20.04 and variants
       echo "OS is Ubuntu 20.04. Using ROS Noetic."
       ros_version="noetic"
elif [ $release == "bionic" ]; then
       echo "OS is Ubuntu 18.04. Using ROS Melodic."
       ros_version="melodic"
elif [ $release == "xenial" ]; then
       echo "OS is Ubuntu 16.04. Using ROS Kinetic."
       ros_version="kinetic"
else
       echo "OS is not a compatible version. Exiting."
       exit 1
fi


# Auto Install ROS

ros_install=ros-$ros_version-desktop-full
pip_install=python-pip
rsync_install=rsync

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt-get install ${ros_install} ${pip_install} ${rsync_install} 
sudo rosdep init
rosdep update

# Check ~/.bashrc for sourcing

mkdir -p ~/catkin_ws/src

if grep -q 'source /opt/ros/$ros_version/setup.bash' ~/.bashrc; then
    echo "~/.bashrc line 1 found"
else
    echo -e "\e[93m~/.bashrc line NOT found. Adding line to EOF.\e[39m"
    echo 'source /opt/ros/$ros_version/setup.bash' >> ~/.bashrc
fi

if grep -q 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc; then
    echo "~/.bashrc line 2 found"
else
    echo -e "\e[93m~/.bashrc line NOT found. Adding line to EOF.\e[39m"
    echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
fi

sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Copy packages

cd $cwd/catkin_ws/src

for D in *; do
    if [ -d "${D}" ]; then
        echo "${D}"   # your processing here
        rsync -r -t -v --progress --delete -u -c -s $cwd/catkin_ws/src/${D}/ ~/catkin_ws/src/${D}
    fi
done

# Extract supporting packages

cd $cwd

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

rosdep install --from-paths ~/catkin_ws/src --ignore-src --rosdistro=$ros_version -y


# Build everything

source /opt/ros/$ros_version/setup.bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash



# Check dependencies

cd ~/catkin_ws/src/

for D in *; do
    if [ -d "${D}" ]; then
        cd ~/catkin_ws/src/${D}
        roswtf
    fi
done
