#!/bin/bash

cwd=$PWD

echo "Currernt working directory:" $cwd



# Auto Install ROS
ros_install=ros-kinetic-desktop-full
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


if grep -q 'hid_sensor_custom' /etc/modules; then
    echo "hid_sensor_custom found"
else
    echo -e "\e[93mhid_sensor_custom NOT found. Adding line to EOF.\e[39m"
    echo 'hid_sensor_custom' | sudo tee -a /etc/modules
fi



# Install some stuff needed.
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential



# Install Realsense SDK and dependencies from source.
sudo apt install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt install libglfw3-dev

file=$cwd/librealsense-2.37.0.tar.gz
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "librealsense: '$file'"
    tar -xf "$file" -C ~/Downloads
fi

cd ~/Downloads/librealsense-2.37.0
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts.sh
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -BUILD_WITH_TM2=ON
sudo make uninstall && make clean && make -j$(nproc) && sudo make install



# Copy packages
cd $cwd/catkin_ws/src/
for D in *; do
    if [ -d "${D}" ]; then
        echo "Package: ${D}"
        rsync -r -t -v --progress --delete -u -c -s $cwd/catkin_ws/src/${D}/ ~/catkin_ws/src/${D}
    fi
done



# Copy supporting packages
cd $cwd/supporting_packages/
for F in *.tar.gz; do
    if [ -f "${F}" ]; then
	echo "Supporting Package: ${F}"
	tar -xf "${F}" -C ~/catkin_ws/src
    fi
done



# Gazebo models for the Kinect
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
catkin_make -j$(nproc)
source ~/catkin_ws/devel/setup.bash



# Check dependencies
cd ~/catkin_ws/src/
for D in *; do
    if [ -d "${D}" ]; then
        cd ~/catkin_ws/src/${D}
        roswtf
    fi
done
