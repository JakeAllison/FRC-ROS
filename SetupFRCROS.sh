cwd=$PWD

echo "Currernt working directory:" $cwd

# Check that correct packages are installed.

ros_install=ros-kinetic-desktop-full
if dpkg-query -W -f'${Status}' "${ros_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "${ros_install} installed"
else
    echo -e "\e[31mROS Kinetic NOT installed. Please install '${ros_install}'\e[39m"
    exit 1
fi

pip_install=python-pip
if dpkg-query -W -f'${Status}' "${pip_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "${pip_install} installed"
else
    echo -e "\e[31mpython pip NOT installed. Please install '${pip_install}'\e[39m"
    exit 1
fi

#install network tables for ntbridge
python -m pip install --user pynetworktables==2018.2.0

# Check ~/.bashrc for sourcing

mkdir -p ~/catkin_ws/src

if grep -q 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc; then
    echo "~/.bashrc line found"
else
    echo -e "\e[93m~/.bashrc line NOT found. Adding line to EOF.\e[39m"
    echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
fi

# Copy packages

rm -rf ~/catkin_ws/src/frc_robot/
cp -R $cwd/catkin_ws/src/frc_robot ~/catkin_ws/src/

rm -rf ~/catkin_ws/src/networktable_bridge/
cp -R $cwd/catkin_ws/src/networktable_bridge ~/catkin_ws/src/

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
file=$cwd//hokuyo_04lx_laser.gazebo.xacro
if [ ! -f "$file" ]; then
    echo -e "\e[31m$file is missing in current working directory\e[39m"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/catkin_ws/src/common-sensors/common_sensors/urdf/sensors/"
    cp "$file" ~/catkin_ws/src/common-sensors/common_sensors/urdf/sensors/
fi

# Build everything

cd ~/catkin_ws
catkin_make
source ~/.bashrc
rospack list

# Check dependencies

cd ~/catkin_ws/src/frc_robot
roswtf

cd ~/catkin_ws/src/networktable_bridge
roswtf

echo "You may need to run 'source ~/.bashrc' or start a new terminal if roswtf cannot find 'frc_robot' package."
echo "You can run 'roswtf' in a terminal while you are running your ROS program to check if anything is faulty."
