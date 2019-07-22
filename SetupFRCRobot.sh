cwd=$PWD
echo "Currernt working directory:" $cwd

# Check that correct packages are installed.

ros_install=ros-kinetic-desktop-full
if dpkg-query -W -f'${Status}' "${ros_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "ROS Kinetic installed"
else
    echo "ROS Kinetic NOT installed. Please install '${ros_install}'"
    exit 1
fi

ros_install=ros-kinetic-gazebo-ros-control
if dpkg-query -W -f'${Status}' "${ros_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "ROS Gazebo Control installed"
else
    echo "ROS Fazebo Control NOT installed. Please install '${ros_install}'"
    exit 1
fi

ros_install=ros-kinetic-laser-filters
if dpkg-query -W -f'${Status}' "${ros_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "Laser Filters installed"
else
    echo "Laser Filters NOT installed. Please install '${ros_install}'"
    exit 1
fi

ros_install=ros-kinetic-openni-launch
if dpkg-query -W -f'${Status}' "${ros_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "OpenNI installed"
else
    echo "OpenNI NOT installed. Please install '${ros_install}'"
    exit 1
fi

ros_install=ros-kinetic-rtabmap
if dpkg-query -W -f'${Status}' "${ros_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "RTAB Map installed"
else
    echo "RTAB Map installed. Please install '${ros_install}'"
    exit 1
fi

ros_install=ros-kinetic-rtabmap-ros
if dpkg-query -W -f'${Status}' "${ros_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "RTAB Map ROS installed"
else
    echo "RTAB Map ROS installed. Please install '${ros_install}'"
    exit 1
fi

ros_install=ros-kinetic-hector-slam
if dpkg-query -W -f'${Status}' "${ros_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "Hector SLAM installed"
else
    echo "Hector SLAM installed. Please install '${ros_install}'"
    exit 1
fi

ros_install=ros-kinetic-hector-slam-launch
if dpkg-query -W -f'${Status}' "${ros_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "Hector SLAM Launch installed"
else
    echo "Hector SLAM Launch installed. Please install '${ros_install}'"
    exit 1
fi

ros_install=ros-kinetic-gazebo-plugins
if dpkg-query -W -f'${Status}' "${ros_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "Gazebo Plugins installed"
else
    echo "Gazebo Plugins NOT installed. Please install '${ros_install}'"
    exit 1
fi


ros_install=ros-kinetic-robot-localization
if dpkg-query -W -f'${Status}' "${ros_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "Robot Localization installed"
else
    echo "Robot Localization NOT installed. Please install '${ros_install}'"
  
    exit 1
fi

ros_install=ros-kinetic-move-base
if dpkg-query -W -f'${Status}' "${ros_install}" 2>/dev/null | grep -q "ok installed"; then
    echo "Move Base installed"
else
    echo "Move Base NOT installed. Please install '${ros_install}'"
  
    exit 1
fi
# Check ~/.bashrc for sourcing

mkdir -p ~/catkin_ws/src

if grep -q 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc; then
    echo "~/.bashrc line found"
else
    echo "~/.bashrc line NOT found. Adding line to EOF."
    echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
fi

# Copy necessary packages

file=$cwd/frc_robot.tar.gz
if [ ! -f "$file" ]; then
    echo "$file is missing in current working directory"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/catkin_ws/src"
    tar -xf "$file" -C ~/catkin_ws/src
fi

file=$cwd/common-sensors.tar.gz
if [ ! -f "$file" ]; then
    echo "$file is missing in current working directory"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/catkin_ws/src"
    tar -xf "$file" -C ~/catkin_ws/src
fi

file=$cwd/rplidar_ros.tar.gz
if [ ! -f "$file" ]; then
    echo "$file is missing in current working directory"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/catkin_ws/src"
    tar -xf "$file" -C ~/catkin_ws/src
fi


mkdir -p ~/.gazebo/models
file=$cwd/kinect_ros.tar.gz
if [ ! -f "$file" ]; then
    echo "$file is missing in current working directory"
    exit 1
else
    echo "Found '$file'"
    echo "Extracting to ~/.gazebo/models"
    tar -xf "$file" -C ~/.gazebo/models
fi

# Build everything

cd ~/catkin_ws
catkin_make
source ~/.bashrc
rospack list
cd ~/catkin_ws/src/frc_robot
roswtf
echo "You may need to run 'source ~/.bashrc' or start a new terminal if roswtf cannot find 'frc_robot' package."
echo "You can run 'roswtf' in a terminal while you are running your ROS program to check if anything is faulty."
