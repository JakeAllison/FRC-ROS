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
