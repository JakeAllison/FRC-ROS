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


# Copy necessary packages

cd $cwd/catkin_ws/src

# Copy necessary packages

for D in *; do
    if [ -d "${D}" ]; then
        echo "${D}"   # your processing here
        rsync -r -t -v --progress --delete -u -c -s $cwd/catkin_ws/src/${D}/ ~/catkin_ws/src/${D}
    fi
done

# Build everything

source /opt/ros/$ros_version/setup.bash
cd ~/catkin_ws
catkin_make -j$(nproc)
source ~/catkin_ws/devel/setup.bash
