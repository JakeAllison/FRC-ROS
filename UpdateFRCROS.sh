cwd=$PWD
echo "Currernt working directory:" $cwd


# Copy necessary packages

rm -rf ~/catkin_ws/src/frc_robot/
cp -R $cwd/catkin_ws/src/frc_robot ~/catkin_ws/src/

rm -rf ~/catkin_ws/src/networktable_bridge/
cp -R $cwd/catkin_ws/src/networktable_bridge ~/catkin_ws/src

rm -rf ~/catkin_ws/src/swerve_controller/
cp -R $cwd/catkin_ws/src/swerve_controller ~/catkin_ws/src/

# Build everything

source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
