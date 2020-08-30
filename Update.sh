cwd=$PWD
echo "Currernt working directory:" $cwd


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

source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
catkin_make -j$(nproc)
source ~/catkin_ws/devel/setup.bash
