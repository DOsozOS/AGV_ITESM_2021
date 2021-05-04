# catkin_ws
# export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/${user}/Documents/king/ORB_SLAM2_MAP/Examples/ROS

echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j4
