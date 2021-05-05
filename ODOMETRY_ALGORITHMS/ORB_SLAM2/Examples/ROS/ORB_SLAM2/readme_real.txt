
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/diego/ORB_SLAM2/Examples/ROS
cd ~/ORB_SLAM2/Examples/ROS/ORB_SLAM2
rosrun ORB_SLAM2 Stereo1 /home/diego/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/diego/ORB_SLAM2/Examples/ROS/ORB_SLAM2/StereoCameraCalib.yaml true /home/diego/ORB_SLAM2/Examples/ROS/ORB_SLAM2/src/maps true
