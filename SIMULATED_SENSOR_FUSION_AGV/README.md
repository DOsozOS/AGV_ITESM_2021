# SIMULATION PATH SAVING, TRACKING AND ODOMETRY ON GAZEBO 9
This simulation is based on the real UGV located on the ITESM campus Puebla

1 - Run the roscore

	echo "source /opt/ros/melodic/setup.sh" >> ~/.bashrc
	source /opt/ros/melodic/setup.bash
	roscore
2 - Run the Gazebo 9 simulation

	source /opt/ros/melodic/setup.bash
	cd ~/agv_plugin/build
	export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/agv_plugin/build
	export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/diegoosorio/model_editor_models
	 source ~/.bashrc
	rosrun gazebo_ros gazebo ../agv6.world  paused:=true
3 - Run the ORB SLAM 2 algorithm
	Be sure that you use the provided maps. If you map your location, be sure everything is on the same reference frame!

	export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/diegoosorio/ORB_SLAM2/Examples/ROS
	cd ~/ORB_SLAM2/Examples/ROS/ORB_SLAM2
	rosrun ORB_SLAM2 Stereo1 /home/diegoosorio/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/diegoosorio/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Asus1.yaml false /home/diegoosorio/ORB_SLAM2/Examples/ROS/ORB_SLAM2/src/maps true
4 - Run the pure pursuit controller (in the SIMULATED_SENSOR_FUSION_AGV directory)

	python PURE_PURSUIT_CONTROLLER.py
5 - Run the button controller (in the SIMULATED_SENSOR_FUSION_AGV directory)

	python CONTROL_INTEGRADO.py
5 - Run the odometry algorithm which is the main program of this simulation (in the SIMULATED_SENSOR_FUSION_AGV directory)

	python ODOMETRY_ALGORITHM.py
6 - Once done, run the ROUTE_FOLLOW_PLOT.py in order to get a visual file of what actually happened.

	python ROUTE_FOLLOW_PLOT.py
	
Every simulation result will be saved inside the simulation files folder

You can download the map files from mega: https://mega.nz/folder/U4BTSCTR#Pz4S0KjTeyhFtC5opw87_Q
Use the maps located in the Path Following folder in Mega
Do not hesitate to contact me in case of any doubt: diegoosorio0000@gmail.com
