# THE SENSOR FUSION WITH 4 ORB SLAM 2 AND GAZEBO 9
4 simulated stereo cameras are included for sensor redundancy. This program demands excessive pc resources but if you wish to use it then:
- Run the simulation
- Run the 4 ORB SLAM 2 algorithms
- Run the program AGV_SAVE_XYDATA.py in order to save data from your simulation that will be used for Neural Network training. drive your simulated vehicle all across the map for better results.
- Run NN_Y_4O.py to generate the Neural Network model of 4 inputs.

- Run NN_X_4O.py to generate the Neural Network model of 4 inputs.

- Run NN_ORIENTATION_4O.py to generate the Neural Network model for 4 inputs.

- Run the SENSOR_FUSION_4OS2.py program within your simulation and voila. Now you have a simulation of 4 ORB SLAM 2 combined via Neural Networks.
