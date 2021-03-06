
# EXPERIMENTAL SOFTWARE FOR THE UGV LOCATED AT THE ITESM CAMPUS PUEBLA
Are you ready for experimenting with the real UGV, you should consider all the safety requirements before anything. Always verify the batteries voltage are over 48 volts, otherwise the vehicle will be stocked in the middle of nowhere. Arduino programs are located in the folder ARDUINO/ARDUINO_UGV_SERIAL. The arduino program ARDUINO_UGV_SERIAL.ino controls the UGV steer, speed and brakes. The arduino program ARDUINO_GPS_IMU2.ino reads the GPS and IMU sensors. Experiment and setup tutorial https://youtu.be/SxX4d-lKEHg

The presented results where obtained with the camera placed at 45 cm from the upper part of the vehicle, 44 from the button to the top of the aluminium structure and 1cm of the camera base.
![](arco_perfil.png)


0 - Run roscore

        source /opt/ros/melodic/setup.bash
        roscore
        
1 - First of all turn on the vehicle. NEVER PLUG USB CABLES TO YOUR LAPTOP BEFORE TURNING ON THE VEHICLE. it might cause damage to your USB.

2 - Once everything is on, plug in the 3 wires to your laptop (Xbox receiver, IMU-GPS Arduino, UGV Arduino).

3 - Wait around 15 seconds before running any program, the Arduinos take some time before your serial ports are available.

4 - Verify that the UGV Arduino is on serial port /dev/ttyACM1, otherwise you will have to change the AGV_INTERFACE_1.py program.

5 - Run the AGV_INTERFACE_1.py. If it breaks, try several times. Sometimes it does not work on the first try.

    python AGV_INTERFACE_1.py
6 - Verify that the GPS-IMU Arduino is on /dev/ttyACM0. This serial port can be easilly modified because it works with rosserial. Then run the serial node as indicated.
    
    cd ~/catkin_ws
    source devel/setup.bash
    rosrun rosserial_python serial_node.py /dev/ttyACM0
    
7 - Run the serial controller in order to communicate with the AGV_INTERFACE_1.py. You need to plug the Xbox controller first. Be careful, this controller usually unplugs and stops working!.

    python XBOX_CONTROLLER.py
![](xbox_controller.png)

8 - Run the button controller. This program orders the main odometry program what it has to do like: save a new route (new routes are saved as "saved route_1.csv"), load a new route (rigth now it loads the "saved_route_6.csv" this route starts at the emergency meeting point outside of CIDMA), follow a route (follows "saved_route_6.csv", in order to work initial conditions should be defined inside the program ODOMETRY_ALGORITHM_EXPERIMENTAL.py the variables are: PUNTO_DE_REUNION_X, PUNTO_DE_REUNION_Y, that by the fault the meeting point is (0,0) )

    python CONTROL_INTEGRADO.py
    
9 - Run the pure pursuit controller. 
    
    python PURE_PURSUIT_CONTROLLER.py
    
10 - Plug the ZED camera to your computer and run the ZED ROS node.

        cd ~/catkin_ws
        source devel/setup.bash
        roslaunch zed_wrapper zed.launch
        
    
10 - Run the main program which is 
    
    python ODOMETRY_ALGORITHM_EXPERIMENTAL.py
    
11 - Execute any task with the UGV and remember that security is first.

12 - If you finished following a path or traveling across any path the plot your odometry with the next python program. Inside ROUTE_FOLLOW_PLOT.py modify the file names of the odometry you want to plot and the route you saved.

        python ROUTE_FOLLOW_PLOT.py

