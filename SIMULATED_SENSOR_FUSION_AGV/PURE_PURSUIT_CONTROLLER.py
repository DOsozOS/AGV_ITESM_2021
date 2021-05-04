#!/usr/bin/env python
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import math
import rospy
import matplotlib.pyplot as plt
import numpy as np
import time
import rospy

PI = 3.14159265358979323846264338327950288
vehicle_l1=0.669670
ACTUAL_TIME = 0
WORKING_CMD = 0
vehicle_relative_angle = 0
delta_angle_point2point = 0
READY_DATA = 0
REQUIRED_ST = 0
ACTUAL_ST = 0
X_POS_KF = 0
Y_POS_KF = 0
ESTIMATED_YAW = 0
ROUTE_DATA_X = 0
ROUTE_DATA_Y = 0
ROUTE_DATA_YAW = 0
ROTARY_VELOCITY_GAZEBO = 0
controller_v = 0
controller_ST = 0
KH = 1
KV = 2
KI = 0.1
NEXT_ROUTE_POINT = 0.0
PATH_POINTS = 0.0
GLOBAL = globals()

def TIME_CALLBACK(message):
    GLOBAL['ACTUAL_TIME'] = message.data
def CMD_CALLBACK(message):
    GLOBAL['WORKING_CMD'] = message.data
def DATA_CALLBACK(message):
	GLOBAL['READY_DATA'] = 1
	GLOBAL['X_POS_KF'] = message.data[0]
	GLOBAL['Y_POS_KF'] = message.data[1]
	GLOBAL['ESTIMATED_YAW'] = message.data[2]
	GLOBAL['ROUTE_DATA_X'] = message.data[3]
	GLOBAL['ROUTE_DATA_Y'] = message.data[4]
	GLOBAL['ROUTE_DATA_YAW'] = message.data[5]
	GLOBAL['ROTARY_VELOCITY_GAZEBO'] = message.data[6]
	GLOBAL['NEXT_ROUTE_POINT'] = message.data[7]
	GLOBAL['PATH_POINTS'] = message.data[8]
def angle_delta (yaw_1,yaw_2):
    yaw_a = positive_angles(yaw_1)
    yaw_f = positive_angles(yaw_2)

    delta_cw = positive_angles(yaw_a-yaw_f)
    delta_icw = positive_angles(yaw_f-yaw_a)

    if(delta_cw<delta_icw):
        return -1*delta_cw
    else:
        return  delta_icw
def transform_coordinates(xa,ya,xn,yn,thetaa):
    #displace coordinates
    xt = xn-xa
    yt = yn-ya
    #rotate coordinates
    xr = xt*math.cos(thetaa) + yt*math.sin(thetaa)
    yr = -xt*math.sin(thetaa) + yt*math.cos(thetaa)
    return np.array([xr,yr])
def positive_angles(angle):
    while(angle < 0):
        angle = angle + 2.0*PI
    while(angle > 2*PI):
        angle = angle - 2.0*PI
    return angle
def steering_bound(st):
    return max(-0.35,min(0.35,st))
def speed_bound(speed):
    return max(-3,min(3,speed))
def MAIN():
	rospy.init_node('agv_low_level_control', anonymous=1)
	rospy.Subscriber("/my_agv_controller/working_cmd", Int32, CMD_CALLBACK)
	rospy.Subscriber("/my_agv_controller/time", Float32, TIME_CALLBACK)
	rospy.Subscriber("/my_agv/controller_data",Float32MultiArray,DATA_CALLBACK)
	pubST = rospy.Publisher('/my_agv/steering_cmd', Float32, queue_size=1)
	pubSp = rospy.Publisher('/my_agv/rear_speed_cmd', Float32, queue_size=1)
	rate = rospy.Rate(100)
	past_time = 0
	eintegral = 0
	prevtime = 0
	deltatime = 0
	firstime = True
	finished = False
	tfc_x = 0.0
	tfc_y = 0.0
	while not (rospy.is_shutdown()):
		if(GLOBAL['READY_DATA'] == 1 and firstime==True):
			prevtime = GLOBAL['ACTUAL_TIME']
			firstime = False
			print("First time done")

		if(GLOBAL['READY_DATA'] == 1 and GLOBAL['WORKING_CMD'] == 6 and firstime == False):
			if(prevtime != 0):
				GLOBAL['READY_DATA'] == 0
				deltatime = GLOBAL['ACTUAL_TIME']-prevtime
				tfc = transform_coordinates(GLOBAL['X_POS_KF'],GLOBAL['Y_POS_KF'],GLOBAL['ROUTE_DATA_X'],GLOBAL['ROUTE_DATA_Y'],GLOBAL['ESTIMATED_YAW'])
				tfc_x = tfc[0]
				tfc_y = tfc[1]
				desired_theta = math.atan2(tfc_y,tfc_x)
				dv = 0.0
				
				e = math.sqrt(math.pow(GLOBAL['ROUTE_DATA_X']-GLOBAL['X_POS_KF'],2)+math.pow(GLOBAL['ROUTE_DATA_Y']-GLOBAL['Y_POS_KF'],2))
				if(e<0.2 and GLOBAL['NEXT_ROUTE_POINT']== GLOBAL['PATH_POINTS']-1):
					finished = True
					print("TIME TO STOP")
				
				if(finished==False):
					eintegral = e*deltatime
					GLOBAL['controller_ST'] = steering_bound(KH *desired_theta)
					dv = speed_bound(KV*e + KI*eintegral)

				print("--------------------------------------------")
				print("desired speed "+str(dv))
				print("desired st "+str(GLOBAL['controller_ST']))
				print("NEXT POINT: X:"+str(GLOBAL['ROUTE_DATA_X'])+" Y:"+str(GLOBAL['ROUTE_DATA_Y']))

				e = math.sqrt(math.pow(GLOBAL['ROUTE_DATA_X']-GLOBAL['X_POS_KF'],2)+math.pow(GLOBAL['ROUTE_DATA_Y']-GLOBAL['Y_POS_KF'],2))
				eintegral = e*deltatime
				
				
				if(finished==False):
					dv = speed_bound(KV*e + KI*eintegral)
					pubSp.publish(dv) 
					pubST.publish(GLOBAL['controller_ST'])
				else:
					pubST.publish(0)
					pubSp.publish(0)
			prevtime = GLOBAL['ACTUAL_TIME']
		
		time.sleep(0.1)

if __name__ == '__main__':
	try:
		MAIN()
	except (KeyboardInterrupt,rospy.ROSInterruptException):
		print("\nEnding execution")
