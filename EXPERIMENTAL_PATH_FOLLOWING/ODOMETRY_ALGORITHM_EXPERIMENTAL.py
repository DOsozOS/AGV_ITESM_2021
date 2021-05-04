#!/usr/bin/env python
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from random import seed
from random import gauss
from std_msgs.msg import String
from random import seed
from random import gauss
from vehicle_model import vehicle_model
from vehicle_model import vehicle_model

import math
import rospy
import matplotlib.pyplot as plt
import numpy as np
import random
import time
import rospy
import statistics
import csv
import datetime
x = datetime.datetime.now()

viewing_horizon = 3

filename = "PATH_FOLLOW_TEST_"+str(x.date())+"_"+str(x.time())+"_REAL_H"+str(viewing_horizon)+".csv"
#filename = "YAW_CORRECTION_GPSKF_02_DM.csv"
PI = 3.14159265358979323846264338327950288
READY_FLAG_FRONT_ZED = 0
READY_FLAG_REAR_OS2 = 0
READY_FLAG_IMU = 0
READY_FLAG_IMU_YAW = 0
ReadyFlag_lidar = 0
SteerAngle = 0
file2 = open("saved_route_1.csv","w")
file1 = open(filename,"w")
counter = 4
#IMU Data
XACC_IMU = 0.0
YACC_IMU = 0.0
ZACC_IMU = 0.0
XANG_SPEED_IMU = 0.0
YANG_SPEED_IMU = 0.0
ZANG_SPEED_IMU = 0.0

EXECUTION_TIME = 0.0
PREV_EXECUTION_TIME = 0.0
RECORD_TIME = 0.0
YAW_Z_IMU = 0.0 #psi
PITCH_Y_IMU = 0.0#theta
BANK_X_IMU  = 0.0#phi
YAW_IMU=0.0
#GPS position estimate
Y_COORDINATE_GPS_PREV = 0.0
X_COORDINATE_GPS_PREV = 0.0
Y_COORDINATE_GPS   = 0.0
X_COORDINATE_GPS   = 0.0
GPS_UPDATE_TIME = 0.0
GPS_DELTA = 0.0
#ADJUST DEPENDING ON THE CLIMATE AND NOISE
GPS_RESET_DELTA = 500
GPS_ADJUSTED_DELTA_TH = 500
GPS_DELTA_TH = 10
#COORDINATES OF YOUR REFERENCE 0,0 
LATITUDE_OFFSET = 19.017241
LONGITUD_OFFSET  = -98.242252

PUNTO_DE_REUNION_X = 0
PUNTO_DE_REUNION_Y = 0

READY_FLAG_GPS_Y = 0
READY_FLAG_GPS_X = 0
GPS_YAW = 0.0
IMU_GPS_OFFSET = 0.0
#ORB SLAM 2 position estimate 1
X_FRONT_ZED = 0.0
Y_FRONT_ZED = 0.0
X_FRONT_ZED_TEMP = 0.0
Y_FRONT_ZED_TEMP = 0.0
X_FRONT_ZED_PAST = 0.0
Y_FRONT_ZED_PAST = 0.0
X_FRONT_ZED_RAW = 0.0
Y_FRONT_ZED_RAW = 0.0
X_FRONT_ZED_RAW_2 = 0.0
Y_FRONT_ZED_RAW_2 = 0.0
ROTATION_DISPLACEMENT_ZED_IMU = 0.0
X_FRONT_ZED_RAW = 0.0
Y_FRONT_ZED_RAW = 0.0
YAW_Z_ZED = 0.0
PITCH_Y_ZED = 0.0
BANK_X_ZED = 0.0
BANK_X_ZED_RAW = 0.0
X_FRONT_ZED_C = 0.0
Y_FRONT_ZED_C = 0.0
ZED_OFFSET_X = 0
ZED_OFFSET_Y = 0
X_OFFSET_ZED = 0
Y_OFFSET_ZED = 0
ZQW = 0.0
ZQX = 0.0
ZQY = 0.0
ZQZ = 0.0
DELTA_ZED_IMU_YAW = 0
DELTA_ZED_IMU_YAW_C = 0
FRONT_QUALITY_OS2 = 0
ZED_OFFSET_NOW = False
#ORB SLAM 2 position estimate 2
X_REAR_OS2 = 0.0
Y_REAR_OS2 = 0.0
YAW_REAR_OS2 = 0.0
REAR_QUALITY_OS2 = 0
#Real Position
velocity_sensor = 0
SPEED_ESTIMATE = 0
SPEED_ESTIMATE_PAST = 0
#MODEL variables
STEER_ANGLE_AGV = 0
FILTERED_STEER_ANGLE_AGV = 0
FILTERED_SPEED = 0
#KALMAN FILTER VARIABLES--------------------------------------------------------------------------------------------------
FIRST_TIME = False
#Initial conditions
#    pass
FAILED_CALCULATION_TIMES = 0
X0_KF = 0.0 #Theta
X1_KF = 0.0 #Omega
Y_VEL_KF = 0.0 #Lateral velocity vy
X3 = 0.0 #Longitudinal velocity vx
X_POS_KF = 0 #Global x position
Y_POS_KF =  0#Global y position
ESTIMATED_YAW = 0
X_POS_KF_PAST = 0.0
Y_POS_KF_PAST = 0.0
STEER_FLAG = False
SPEED_FLAG = False
FYF_TEMP = 0.0
FYR_TEMP = 0.0
vm = vehicle_model(vehicle_mass=400.0,vehicle_l1=0.669670,vehicle_l2=0.669670,
            vehicle_width=1.16,height_iz=0.38,vehicle_wheel_radius=0.2876,
            vehicle_initial_theta=0.0,vehicle_initial_vx=0.0,vehicle_initial_vy=0.0,
            vehicle_initial_x=0.0,vehicle_initial_y=0.0)

#Saving route for the vehile trajectory
WORKING_CMD = 0
X_DESIRED_PREV = 0
Y_DESIRED_PREV = 0
PATH_POINTS = 0
ROUTE_DATA = np.empty([2,2])
NEXT_ROUTE_POINT = 1
ZED_DELTA_KF = 0
WHAT_HAPPEN = ""
DELTA_BEGIN = 0.0
ONCE_CORRECTED_ZED = False

X_CALIB_GPS = 0.0
Y_CALIB_GPS = 0.0


GLOBAL = globals()


def transform_coordinates(xa,ya,xn,yn,thetaa):
    #displace coordinates
    xt = xn-xa
    yt = yn-ya
    #rotate coordinates
    xr = xt*math.cos(thetaa) + yt*math.sin(thetaa)
    yr = -xt*math.sin(thetaa) + yt*math.cos(thetaa)
    return np.array([xr,yr])
def rotate_coordinates (xa,ya,thetaa):
    #rotate coordinates
    if(thetaa!=0.0):
        xr = xa*math.cos(thetaa) + ya*math.sin(thetaa)
        yr = -xa*math.sin(thetaa) + ya*math.cos(thetaa)
        return np.array([xr,yr])
    else:
        return np.array([xa,ya])
def offset_calculation(xz,yz,xgps,ygps):
    GLOBAL['ZED_OFFSET_X'] = xgps-xz
    GLOBAL['ZED_OFFSET_Y'] = ygps-yz
def displace_coordinates_zed():
    GLOBAL['X_FRONT_ZED'] = GLOBAL['X_FRONT_ZED'] + GLOBAL['ZED_OFFSET_X']
    GLOBAL['Y_FRONT_ZED'] = GLOBAL['Y_FRONT_ZED'] + GLOBAL['ZED_OFFSET_Y']
def displace_coordinates(xa,ya,xn,yn):
    xr = xn-xa
    yr = yn-ya
    return np.array([xr,yr])

def positive_angles(angle):
    while(angle < 0):
        angle = angle + 2.0*PI
    while(angle > 2*PI):
        angle = angle - 2.0*PI
    return angle
def ANGLE_DELTA (yaw_1,yaw_2):
    yaw_a = positive_angles(yaw_1)
    yaw_f = positive_angles(yaw_2)

    delta_cw = positive_angles(yaw_a-yaw_f)
    delta_icw = positive_angles(yaw_f-yaw_a)

    if(delta_cw<delta_icw):
        return -1*delta_cw
    else:
        return  delta_icw


def ANGLE_AVERAGE(yaw_1,yaw_2):
    yaw_1 = positive_angles(yaw_1)
    yaw_2 = positive_angles(yaw_2)
    x1 = math.cos(yaw_1)
    y1 = math.sin(yaw_1)
    x2 = math.cos(yaw_2)
    y2 = math.sin(yaw_2)
    xav = (x1+x2)/2
    yav = (y1+y2)/2
    #primer cuadrante
    if(xav>0 and yav>0):
        return(math.atan(abs(yav)/abs(xav)))
    #segundo cuadrante
    elif(xav<0 and yav>0):
        return( PI-1*math.atan(abs(yav)/abs(xav)))
    #tercer cuadrante
    elif(xav<0 and yav<0):
        return(math.atan(abs(yav)/abs(xav)) + PI)
    #cuarto cuadrante
    elif(xav>0 and yav<0):
        return(-1*math.atan(abs(yav)/abs(xav)) + 2*PI)
    #casos especiales
    elif(xav>0 and yav==0):
        return 0
    elif(xav==0 and yav>0):
        return PI/2
    elif(xav<0 and yav==0):
        return PI
    elif(xav==0 and yav<0):
        return PI*1.5
    else:
        return 0 

def IMU_CALLBACK(message):
    GLOBAL['READY_FLAG_IMU'] = 1
    #GLOBAL['XACC_IMU'] = message.linear_acceleration.x
    GLOBAL['YACC_IMU'] = message.linear_acceleration.y
    GLOBAL['ZACC_IMU'] = message.linear_acceleration.z
    GLOBAL['XANG_SPEED_IMU'] = message.angular_velocity.x
    GLOBAL['YANG_SPEED_IMU'] = message.angular_velocity.y
    GLOBAL['ZANG_SPEED_IMU'] = message.angular_velocity.z
    q0 = message.orientation.w
    q1 = message.orientation.x
    q2 = message.orientation.y
    q3 = message.orientation.z
    GLOBAL['YAW_Z_IMU'] = positive_angles(math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2)))
    GLOBAL['PITCH_Y_IMU'] = positive_angles(math.asin(2*(q0*q2-q3*q1)))
    #GLOBAL['BANK_X_IMU'] = positive_angles(math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3)) -math.radians(10))#LAST OFFSET DUE TO INITIAL CALIBRATION
def OS_2_POSE_CALLBACK(message):
    GLOBAL['READY_FLAG_REAR_OS2'] = 1
    GLOBAL['X_REAR_OS2'] = message.pose.position.x 
    GLOBAL['Y_REAR_OS2'] = message.pose.position.y  
    GLOBAL['X_REAR_OS2'] = round(GLOBAL['X_REAR_OS2'], 4)
    GLOBAL['Y_REAR_OS2'] = round(GLOBAL['Y_REAR_OS2'], 4)
    GLOBAL['YAW_REAR_OS2'] = -1*(message.pose.orientation.y)*PI/180.0
def OS_2_QUALITY_CALLBACK(message):
    GLOBAL['REAR_QUALITY_OS2'] = message.data
def ZED_POSE_CALLBACK(message):
    GLOBAL['READY_FLAG_FRONT_ZED'] = 1
    GLOBAL['X_FRONT_ZED_RAW_2'] = message.pose.position.x
    GLOBAL['Y_FRONT_ZED_RAW_2'] = message.pose.position.y
    ret = rotate_coordinates(message.pose.position.x,message.pose.position.y,GLOBAL['ROTATION_DISPLACEMENT_ZED_IMU'])
    GLOBAL['X_FRONT_ZED_RAW'] = ret[0]
    GLOBAL['Y_FRONT_ZED_RAW'] = ret[1]
    if(GLOBAL['ZED_OFFSET_NOW']):
        GLOBAL['ZED_OFFSET_NOW'] = False
        GLOBAL['X_OFFSET_ZED'] = GLOBAL['X_FRONT_ZED_RAW']-GLOBAL['X_POS_KF']
        GLOBAL['Y_OFFSET_ZED'] = GLOBAL['Y_FRONT_ZED_RAW']-GLOBAL['Y_POS_KF']
    
    GLOBAL['X_FRONT_ZED'] = GLOBAL['X_FRONT_ZED_RAW'] - GLOBAL['X_OFFSET_ZED']
    GLOBAL['Y_FRONT_ZED'] = GLOBAL['Y_FRONT_ZED_RAW'] - GLOBAL['Y_OFFSET_ZED']
    GLOBAL['ZQW'] = message.pose.orientation.w
    GLOBAL['ZQX'] = message.pose.orientation.x
    GLOBAL['ZQY'] = message.pose.orientation.y
    GLOBAL['ZQZ'] = message.pose.orientation.z

    q0 = message.pose.orientation.w
    q1 = message.pose.orientation.x
    q2 = message.pose.orientation.y
    q3 = message.pose.orientation.z

    GLOBAL['YAW_Z_ZED'] = positive_angles(math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2)))
    GLOBAL['PITCH_Y_ZED'] = positive_angles(math.asin(2*(q0*q2-q3*q1)))
    GLOBAL['BANK_X_ZED'] = positive_angles(math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3)))

def ZED_YAW_CALLBACK(message):
    GLOBAL['BANK_X_ZED'] = message.data
def OS_1_QUALITY_CALLBACK(message):
    GLOBAL['FRONT_QUALITY_OS2'] = message.data
def STEERING_CALLBACK(message):
    GLOBAL['STEER_FLAG'] = True
    GLOBAL['STEER_ANGLE_AGV'] = (message.data/57.2958)

def SPEED_CALLBACK(message):
    GLOBAL['SPEED_FLAG'] = True
    GLOBAL['velocity_sensor'] = message.data

def LAT_CALLBACK(message):
    GLOBAL['READY_FLAG_GPS_Y'] = True
    GLOBAL['Y_COORDINATE_GPS'] = (message.data- GLOBAL['LATITUDE_OFFSET'])*111000 + GLOBAL['Y_CALIB_GPS']
def LON_CALLBACK(message):
    GLOBAL['READY_FLAG_GPS_X'] = True
    GLOBAL['X_COORDINATE_GPS'] = (message.data- GLOBAL['LONGITUD_OFFSET'])*111000 + GLOBAL['X_CALIB_GPS']
def CMD_CALLBACK(message):
    GLOBAL['WORKING_CMD'] = message.data
    if(GLOBAL['WORKING_CMD'] == 2):
        print("READY FOR SAVING A NEW ROUTE")
def bound(low, high, value):
    return max(low, min(high, value))
def RECORD():
    if (GLOBAL['EXECUTION_TIME'] - GLOBAL['RECORD_TIME']>0.05):
        file1.write(str(GLOBAL['EXECUTION_TIME'])+","+#0 A
        str(GLOBAL['X_POS_KF'])+","+#1 B
        str(GLOBAL['Y_POS_KF'])+","+#2 C
        str(GLOBAL['X_COORDINATE_GPS'])+","+#3 D
        str(GLOBAL['Y_COORDINATE_GPS'])+","+#4 E
        str(GLOBAL['YAW_IMU'])+","+#5 F
        str(GLOBAL['ESTIMATED_YAW'])+","+#6 G
        str(GLOBAL['velocity_sensor'])+","+#7 H
        str(GLOBAL['STEER_ANGLE_AGV']*57.2958)+","+#8 I
        str(GLOBAL['READY_FLAG_GPS_Y'])+","+#9 J
        str(GLOBAL['READY_FLAG_GPS_X'])+","+#10 K
        str(GLOBAL['WORKING_CMD'])+","+#11 L
        str(GLOBAL['XACC_IMU'])+","+#12 M
        str(GLOBAL['GPS_YAW'])+","+#13 N
        str(GLOBAL['X_FRONT_ZED'])+","+#14 O
        str(GLOBAL['Y_FRONT_ZED'])+","+#15 P
        str(GLOBAL['IMU_GPS_OFFSET'])+","+#16 Q
        str(GLOBAL['GPS_DELTA'])+","+ #17 R
        GLOBAL['WHAT_HAPPEN']+","+ #18 S
        str(GLOBAL['BANK_X_ZED'])+","+#19 T
        str(GLOBAL['ROTATION_DISPLACEMENT_ZED_IMU'])+","+ #20 U
        str(GLOBAL['X_OFFSET_ZED'])+","+#21 V
        str(GLOBAL['Y_OFFSET_ZED'])+","+#22 W
        str(GLOBAL['X_REAR_OS2'])+","+#23 X
        str(GLOBAL['Y_REAR_OS2'])+","+# 24 Y
        str(GLOBAL['YAW_REAR_OS2'])+","+# 25 Z
        str(GLOBAL['X_FRONT_ZED_RAW_2'])+","+# 26 AA
        str(GLOBAL['Y_FRONT_ZED_RAW_2'])+","+# 27 AB
        str(GLOBAL['YAW_Z_IMU']) +","+ #28 AC
        str(GLOBAL['PITCH_Y_IMU']) +","+ #29 AD
        str(GLOBAL['SPEED_ESTIMATE'])+","+# 23 AE
        str(GLOBAL['Y_VEL_KF'])+","+
        str(GLOBAL['FYR_TEMP'])+","+
        str(GLOBAL['FYF_TEMP'])+","+
        str(GLOBAL['FILTERED_STEER_ANGLE_AGV']*57.2958)+","+
        str(GLOBAL['FILTERED_SPEED'])+","+
        str(GLOBAL['ZED_DELTA_KF'])+","+
        "\n")#11
        PRINT()
def RECORD_SIMULATION():
    file1.write(str(GLOBAL['EXECUTION_TIME'])+","+#0 A
        str(GLOBAL['X_POS_KF'])+","+#1 B
        str(GLOBAL['Y_POS_KF'])+","+#2 C
        str(GLOBAL['X_COORDINATE_GPS'])+","+#3 D
        str(GLOBAL['Y_COORDINATE_GPS'])+","+#4 E
        str(GLOBAL['YAW_IMU'])+","+#5 F
        str(GLOBAL['ESTIMATED_YAW'])+","+#6 G
        str(GLOBAL['velocity_sensor'])+","+#7 H
        str(GLOBAL['STEER_ANGLE_AGV']*57.2958)+","+#8 I
        str(GLOBAL['READY_FLAG_GPS_Y'])+","+#9 J
        str(GLOBAL['READY_FLAG_GPS_X'])+","+#10 K
        str(GLOBAL['WORKING_CMD'])+","+#11 L
        str(GLOBAL['XACC_IMU'])+","+#12 M
        str(GLOBAL['GPS_YAW'])+","+#13 N
        str(GLOBAL['X_FRONT_ZED'])+","+#14 O
        str(GLOBAL['Y_FRONT_ZED'])+","+#15 P
        str(GLOBAL['IMU_GPS_OFFSET'])+","+#16 Q
        str(GLOBAL['GPS_DELTA'])+","+ #17 R
        GLOBAL['WHAT_HAPPEN']+","+ #18 S
        str(GLOBAL['BANK_X_ZED'])+","+#19 T
        str(GLOBAL['ROTATION_DISPLACEMENT_ZED_IMU'])+","+ #20 U
        str(GLOBAL['X_OFFSET_ZED'])+","+#21 V
        str(GLOBAL['Y_OFFSET_ZED'])+","+#22 W
        str(GLOBAL['X_REAR_OS2'])+","+#23 X
        str(GLOBAL['Y_REAR_OS2'])+","+# 24 Y
        str(GLOBAL['YAW_REAR_OS2'])+","+# 25 Z
        str(GLOBAL['X_FRONT_ZED_RAW_2'])+","+# 26 AA
        str(GLOBAL['Y_FRONT_ZED_RAW_2'])+","+# 27 AB
        str(GLOBAL['YAW_Z_IMU']) +","+ #28 AC
        str(GLOBAL['PITCH_Y_IMU']) +","+ #29 AD
        str(GLOBAL['SPEED_ESTIMATE'])+","+# 23 AE
        str(GLOBAL['Y_VEL_KF'])+","+
        str(GLOBAL['FYR_TEMP'])+","+
        str(GLOBAL['FYF_TEMP'])+","+
        str(GLOBAL['FILTERED_STEER_ANGLE_AGV']*57.2958)+","+
        str(GLOBAL['FILTERED_SPEED'])+","+
        "\n")#11
    PRINT()
        
def PRINT():
    print("--------------------------------------")
    print("Position = "+ " X:"+str(round(GLOBAL['X_POS_KF'],2))+
    " Y:"+str(round(GLOBAL['Y_POS_KF'],2)) + "\ntheta: "+str(math.degrees(GLOBAL['YAW_IMU'])))
    print("Speed:"),
    print(GLOBAL['FILTERED_SPEED']),
    print(" Steer:"),
    print(GLOBAL['FILTERED_STEER_ANGLE_AGV']),
    print("Acc:"),
    print(str(GLOBAL['XACC_IMU']))
    print("Execution time: "+str(GLOBAL['EXECUTION_TIME']))
    GLOBAL['RECORD_TIME'] = GLOBAL['EXECUTION_TIME']
    print("Next point: " +str(GLOBAL['NEXT_ROUTE_POINT']))
def YAW_CALLBACK(message):
    GLOBAL['READY_FLAG_IMU_YAW'] = 1
    GLOBAL['YAW_IMU'] = positive_angles(-1*(message.data+90-13)*0.0174533  -math.radians(-20))
def MAIN():
    rospy.init_node('agv_control', anonymous=1)
    pubControllerData = rospy.Publisher('/my_agv/controller_data', Float32MultiArray, queue_size=1)
    rospy.Subscriber("/zed/zed_node/pose", PoseStamped, ZED_POSE_CALLBACK)
    rospy.Subscriber("/ORB_SLAM2_1/quality", Int32, OS_1_QUALITY_CALLBACK)
    rospy.Subscriber("/my_agv_controller/working_cmd", Int32, CMD_CALLBACK)
    pubCMD = rospy.Publisher('/my_agv_controller/working_cmd', Int32, queue_size=1)
    pubTIME = rospy.Publisher('/my_agv_controller/time', Float32, queue_size=1)
    pubCCMD = rospy.Publisher('/my_agv_controller/working_cmd_automatic', Int32, queue_size=1)
    rospy.Subscriber("/ORB_SLAM2_3/pose", PoseStamped, OS_2_POSE_CALLBACK)
    rospy.Subscriber("/ORB_SLAM2_3/quality", Int32, OS_2_QUALITY_CALLBACK)
    rospy.Subscriber("/my_agv/imu", Imu, IMU_CALLBACK)
    rospy.Subscriber("/steering_sensor",Float32,STEERING_CALLBACK)
    rospy.Subscriber("/rear_speed_sensor",Float32,SPEED_CALLBACK)
    rospy.Subscriber("/my_agv/latitude",Float32,LAT_CALLBACK)
    rospy.Subscriber("/my_agv/longitude",Float32,LON_CALLBACK)
    rospy.Subscriber("/my_agv/yaw",Float32,YAW_CALLBACK)
    rospy.Subscriber("/my_agv/zed_yaw",Float32,ZED_YAW_CALLBACK)
    rate = rospy.Rate(100) 
    print("Welcome to the AGV System")
    #Customize your ackerman vehicle (refer to the book introduction to ground vehicles)
    GLOBAL['vm'] = vehicle_model(vehicle_mass=400.0,vehicle_l1=0.669670,vehicle_l2=0.669670,
            vehicle_width=1.16,height_iz=0.38,vehicle_wheel_radius=0.2876,
            vehicle_initial_theta=0.0,vehicle_initial_vx=0.0,vehicle_initial_vy=0.0,
            vehicle_initial_x=0.0,vehicle_initial_y=0.0)

    actual_x_gps = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    actual_y_gps = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    moving_x_gps = [0,0,0]
    moving_y_gps = [0,0,0]

    steer_angle =        [0,0,0,0,0,0,0,0,0,0,0,0]
    longitudinal_speed = [0,0,0,0,0,0,0,0,0,0,0,0]
    moving_gps_count = 0

    count = 0
    count_gps_points = 0
    saved_points = 0
    time_offset = time.time()
    time_rotation_calculation = 0
    time_zed_rotation = 0
    zed_orientation_correction_time = 0
    zed_position_correction_time = 0
    first_time_zed_correction = True
    acceleration_time = 0
    dm_first_time = False
    first_gps_correction=False
    time_free_before_gps_correction = False

    time_dm = 0
    xa = 0
    xp = 0
    ya = 0
    yp = 0
    while not (rospy.is_shutdown() ):
        GLOBAL['EXECUTION_TIME'] = time.time()-time_offset       
        RECORD()
        #PRINT()
        #RECORD_SIMULATION()
        pubTIME.publish(GLOBAL['EXECUTION_TIME'])
        #LETS FILTER STEER DATA WITH A SIMPLE AVERAGE FILTER
        if(GLOBAL['STEER_FLAG']):
            GLOBAL['STEER_FLAG'] = False
            steer_angle.append(GLOBAL['STEER_ANGLE_AGV'])
            steer_angle.pop(0)
            GLOBAL['FILTERED_STEER_ANGLE_AGV'] = statistics.mean(steer_angle)

        if(GLOBAL['SPEED_FLAG']):
            GLOBAL['SPEED_FLAG'] = False
            longitudinal_speed.append(GLOBAL['velocity_sensor'])
            longitudinal_speed.pop(0)
            GLOBAL['FILTERED_SPEED'] = statistics.mean(longitudinal_speed)

        if(GLOBAL['EXECUTION_TIME']-acceleration_time>0.3):
            acceleration_time = GLOBAL['EXECUTION_TIME']
            #GLOBAL['XACC_IMU'] = (GLOBAL['velocity_sensor'] - GLOBAL['SPEED_ESTIMATE_PAST'])/0.3
            GLOBAL['SPEED_ESTIMATE_PAST'] = GLOBAL['velocity_sensor']

        

        elif(GLOBAL['FILTERED_STEER_ANGLE_AGV']>0.10):
            time_zed_rotation = GLOBAL['EXECUTION_TIME']

        if ( GLOBAL['WORKING_CMD'] == 5):
            pubCCMD.publish(0)
            break
        #Load route
        if(GLOBAL['WORKING_CMD'] == 7):
            GLOBAL['WORKING_CMD'] = 0
            Data = []
            line_count = 0
            with open('saved_route_6.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for row in csv_reader:
                    floatRow = [float(row[0]),
                    float(row[1]),
                    float(row[2])]
                    Data.append(floatRow)
                    line_count += 1
            
            GLOBAL['PATH_POINTS']=line_count
            print("Total trajectory points " + str(GLOBAL['PATH_POINTS']))
            GLOBAL['ROUTE_DATA'] = np.array(Data)

        if(GLOBAL['WORKING_CMD'] == 4):
            #INITIAL GPS, YAW AND ZED ORIENTATION
            actual_x_average = statistics.mean(actual_x_gps)
            actual_y_average = statistics.mean(actual_y_gps)
            #WE MUST ASSUME THAT THE BEGINING POINT IS PREDEFINED IN ORDER TO CALIBRATE THE GPS OFFSET JUST AS
            #THE ONE WHEN THE ROUTE WAS SAVED (GPS OFFSET MIGHT VARY WITHIN 1 HOUR!)
            GLOBAL['X_CALIB_GPS'] = PUNTO_DE_REUNION_X - actual_x_average#GLOBAL['ROUTE_DATA'][0,0] - actual_x_average
            GLOBAL['Y_CALIB_GPS'] = PUNTO_DE_REUNION_Y - actual_y_average#GLOBAL['ROUTE_DATA'][0,1] - actual_y_average

            GLOBAL['ROTATION_DISPLACEMENT_ZED_IMU'] = GLOBAL['BANK_X_ZED']-GLOBAL['ESTIMATED_YAW']
            GLOBAL['WORKING_CMD'] = 0
            GLOBAL['ZED_OFFSET_NOW'] = True

            GLOBAL['X_POS_KF'] = actual_x_average + GLOBAL['X_CALIB_GPS']
            GLOBAL['Y_POS_KF'] = actual_y_average + GLOBAL['Y_CALIB_GPS']

            GLOBAL['ESTIMATED_YAW'] = GLOBAL['YAW_IMU']
            print("\n\n\n\n\n--------------------------------------------INITIAL POSITION CALIBRATED--------------------------------------------")
            print("New position X: "+str(GLOBAL['X_POS_KF'])+" Y: "+str(GLOBAL['Y_POS_KF']))
            print("\n\n\n\n\n")
        if(abs(GLOBAL['FILTERED_SPEED'])>0):
            count = count+1
            #Time step at this specific moment
            delta_time = (GLOBAL['EXECUTION_TIME']-GLOBAL['PREV_EXECUTION_TIME'])
            #Division by 3 due to certain gain error presented on the calculation that was not present on the Gazebo simulation
            try:
                GLOBAL['DELTA_BEGIN'] = math.sqrt(math.pow(statistics.mean(actual_x_gps)-GLOBAL['X_POS_KF'],2) + math.pow(statistics.mean(actual_y_gps)-GLOBAL['Y_POS_KF'],2))
            except:
                print("Could not calculate delta from initial point")

            if(GLOBAL['EXECUTION_TIME'] - zed_position_correction_time>10):
                zed_position_correction_time = GLOBAL['EXECUTION_TIME']
                #GLOBAL['X_FRONT_ZED_C'] = GLOBAL['X_FRONT_ZED_TEMP'] - GLOBAL['X_POS_KF']
                #GLOBAL['Y_FRONT_ZED_C'] = GLOBAL['Y_FRONT_ZED_TEMP'] - GLOBAL['Y_POS_KF']

            agv_encoder_long_speed = GLOBAL['FILTERED_SPEED']
            rear_applied_force = GLOBAL['vm'].fxr(acceleration=GLOBAL['XACC_IMU'] ,vx=agv_encoder_long_speed)
            fxf = 0.0
            yaw_dm_input = -1*GLOBAL['ESTIMATED_YAW']

            fzr = GLOBAL['vm'].fzr(fxr=rear_applied_force)
            fzf = GLOBAL['vm'].fzf(fxr=rear_applied_force)
            ar = GLOBAL['vm'].alpha_r(omega=GLOBAL['X1_KF'],vy=GLOBAL['Y_VEL_KF'],vx=agv_encoder_long_speed)
            af = GLOBAL['vm'].alpha_f(omega=GLOBAL['X1_KF'],vy=GLOBAL['Y_VEL_KF'],vx=agv_encoder_long_speed,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'])
            fyf = GLOBAL['vm'].fyf(fzf=fzf,af=af)
            fyr = GLOBAL['vm'].fyr(fzr=fzr,ar=ar)
            GLOBAL['FYR_TEMP'] = fyf
            GLOBAL['FYF_TEMP'] = fyr
            k00 = GLOBAL['vm'].x0dot(omega=GLOBAL['X1_KF'])
            k10 = GLOBAL['vm'].x1dot(fxf=fxf,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'],fyf=fyf,fyr=fyr)
            k20 = GLOBAL['vm'].x2dot(fxf=fxf,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'],fyf=fyf,fyr=fyr,vx=agv_encoder_long_speed,omega=GLOBAL['X1_KF'])
            k30 = GLOBAL['vm'].x3dot(fxf=fxf,fxr=rear_applied_force,fyf=fyf,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'],vy=GLOBAL['Y_VEL_KF'],omega=GLOBAL['X1_KF'])
            k40 = GLOBAL['vm'].x4dot(vy=GLOBAL['Y_VEL_KF'],theta=yaw_dm_input,vx=agv_encoder_long_speed)
            k50 = GLOBAL['vm'].x5dot(vy=GLOBAL['Y_VEL_KF'],theta=yaw_dm_input,vx=agv_encoder_long_speed)

            k01 = GLOBAL['vm'].x0dot(omega=GLOBAL['X1_KF']+0.5*k10)
            k11 = GLOBAL['vm'].x1dot(fxf=fxf,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'],fyf=fyf,fyr=fyr)
            k21 = GLOBAL['vm'].x2dot(fxf=fxf,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'],fyf=fyf,fyr=fyr,vx=agv_encoder_long_speed+0.5*k30, omega=GLOBAL['X1_KF']+0.5*k10)
            k31 = GLOBAL['vm'].x3dot(fxf=fxf,fxr=rear_applied_force,fyf=fyf,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'],vy=GLOBAL['Y_VEL_KF']+0.5*k20, omega=GLOBAL['X1_KF']+0.5*k10)
            k41 = GLOBAL['vm'].x4dot(vy=GLOBAL['Y_VEL_KF']+0.5*k20,theta=yaw_dm_input+0.5*k00,vx=agv_encoder_long_speed+0.5*k30)
            k51 = GLOBAL['vm'].x5dot(vy=GLOBAL['Y_VEL_KF']+0.5*k20,theta=yaw_dm_input+0.5*k00,vx=agv_encoder_long_speed+0.5*k30)

            k02 = GLOBAL['vm'].x0dot(omega=GLOBAL['X1_KF']+0.5*k11)
            k12 = GLOBAL['vm'].x1dot(fxf=fxf,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'],fyf=fyf,fyr=fyr)
            k22 = GLOBAL['vm'].x2dot(fxf=fxf,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'],fyf=fyf,fyr=fyr,vx=agv_encoder_long_speed+0.5*k31, omega=GLOBAL['X1_KF']+0.5*k11)
            k32 = GLOBAL['vm'].x3dot(fxf=fxf,fxr=rear_applied_force,fyf=fyf,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'],vy=GLOBAL['Y_VEL_KF']+0.5*k21, omega=GLOBAL['X1_KF']+0.5*k11)
            k42 = GLOBAL['vm'].x4dot(vy=GLOBAL['Y_VEL_KF']+0.5*k21,theta=yaw_dm_input+0.5*k01,vx=agv_encoder_long_speed+0.5*k31)
            k52 = GLOBAL['vm'].x5dot(vy=GLOBAL['Y_VEL_KF']+0.5*k21,theta=yaw_dm_input+0.5*k01,vx=agv_encoder_long_speed+0.5*k31)

            k03 = GLOBAL['vm'].x0dot(omega=GLOBAL['X1_KF']+k12)
            k13 = GLOBAL['vm'].x1dot(fxf=fxf,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'],fyf=fyf,fyr=fyr)
            k23 = GLOBAL['vm'].x2dot(fxf=fxf,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'],fyf=fyf,fyr=fyr,vx=agv_encoder_long_speed+k32, omega=GLOBAL['X1_KF']+k12)
            k33 = GLOBAL['vm'].x3dot(fxf=fxf,fxr=rear_applied_force,fyf=fyf,st=-1*GLOBAL['FILTERED_STEER_ANGLE_AGV'],vy=GLOBAL['Y_VEL_KF']+k22, omega=GLOBAL['X1_KF']+k12)
            k43 = GLOBAL['vm'].x4dot(vy=GLOBAL['Y_VEL_KF']+k22,theta=yaw_dm_input+k02,vx=agv_encoder_long_speed+k32)
            k53 = GLOBAL['vm'].x5dot(vy=GLOBAL['Y_VEL_KF']+k22,theta=yaw_dm_input+k02,vx=agv_encoder_long_speed+k32)
            
            GLOBAL['PREV_EXECUTION_TIME'] = GLOBAL['EXECUTION_TIME']
            #CAMBIO PARA VALIDAR EL MODELO
            GLOBAL['X0_KF'] = positive_angles(yaw_dm_input+1.0/6.0*(k00+2.0*k01+2.0*k02+k03)*delta_time) #Theta            
            x1_kf_temp= GLOBAL['X1_KF']+1.0/6.0*(k10+2.0*k11+2.0*k12+k13)*delta_time #Omega
            GLOBAL['Y_VEL_KF'] = GLOBAL['Y_VEL_KF']+1.0/6.0*(k20+2.0*k21+2.0*k22+k23)*delta_time #Lateral velocity vy
            #CAMBIO PARA VALIDAR EL MODELO
            x_position_estimate = GLOBAL['X_POS_KF']+1.0/6.0*(k40+2.0*k41+2.0*k42+k43)*delta_time #Global x position
            y_position_estimate = GLOBAL['Y_POS_KF']+1.0/6.0*(k50+2.0*k51+2.0*k52+k53)*delta_time #Global y position
            pose_gain_gps = 0
            pose_gain_zed = 0
            yaw_gain = 0.5
            var_y = 0
            var_x = 0
            xo_transformed = -1*GLOBAL['X0_KF']

            if(count==11):
                print("Ready for variance")         
            time_for_gps = False
            if(GLOBAL['READY_FLAG_FRONT_ZED']==1):
                #GLOBAL['DELTA_ZED_IMU_YAW'] = -1*(GLOBAL['ESTIMATED_YAW'] - GLOBAL['BANK_X_ZED'])
                pose_gain_zed = (1-0.5*GLOBAL['ZED_DELTA_KF'])*0.5
                
            if(pose_gain_zed<0.1):
                time_for_gps = True
                pose_gain_gps = 0.3

            if(not(dm_first_time)):
                dm_first_time = True
                x_position_estimate = GLOBAL['X_POS_KF']
                y_position_estimate = GLOBAL['Y_POS_KF']
                xo_transformed = GLOBAL['ESTIMATED_YAW']
                GLOBAL['X1_KF'] = 0
                GLOBAL['Y_VEL_KF'] = 0
                time_dm = GLOBAL['EXECUTION_TIME']
                time_zed_rotation = GLOBAL['EXECUTION_TIME']
            if(GLOBAL['EXECUTION_TIME']-time_dm > 5):
                time_free_before_gps_correction = True

            else:
                #FIRST DM ESTIMATE MIGHT CAUSE UNSTABLE RESULTS, THEREFORE WE IGNORE IT
                try:
                    if((GLOBAL['ESTIMATED_YAW'] - xo_transformed)>0.05 or abs(x_position_estimate-GLOBAL['X_POS_KF'])>0.0005 or abs(y_position_estimate-GLOBAL['Y_POS_KF'])>0.0005 or (math.isnan(x_position_estimate) or math.isnan(y_position_estimate))):
                        x_position_estimate = GLOBAL['X_POS_KF']
                        y_position_estimate = GLOBAL['Y_POS_KF']
                        xo_transformed = GLOBAL['ESTIMATED_YAW']
                        GLOBAL['X1_KF'] = 0
                        GLOBAL['Y_VEL_KF'] = 0
                        GLOBAL['FAILED_CALCULATION_TIMES'] = GLOBAL['FAILED_CALCULATION_TIMES']+1
                        #print("SOMETHING WENT WRONG BUT, WE WILL HANDLE ;) FAILED TIMES: "+str(GLOBAL['FAILED_CALCULATION_TIMES']))
                    else:
                        GLOBAL['FAILED_CALCULATION_TIMES'] = 0
                        GLOBAL['X1_KF'] = x1_kf_temp
                except:
                    x_position_estimate = GLOBAL['X_POS_KF']
                    y_position_estimate = GLOBAL['Y_POS_KF']
                    xo_transformed = GLOBAL['ESTIMATED_YAW']
                    GLOBAL['X1_KF'] = 0
                    GLOBAL['Y_VEL_KF'] = 0
                    GLOBAL['FAILED_CALCULATION_TIMES'] = GLOBAL['FAILED_CALCULATION_TIMES']+1
                    #print("SOMETHING WENT WRONG BUT, WE WILL HANDLE ;) FAILED TIMES: "+str(GLOBAL['FAILED_CALCULATION_TIMES']))

            #CAMBIO PARA VALIDAR EL MODELO
            if(time_free_before_gps_correction):
                if(GLOBAL['READY_FLAG_IMU_YAW']==1):
                    GLOBAL['READY_FLAG_IMU_YAW'] = 0
                    if(GLOBAL['YAW_IMU']>5 or GLOBAL['YAW_IMU']<1):
                        GLOBAL['ESTIMATED_YAW'] = ANGLE_AVERAGE((positive_angles(xo_transformed)),(GLOBAL['YAW_IMU'] + GLOBAL['IMU_GPS_OFFSET']))
                    else:
                        GLOBAL['ESTIMATED_YAW'] = positive_angles(positive_angles(xo_transformed) + yaw_gain*(positive_angles(GLOBAL['YAW_IMU'] + GLOBAL['IMU_GPS_OFFSET'])-positive_angles(xo_transformed)) )
                    
                    #print("--------------------------------------New IMU data")
                else:
                    GLOBAL['ESTIMATED_YAW'] = positive_angles(xo_transformed)

            
            if(GLOBAL['FILTERED_SPEED']>0.05 and time_for_gps == False and GLOBAL['READY_FLAG_FRONT_ZED']):
                #print("--------------------------------------ZED CORRECTION")
                GLOBAL['READY_FLAG_FRONT_ZED'] = 0
                GLOBAL['WHAT_HAPPEN'] = "ZED"
                GLOBAL['X_POS_KF'] = x_position_estimate + pose_gain_gps*(GLOBAL['X_FRONT_ZED']-x_position_estimate)
                GLOBAL['Y_POS_KF'] = y_position_estimate + pose_gain_gps*(GLOBAL['Y_FRONT_ZED']-y_position_estimate)

            elif(GLOBAL['FILTERED_SPEED']>0.05 and time_for_gps==True and GLOBAL['READY_FLAG_GPS_Y'] and GLOBAL['READY_FLAG_GPS_X']):
                try:
                    GLOBAL['GPS_DELTA'] = math.sqrt(math.pow(GLOBAL['X_POS_KF']-GLOBAL['X_COORDINATE_GPS'],2) + math.pow(GLOBAL['Y_POS_KF']-GLOBAL['Y_COORDINATE_GPS'],2))
                except:
                    GLOBAL['GPS_DELTA'] = 100
                #If the next point is within 3 meters from the previous estimated position
                #avoiding outliers
                GLOBAL['READY_FLAG_GPS_Y'] = False
                GLOBAL['READY_FLAG_GPS_X'] = False

                #DYNAMIC MODEL MIGHT PRESENT ERRATIC BEHAVIOUR AT THE FIRST STEPS
                if(GLOBAL['GPS_DELTA']<GLOBAL['GPS_ADJUSTED_DELTA_TH']):
                    GLOBAL['GPS_ADJUSTED_DELTA_TH'] = bound(10,GLOBAL['GPS_RESET_DELTA'],GLOBAL['GPS_ADJUSTED_DELTA_TH']/math.exp(count_gps_points))#max(GLOBAL['GPS_DELTA_TH'], min(0, GLOBAL['GPS_ADJUSTED_DELTA_TH']-count_gps_points*10))
                    print(GLOBAL['GPS_ADJUSTED_DELTA_TH'])
                    count_gps_points = count_gps_points+1
                    #print("--------------------------------------New GPS position data") 
                    GLOBAL['WHAT_HAPPEN'] = "GPS"
                    GLOBAL['Y_POS_KF'] = y_position_estimate + pose_gain_gps*(GLOBAL['Y_COORDINATE_GPS']-y_position_estimate)
                    GLOBAL['X_POS_KF'] = x_position_estimate + pose_gain_gps*(GLOBAL['X_COORDINATE_GPS']-x_position_estimate)  

                    if(GLOBAL['FILTERED_SPEED']>0.05):#and abs(GLOBAL['STEER_ANGLE_AGV'])<0.104533):                        
                        #If the vehicle is moving save the position to calculate the yaw offset
                        moving_x_gps.append(GLOBAL['X_COORDINATE_GPS'])
                        moving_y_gps.append(GLOBAL['Y_COORDINATE_GPS'])
                        moving_x_gps.pop(0)
                        moving_y_gps.pop(0)
                        xa = GLOBAL['X_COORDINATE_GPS']
                        xp = moving_x_gps[0]
                        ya = GLOBAL['Y_COORDINATE_GPS']
                        yp = moving_y_gps[0]
                        moving_gps_count = moving_gps_count+1
                    else:
                        #If the vehicle suddenly stops, then a yaw angle cannot be calculated from GPS displacements
                        moving_gps_count = 0
                        moving_x_gps = [0,0,0]
                        moving_y_gps = [0,0,0]
                        GLOBAL['GPS_ADJUSTED_DELTA_TH'] = GLOBAL['GPS_RESET_DELTA']
                else:
                    GLOBAL['WHAT_HAPPEN'] = "DM"
                    GLOBAL['Y_POS_KF'] = y_position_estimate 
                    GLOBAL['X_POS_KF'] = x_position_estimate 

                if(moving_gps_count >= 3 and time_free_before_gps_correction):
                    print("--------------------------------------GPS YAW")
                    GLOBAL['GPS_YAW'] = positive_angles(math.atan2(ya-yp,xa-xp))
                    first_gps_correction = True
                    GLOBAL['IMU_GPS_OFFSET'] = (GLOBAL['GPS_YAW']-GLOBAL['YAW_IMU'])
                    moving_gps_count = 0
            else:
                GLOBAL['WHAT_HAPPEN'] = "DM"
                GLOBAL['Y_POS_KF'] = y_position_estimate 
                GLOBAL['X_POS_KF'] = x_position_estimate 
            
            if(GLOBAL['WORKING_CMD'] == 2):
                if(saved_points == 0):
                    try:
                        file2.write(str(GLOBAL['X_DESIRED_PREV'])+","+str(GLOBAL['Y_DESIRED_PREV'])+","+str(GLOBAL['ESTIMATED_YAW'])+"\n")
                        print("Initial point saved : "+str(saved_points))
                    except:
                        print("Initial point not saved")
                    saved_points = saved_points+1

                if(math.sqrt(math.pow(GLOBAL['X_DESIRED_PREV']-GLOBAL['X_POS_KF'],2)+math.pow(GLOBAL['Y_DESIRED_PREV']-GLOBAL['Y_POS_KF'],2))>=1):
                    GLOBAL['X_DESIRED_PREV'] = GLOBAL['X_POS_KF']
                    GLOBAL['Y_DESIRED_PREV'] = GLOBAL['Y_POS_KF']
                    try:
                        file2.write(str(GLOBAL['X_DESIRED_PREV'])+","+str(GLOBAL['Y_DESIRED_PREV'])+","+str(GLOBAL['ESTIMATED_YAW'])+"\n")
                        print("New point saved : "+str(saved_points))
                    except:
                        print("Point not saved")
                    saved_points = saved_points+1
            
            

            #IS THE ZED POSE FAR AWAY FROM WHAT WE ACTUALLY ESTIMATE?
            try:
                GLOBAL['ZED_DELTA_KF'] = math.sqrt(math.pow(GLOBAL['X_POS_KF']-GLOBAL['X_FRONT_ZED'],2)+math.pow(GLOBAL['Y_POS_KF']-GLOBAL['Y_FRONT_ZED'],2))
            except:
                print("\n\n\nCOULD NOT CALCULATE ZED DIVERGENCE\n\n\n")

            if((GLOBAL['ZED_DELTA_KF'] >2 ) and GLOBAL['FILTERED_STEER_ANGLE_AGV']<=0.10 and GLOBAL['FILTERED_SPEED']>0.1 and time_free_before_gps_correction and GLOBAL['EXECUTION_TIME']-time_zed_rotation>5):
                time_zed_rotation = GLOBAL['EXECUTION_TIME']
                GLOBAL['ROTATION_DISPLACEMENT_ZED_IMU'] = positive_angles(GLOBAL['BANK_X_ZED']-positive_angles(GLOBAL['YAW_IMU'] + GLOBAL['IMU_GPS_OFFSET']))
                GLOBAL['ZED_OFFSET_NOW'] = True
                print("\n\n\n----------------------------------------------------------------------------FIRST CORRECTION -\n\n\n")

        else: #(GLOBAL['FILTERED_SPEED'] == 0):
            #If the vehicle is not moving, reset dynamic model variables
            GLOBAL['X0_KF'] = 0
            GLOBAL['X1_KF'] = 0
            GLOBAL['Y_VEL_KF'] = 0
            dm_first_time = False
            GLOBAL['GPS_ADJUSTED_DELTA_TH']=GLOBAL['GPS_RESET_DELTA']

        #Follow Route
        found = False
        if(GLOBAL['WORKING_CMD']==6):
            distance_prev = 100                    
            for point in range(GLOBAL['NEXT_ROUTE_POINT']+1,GLOBAL['PATH_POINTS']):
                distance = math.sqrt(math.pow(GLOBAL['ROUTE_DATA'][point,0]- GLOBAL['X_POS_KF'],2)+math.pow(GLOBAL['ROUTE_DATA'][point,1]- GLOBAL['Y_POS_KF'],2))
                if(distance<3):
                    GLOBAL['NEXT_ROUTE_POINT'] = point
                    distance_prev = distance
                    found = True
                    break

            if(not(GLOBAL['NEXT_ROUTE_POINT'] <GLOBAL['PATH_POINTS'])):
                print("Done Following Path Baby")
                GLOBAL['WORKING_CMD'] = 0
                pubCMD.publish(0)
                break
            if(GLOBAL['NEXT_ROUTE_POINT']+viewing_horizon<GLOBAL['PATH_POINTS']-1-viewing_horizon):
                controller_data = [GLOBAL['X_POS_KF'],
                GLOBAL['Y_POS_KF'],GLOBAL['ESTIMATED_YAW'],
                GLOBAL['ROUTE_DATA'][GLOBAL['NEXT_ROUTE_POINT']+viewing_horizon,0],
                GLOBAL['ROUTE_DATA'][GLOBAL['NEXT_ROUTE_POINT']+viewing_horizon,1],
                GLOBAL['ROUTE_DATA'][GLOBAL['NEXT_ROUTE_POINT']+viewing_horizon,2],
                GLOBAL['FILTERED_SPEED'],GLOBAL['NEXT_ROUTE_POINT'],GLOBAL['PATH_POINTS']]
                controller_data_stdmsgs = Float32MultiArray(data=controller_data)
                pubControllerData.publish(controller_data_stdmsgs)
            else:
                controller_data = [GLOBAL['X_POS_KF'],
                GLOBAL['Y_POS_KF'],GLOBAL['ESTIMATED_YAW'],
                GLOBAL['ROUTE_DATA'][GLOBAL['NEXT_ROUTE_POINT'],0],
                GLOBAL['ROUTE_DATA'][GLOBAL['NEXT_ROUTE_POINT'],1],
                GLOBAL['ROUTE_DATA'][GLOBAL['NEXT_ROUTE_POINT'],2],
                GLOBAL['FILTERED_SPEED'],GLOBAL['NEXT_ROUTE_POINT'],GLOBAL['PATH_POINTS']]
                controller_data_stdmsgs = Float32MultiArray(data=controller_data)
                pubControllerData.publish(controller_data_stdmsgs)

        if(GLOBAL['READY_FLAG_GPS_Y'] and GLOBAL['READY_FLAG_GPS_X'] and GLOBAL['WORKING_CMD'] == 8):
            actual_x_gps.append(GLOBAL['X_COORDINATE_GPS'])
            actual_y_gps.append(GLOBAL['Y_COORDINATE_GPS'])
            actual_x_gps.pop(0)
            actual_y_gps.pop(0)
            GLOBAL['ESTIMATED_YAW'] = GLOBAL['YAW_IMU']
            xa = GLOBAL['X_COORDINATE_GPS']
            xp = actual_x_gps[7]
            ya = GLOBAL['Y_COORDINATE_GPS']
            yp = actual_y_gps[7]
            GLOBAL['READY_FLAG_GPS_Y']=False
            GLOBAL['READY_FLAG_GPS_X']=False
            GLOBAL['READY_FLAG_IMU_YAW'] =False
            print("-------------------------------------------------------------------------New GPS position data")
            print("New coordinates")
            print("X: "),
            print(xa),
            print("Y: "),
            print(ya), 
            print(" Heading: "),
            print(GLOBAL['YAW_IMU'])
        count = count + 1
        time.sleep(0.01)
        #rate.sleep()
    print("Finishing Program and Saving Route File")
    pubCCMD.publish(0)
    file2.close()
    file1.close()
if __name__ == '__main__':
	try:
		MAIN()
	except (KeyboardInterrupt,rospy.ROSInterruptException):
		print("\nEnding execution")