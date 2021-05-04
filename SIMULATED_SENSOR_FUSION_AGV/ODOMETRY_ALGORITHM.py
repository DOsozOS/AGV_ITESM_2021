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

PI = 3.14159265358979323846264338327950288
READY_FLAG_GAZEBO = 0
READY_FLAG_FRONT_OS2 = 0
READY_FLAG_REAR_OS2 = 0
READY_FLAG_IMU = 0
ReadyFlag_lidar = 0
SteerAngle = 0

x = datetime.datetime.now()
file2 = open("saved_route_1.csv","w")
filename = "SIMULATION_FILES/PATH_FOLLOW_TEST_"+str(x.date())+"_"+str(x.time())+"_SIMULATION"+".csv"
file1 = open(filename,"w")

counter = 4
#IMU Data
XACC_IMU = 0.0
YACC_IMU = 0.0
ZACC_IMU = 0.0
XANG_SPEED_IMU = 0.0
YANG_SPEED_IMU = 0.0
ZANG_SPEED_IMU = 0.0

SIM_TIME_GAZEBO = 0.0
PREV_SIM_TIME_GAZEBO = 0.0

YAW_Z_IMU = 0.0 #psi
PITCH_Y_IMU = 0.0#theta
BANK_X_IMU  = 0.0#phi
#GPS position estimate
LATITUDE_GPS = 0.0
LONGITUDE_GPS = 0.0
Y_COORDINATE_GPS   = 0.0
X_COORDINATE_GPS   = 0.0
GPS_UPDATE_TIME = 0.0
GPS_READY_FLAG = 0
#COORDINATES OF YOUR REFERENCE 0,0 
LONGITUD_OFFSET = 19.018523
LATITUDE_OFFSET  = -98.242336
#ORB SLAM 2 position estimate 1
X_FRONT_OS2 = 0.0
Y_FRONT_OS2 = 0.0
YAW_FRONT_OS2 = 0.0
FRONT_QUALITY_OS2 = 0
#ORB SLAM 2 position estimate 2
X_REAR_OS2 = 0.0
Y_REAR_OS2 = 0.0
YAW_REAR_OS2 = 0.0
REAR_QUALITY_OS2 = 0
#Real Position
X_REAL_GAZEBO = 0.0
Y_REAL_GAZEBO = 0.0
YAW_REAL_GAZEBO = 0.0
LONGITUDINAL_VELOCITY_GAZEBO = 0.0
ROTARY_VELOCITY_GAZEBO = 0.0
LATERAL_VELOCITY_GAZEBO      = 0.0
#MODEL variables
STEER_ANGLE_AGV_GAZEBO = 0

#KALMAN FILTER VARIABLES--------------------------------------------------------------------------------------------------
FIRST_TIME = False
#Initial conditions
#    pass
X0_KF = 0.0 #Theta
X1_KF = 0.0 #Omega
Y_VEL_KF = 0.0 #Lateral velocity vy
#x3 = 0.0 #Longitudinal velocity vx
X_POS_KF = 0.0 #Global x position
Y_POS_KF = 0.0 #Global y position
Pk0 = np.array([[10, 0.0001],[0.0001, 10]])
PK = np.array([])
XK = np.array([])
stdX = (19.01764309)
stdY = (19.39629044)
varX = math.pow(stdX,2)
varY = math.pow(stdY,2)
vm = vehicle_model(vehicle_mass=400.0,vehicle_l1=0.669670,vehicle_l2=0.669670,
            vehicle_width=1.16,height_iz=0.38,vehicle_wheel_radius=0.2876,
            vehicle_initial_theta=0.0,vehicle_initial_vx=0.0,vehicle_initial_vy=0.0,
            vehicle_initial_x=0.0,vehicle_initial_y=0.0)
Xdata = []
Ydata = []
RXdata = []
RYdata = []
GPSXdata = []
GPSYdata = []
Time = []
KGxdata = []
KGydata = []
AVGdata = []
ESTIMATED_YAW = 0
A = np.array([[1.0,0.0],[0.0,1.0]])
H = np.array([[1.0,0.0],[0.0,1.0]])
XK_M1 = np.array([[0.0],[0.0]])
XK = np.array([[0.0],[0.0]])
PK = np.array([[0,0],[0,0]])
PK_M1 = Pk0
R = np.array([[varX,stdX*stdY],[stdX*stdY,varY]])
ErrorArray = []
ESTt_x = 0
ESTt_y = 0
EESTt_x = 10
EESTt_y = 10
EMEA_x = 6
EMEA_y = 6
#Saving route for the vehile trajectory
WORKING_CMD = 0
X_DESIRED_PREV = 0
Y_DESIRED_PREV = 0
PATH_POINTS = 0
ROUTE_DATA = np.empty([2,2])
ACTUAL_ROUTE_POINT = 0
NEXT_ROUTE_POINT = 1
delta_angle_point2point = 0
delta_theta_as = 0
vehicle_relative_angle = 0
GLOBAL = globals()
def transform_coordinates(xa,ya,xn,yn,thetaa):
    #displace coordinates
    xt = xn-xa
    yt = yn-ya
    #rotate coordinates
    xr = xt*math.cos(thetaa) + yt*math.sin(thetaa)
    yr = -xt*math.sin(thetaa) + yt*math.cos(thetaa)
    return np.array([xr,yr])
def sign_generator(time):
    return 1 if math.sin(time*0.1) < 0 else -1
def GPS_simulator(x, y,time):
    if (time-GPS_UPDATE_TIME > 0.1):
        GLOBAL['GPS_UPDATE_TIME'] = time
        GLOBAL['GPS_READY_FLAG'] = 1
        oscilation = 0.0011
        #y axis will represent the latitude
        #x axis represents the longitude
        sign = sign_generator(time)
        amplitude_gain     = abs(1.2*math.sin(time*2.5))*math.pow(random.weibullvariate(1,1.9),3)*oscilation/2*sign
        new_lat_variation  = math.cos(time)*oscilation-amplitude_gain
        amplitude_gain     = abs(1.2*math.sin(time*2.5))*math.pow(random.weibullvariate(1,1.9),3)*oscilation/2*sign
        new_long_variation = math.sin(time)*oscilation-amplitude_gain
        actual_lat =((y/1000/111.321)+new_lat_variation/50 + LATITUDE_OFFSET)
        actual_long=((x/1000/111.321)+new_long_variation/50+ LONGITUD_OFFSET)
        return_data = np.array([actual_lat,actual_long])
        return return_data

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
    GLOBAL['XACC_IMU'] = message.linear_acceleration.x
    GLOBAL['YACC_IMU'] = message.linear_acceleration.y
    GLOBAL['ZACC_IMU'] = message.linear_acceleration.z
    GLOBAL['XANG_SPEED_IMU'] = message.angular_velocity.x
    GLOBAL['YANG_SPEED_IMU'] = message.angular_velocity.y
    GLOBAL['ZANG_SPEED_IMU'] = message.angular_velocity.z
    q0 = message.orientation.w
    q1 = message.orientation.x
    q2 = message.orientation.y
    q3 = message.orientation.z
    GLOBAL['YAW_Z_IMU'] = positive_angles(math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2)))+random.random()*0.001
    GLOBAL['PITCH_Y_IMU'] = positive_angles(math.asin(2*(q0*q2-q3*q1)))+random.random()*0.001
    GLOBAL['BANK_X_IMU'] = positive_angles(math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3)))+random.random()*0.001
def OS_2_POSE_CALLBACK(message):
    GLOBAL['READY_FLAG_REAR_OS2'] = 1
    GLOBAL['X_REAR_OS2'] = message.pose.position.x 
    GLOBAL['Y_REAR_OS2'] = message.pose.position.y  
    GLOBAL['X_REAR_OS2'] = round(GLOBAL['X_REAR_OS2'], 4)
    GLOBAL['Y_REAR_OS2'] = round(GLOBAL['Y_REAR_OS2'], 4)
    GLOBAL['YAW_REAR_OS2'] = -1*(message.pose.orientation.y)*PI/180.0
def OS_2_QUALITY_CALLBACK(message):
    GLOBAL['REAR_QUALITY_OS2'] = message.data
def OS_1_POSE_CALLBACK(message):
    GLOBAL['READY_FLAG_FRONT_OS2'] = 1
    GLOBAL['X_FRONT_OS2'] = message.pose.position.x 
    GLOBAL['Y_FRONT_OS2'] = message.pose.position.y  
    GLOBAL['X_FRONT_OS2'] = round(GLOBAL['X_FRONT_OS2'], 4)
    GLOBAL['Y_FRONT_OS2'] = round(GLOBAL['Y_FRONT_OS2'], 4)
    GLOBAL['YAW_FRONT_OS2'] = -1*(message.pose.orientation.y)*PI/180.0
def OS_1_QUALITY_CALLBACK(message):
    GLOBAL['FRONT_QUALITY_OS2'] = message.data
def GAZEBO_CALLBACK(message):
    GLOBAL['READY_FLAG_GAZEBO'] = 1
    GLOBAL['X_REAL_GAZEBO'] = message.data[0]
    GLOBAL['Y_REAL_GAZEBO'] = message.data[1]
    GLOBAL['X_REAL_GAZEBO'] = round(GLOBAL['X_REAL_GAZEBO'],4)
    GLOBAL['Y_REAL_GAZEBO'] = round(GLOBAL['Y_REAL_GAZEBO'],4)
    GLOBAL['ROTARY_VELOCITY_GAZEBO'] = message.data[3]
    GLOBAL['LONGITUDINAL_VELOCITY_GAZEBO'] = message.data[4]
    GLOBAL['LATERAL_VELOCITY_GAZEBO'] = message.data[5]# + random.random()
    GLOBAL['SIM_TIME_GAZEBO']   = message.data[6]
    GLOBAL['YAW_REAL_GAZEBO'] = positive_angles(message.data[7])
    gpsData = GPS_simulator(X_REAL_GAZEBO,Y_REAL_GAZEBO,SIM_TIME_GAZEBO)
    if(gpsData is not None):
        GLOBAL['LATITUDE_GPS'] = gpsData[0]
        GLOBAL['LONGITUDE_GPS'] = gpsData[1]
        GLOBAL['Y_COORDINATE_GPS']   = GLOBAL['Y_REAL_GAZEBO'] + np.random.normal(0,2)#(gpsData[0]-LATITUDE_OFFSET)*110.57*1000.0
        GLOBAL['X_COORDINATE_GPS']   = GLOBAL['X_REAL_GAZEBO'] + np.random.normal(0,2)#(gpsData[1]-LONGITUD_OFFSET)*110.57*1000.0
def STEERING_CALLBACK(message):
    GLOBAL['STEER_ANGLE_AGV_GAZEBO'] = message.data
def CMD_CALLBACK(message):
    GLOBAL['WORKING_CMD'] = message.data
def MAIN():
    rospy.init_node('agv_control', anonymous=1)
    pub = rospy.Publisher('/my_agv/status_cmd', Int32, queue_size=1)
    pubControllerData = rospy.Publisher('/my_agv/controller_data', Float32MultiArray, queue_size=1)
    rospy.Subscriber("/my_agv/status_pub", Float32MultiArray, GAZEBO_CALLBACK)
    rospy.Subscriber("/ORB_SLAM2_1/pose", PoseStamped, OS_1_POSE_CALLBACK)
    rospy.Subscriber("/ORB_SLAM2_1/quality", Int32, OS_1_QUALITY_CALLBACK)
    rospy.Subscriber("/my_agv_controller/working_cmd", Int32, CMD_CALLBACK)
    pubCMD = rospy.Publisher('/my_agv_controller/working_cmd', Int32, queue_size=1)
    pubTIME = rospy.Publisher('/my_agv_controller/time', Float32, queue_size=1)
    pubCCMD = rospy.Publisher('/my_agv_controller/working_cmd_automatic', Int32, queue_size=1)
    #rospy.Subscriber("/ORB_SLAM2_2/pose", PoseStamped, OS_2_POSE_CALLBACK)
    #rospy.Subscriber("/ORB_SLAM2_2/quality", Int32, OS_2_QUALITY_CALLBACK)
    rospy.Subscriber("/my_agv/imu", Imu, IMU_CALLBACK)
    rospy.Subscriber("/my_agv/steering_cmd",Float32,STEERING_CALLBACK)
    rate = rospy.Rate(100) 
    print("Welcome to the AGV System")
    #Customize your ackerman vehicle (refer to the book introduction to ground vehicles)
    GLOBAL['vm'] = vehicle_model(vehicle_mass=400.0,vehicle_l1=0.669670,vehicle_l2=0.669670,
            vehicle_width=1.16,height_iz=0.38,vehicle_wheel_radius=0.2876,
            vehicle_initial_theta=0.0,vehicle_initial_vx=0.0,vehicle_initial_vy=0.0,
            vehicle_initial_x=0.0,vehicle_initial_y=0.0)
    actual_time = 0
    variance_x_gps = [0,0,0,0,0,0,0,0,0,0]
    variance_y_gps = [0,0,0,0,0,0,0,0,0,0]
    initial_x_gps = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    initial_y_gps = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    variance_x_os2 = [0,0,0,0,0,0,0,0,0,0]
    variance_y_os2 = [0,0,0,0,0,0,0,0,0,0]
    actual_x_gps = [0,0,0,0,0,0,0,0,0,0]
    actual_y_gps = [0,0,0,0,0,0,0,0,0,0]
    count = 0
    saved_points = 0

    while not (rospy.is_shutdown() ):
        pub.publish(0)
        pubTIME.publish(GLOBAL['SIM_TIME_GAZEBO'])
        if ( GLOBAL['WORKING_CMD'] == 5):
            pubCCMD.publish(0)
            break
        #Load route
        if(GLOBAL['WORKING_CMD'] == 7):
            GLOBAL['WORKING_CMD'] = 0
            Data = []
            line_count = 0
            with open('saved_route_3.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for row in csv_reader:
                    floatRow = [float(row[0]),
                    float(row[1]),
                    float(row[2])]
                    Data.append(floatRow)
                    line_count += 1
            print("---------------------------Loading path "+str(line_count)+" points")
            
            GLOBAL['PATH_POINTS']=line_count
            print("Total trajectory points " + str(GLOBAL['PATH_POINTS']))
            GLOBAL['ROUTE_DATA'] = np.array(Data)
        if(GLOBAL['WORKING_CMD'] == 4):
            print("Setting the predefined coordinates")
            GLOBAL['X_DESIRED_PREV'] = 0
            GLOBAL['Y_DESIRED_PREV'] = 0
            GLOBAL['WORKING_CMD'] = 0
            GLOBAL['X_POS_KF'] = 0
            GLOBAL['Y_POS_KF'] = 0
        if(GLOBAL['READY_FLAG_GAZEBO']==1 and GLOBAL['READY_FLAG_IMU']==1 and (GLOBAL['GPS_READY_FLAG']==1 or GLOBAL['READY_FLAG_FRONT_OS2']==1) and count > 2):
            actual_time = GLOBAL['SIM_TIME_GAZEBO']
            if(GLOBAL['FIRST_TIME'] == False):
                GLOBAL['FIRST_TIME'] = True
                GLOBAL['PREV_SIM_TIME_GAZEBO'] = actual_time
            else:
                if(count == 1):
                    print("Setting the initial coordinate to: ")
                    GLOBAL['X_POS_KF'] = np.average(initial_x_gps)
                    GLOBAL['Y_POS_KF'] = np.average(initial_y_gps)
                    print("X: "+str(GLOBAL['X_POS_KF']))
                    print("Y: "+str(Y_COORDINATE_GPSGLOBAL['Y_POS_KF']))
                count = count+1
                #Time step at this specific moment
                delta_time = actual_time-GLOBAL['PREV_SIM_TIME_GAZEBO']
                agv_encoder_long_speed = (ROTARY_VELOCITY_GAZEBO)*0.28735
                rear_applied_force = GLOBAL['vm'].fxr(acceleration=GLOBAL['XACC_IMU'] ,vx=agv_encoder_long_speed)
                fxf = 0.0
                yaw_dm_input = -1*GLOBAL['ESTIMATED_YAW']
                fzr = GLOBAL['vm'].fzr(fxr=rear_applied_force)
                fzf = GLOBAL['vm'].fzf(fxr=rear_applied_force)
                ar = GLOBAL['vm'].alpha_r(omega=GLOBAL['X1_KF'],vy=GLOBAL['Y_VEL_KF'],vx=agv_encoder_long_speed)
                af = GLOBAL['vm'].alpha_f(omega=GLOBAL['X1_KF'],vy=GLOBAL['Y_VEL_KF'],vx=agv_encoder_long_speed,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'])
                fyf = GLOBAL['vm'].fyf(fzf=fzf,af=af)
                fyr = GLOBAL['vm'].fyr(fzr=fzr,ar=ar)

                k00 = GLOBAL['vm'].x0dot(omega=GLOBAL['X1_KF'])
                k10 = GLOBAL['vm'].x1dot(fxf=fxf,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'],fyf=fyf,fyr=fyr)
                k20 = GLOBAL['vm'].x2dot(fxf=fxf,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'],fyf=fyf,fyr=fyr,vx=agv_encoder_long_speed,omega=GLOBAL['X1_KF'])
                k30 = GLOBAL['vm'].x3dot(fxf=fxf,fxr=rear_applied_force,fyf=fyf,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'],vy=GLOBAL['Y_VEL_KF'],omega=GLOBAL['X1_KF'])
                k40 = GLOBAL['vm'].x4dot(vy=GLOBAL['Y_VEL_KF'],theta=yaw_dm_input,vx=agv_encoder_long_speed)
                k50 = GLOBAL['vm'].x5dot(vy=GLOBAL['Y_VEL_KF'],theta=yaw_dm_input,vx=agv_encoder_long_speed)

                k01 = GLOBAL['vm'].x0dot(omega=GLOBAL['X1_KF']+0.5*k10)
                k11 = GLOBAL['vm'].x1dot(fxf=fxf,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'],fyf=fyf,fyr=fyr)
                k21 = GLOBAL['vm'].x2dot(fxf=fxf,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'],fyf=fyf,fyr=fyr,vx=agv_encoder_long_speed+0.5*k30, omega=GLOBAL['X1_KF']+0.5*k10)
                k31 = GLOBAL['vm'].x3dot(fxf=fxf,fxr=rear_applied_force,fyf=fyf,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'],vy=GLOBAL['Y_VEL_KF']+0.5*k20, omega=GLOBAL['X1_KF']+0.5*k10)
                k41 = GLOBAL['vm'].x4dot(vy=GLOBAL['Y_VEL_KF']+0.5*k20,theta=yaw_dm_input+0.5*k00,vx=agv_encoder_long_speed+0.5*k30)
                k51 = GLOBAL['vm'].x5dot(vy=GLOBAL['Y_VEL_KF']+0.5*k20,theta=yaw_dm_input+0.5*k00,vx=agv_encoder_long_speed+0.5*k30)

                k02 = GLOBAL['vm'].x0dot(omega=GLOBAL['X1_KF']+0.5*k11)
                k12 = GLOBAL['vm'].x1dot(fxf=fxf,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'],fyf=fyf,fyr=fyr)
                k22 = GLOBAL['vm'].x2dot(fxf=fxf,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'],fyf=fyf,fyr=fyr,vx=agv_encoder_long_speed+0.5*k31, omega=GLOBAL['X1_KF']+0.5*k11)
                k32 = GLOBAL['vm'].x3dot(fxf=fxf,fxr=rear_applied_force,fyf=fyf,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'],vy=GLOBAL['Y_VEL_KF']+0.5*k21, omega=GLOBAL['X1_KF']+0.5*k11)
                k42 = GLOBAL['vm'].x4dot(vy=GLOBAL['Y_VEL_KF']+0.5*k21,theta=yaw_dm_input+0.5*k01,vx=agv_encoder_long_speed+0.5*k31)
                k52 = GLOBAL['vm'].x5dot(vy=GLOBAL['Y_VEL_KF']+0.5*k21,theta=yaw_dm_input+0.5*k01,vx=agv_encoder_long_speed+0.5*k31)

                k03 = GLOBAL['vm'].x0dot(omega=GLOBAL['X1_KF']+k12)
                k13 = GLOBAL['vm'].x1dot(fxf=fxf,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'],fyf=fyf,fyr=fyr)
                k23 = GLOBAL['vm'].x2dot(fxf=fxf,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'],fyf=fyf,fyr=fyr,vx=agv_encoder_long_speed+k32, omega=GLOBAL['X1_KF']+k12)
                k33 = GLOBAL['vm'].x3dot(fxf=fxf,fxr=rear_applied_force,fyf=fyf,st=-1*GLOBAL['STEER_ANGLE_AGV_GAZEBO'],vy=GLOBAL['Y_VEL_KF']+k22, omega=GLOBAL['X1_KF']+k12)
                k43 = GLOBAL['vm'].x4dot(vy=GLOBAL['Y_VEL_KF']+k22,theta=yaw_dm_input+k02,vx=agv_encoder_long_speed+k32)
                k53 = GLOBAL['vm'].x5dot(vy=GLOBAL['Y_VEL_KF']+k22,theta=yaw_dm_input+k02,vx=agv_encoder_long_speed+k32)
                

                #CAMBIO PARA VALIDAR EL MODELO
                GLOBAL['X0_KF'] = positive_angles(yaw_dm_input+1.0/6.0*(k00+2.0*k01+2.0*k02+k03)*delta_time) #Theta
                #GLOBAL['ESTIMATED_YAW'] = yaw_dm_input+1.0/6.0*(k00+2.0*k01+2.0*k02+k03)*delta_time #Theta
                
                GLOBAL['X1_KF'] = GLOBAL['X1_KF']+1.0/6.0*(k10+2.0*k11+2.0*k12+k13)*delta_time #Omega
                GLOBAL['Y_VEL_KF'] = GLOBAL['Y_VEL_KF']+1.0/6.0*(k20+2.0*k21+2.0*k22+k23)*delta_time #Lateral velocity vy
                x1_kf_temp = GLOBAL['X1_KF']
                #CAMBIO PARA VALIDAR EL MODELO
                x_position_estimate = GLOBAL['X_POS_KF']+1.0/6.0*(k40+2.0*k41+2.0*k42+k43)*delta_time #Global x position
                y_position_estimate = GLOBAL['Y_POS_KF']+1.0/6.0*(k50+2.0*k51+2.0*k52+k53)*delta_time #Global y position

                transformed_x_os2 = GLOBAL['X_FRONT_OS2']
                transformed_y_os2 = GLOBAL['Y_FRONT_OS2']
                pose_gain = 0
                var_y = 0
                var_x = 0
                #FIRST DM ESTIMATE MIGHT CAUSE UNSTABLE RESULTS, THEREFORE WE IGNORE IT
                try:
                    if((GLOBAL['ESTIMATED_YAW'] - xo_transformed)>0.5 or abs(x_position_estimate-GLOBAL['X_POS_KF'])>0.5 or abs(y_position_estimate-GLOBAL['Y_POS_KF'])>0.5 or (math.isnan(x_position_estimate) or math.isnan(y_position_estimate))):
                        x_position_estimate = GLOBAL['X_POS_KF']
                        y_position_estimate = GLOBAL['Y_POS_KF']
                        xo_transformed = GLOBAL['ESTIMATED_YAW']
                        GLOBAL['X1_KF'] = 0
                        GLOBAL['Y_VEL_KF'] = 0
                        print("Skip")
                    else:
                        GLOBAL['X1_KF'] = x1_kf_temp
                except:
                    x_position_estimate = GLOBAL['X_POS_KF']
                    y_position_estimate = GLOBAL['Y_POS_KF']
                    xo_transformed = GLOBAL['ESTIMATED_YAW']
                    GLOBAL['X1_KF'] = 0
                    GLOBAL['Y_VEL_KF'] = 0
                    print("Somethin going wrong but we will fix it")


                if(count==11):
                    print("Ready for variance")         
                time_for_gps = False
                if(GLOBAL['READY_FLAG_FRONT_OS2']==1):
                    pose_gain = max(0,min(1,-0.8682281 - (-0.02331669/0.01221943)*(1 - math.exp(-0.01221943*GLOBAL['FRONT_QUALITY_OS2'] ))))
                    print("Calculating pose gain with quality = "+str(GLOBAL['FRONT_QUALITY_OS2']))
                    GLOBAL['READY_FLAG_FRONT_OS2']=0
                
                if(pose_gain<0.1):
                    time_for_gps = True
                    pose_gain = 0.001


                #CAMBIO PARA VALIDAR EL MODELO
                #GLOBAL['ESTIMATED_YAW'] = (ANGLE_AVERAGE(positive_angles(GLOBAL['X0_KF']),positive_angles(GLOBAL['BANK_X_IMU'])))
                xo_transformed = positive_angles(-1*GLOBAL['X0_KF'])
                GLOBAL['ESTIMATED_YAW'] = ANGLE_AVERAGE((xo_transformed),(GLOBAL['BANK_X_IMU']))
                GLOBAL['X0_KF'] = -1*GLOBAL['ESTIMATED_YAW']

                actual_x_gps.append(GLOBAL['X_COORDINATE_GPS'])
                actual_y_gps.append(GLOBAL['Y_COORDINATE_GPS'])

                actual_x_gps.pop(0)
                actual_y_gps.pop(0)

                actual_x_average = statistics.mean(actual_x_gps)
                actual_y_average = statistics.mean(actual_y_gps)

                actual_x_std = statistics.stdev(actual_x_gps)
                actual_y_std = statistics.stdev(actual_y_gps)

                if(GLOBAL['ROTARY_VELOCITY_GAZEBO']>0.05 and time_for_gps == False):
                    print("-------------------------------------------------------OS2 CORRECTION")
                    GLOBAL['X_POS_KF'] = x_position_estimate + pose_gain*(transformed_x_os2-x_position_estimate)
                    GLOBAL['Y_POS_KF'] = y_position_estimate + pose_gain*(transformed_y_os2-y_position_estimate)
                elif(GLOBAL['ROTARY_VELOCITY_GAZEBO']>0.05 and time_for_gps==True):
                    print("-------------------------------------------------------GPS CORRECTION")
                    GLOBAL['Y_POS_KF'] = y_position_estimate + pose_gain*(GLOBAL['Y_COORDINATE_GPS']-y_position_estimate)
                    GLOBAL['X_POS_KF'] = x_position_estimate + pose_gain*(GLOBAL['X_COORDINATE_GPS']-x_position_estimate)

                ERROR_KF = math.sqrt(math.pow(GLOBAL['X_POS_KF']-GLOBAL['X_REAL_GAZEBO'],2)+math.pow(GLOBAL['Y_POS_KF']-GLOBAL['Y_REAL_GAZEBO'],2))

                print("--------------------------------------")
                print("Error: "+str(ERROR_KF))
                print("Actual point is = "+ " X:"+str(round(GLOBAL['X_POS_KF'],2))+
                    " Y:"+str(round(GLOBAL['Y_POS_KF'],2)) + " theta "+str(round(GLOBAL['ESTIMATED_YAW'],3)))

                
                

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
                
                #Follow Route
                found = False
                tfc_x = 0
                tfc_y = 0

                if(GLOBAL['WORKING_CMD']==6):
                    if(GLOBAL['ROTARY_VELOCITY_GAZEBO']>1):
                        pubCCMD.publish(1)
                    else:
                        pubCCMD.publish(2)
                        print("Not stable speed")
                    distance_prev = 100                    
                    for point in range(GLOBAL['NEXT_ROUTE_POINT']+1,GLOBAL['PATH_POINTS']):
                        distance = math.sqrt(math.pow(GLOBAL['ROUTE_DATA'][point,0]- GLOBAL['X_POS_KF'],2)+math.pow(GLOBAL['ROUTE_DATA'][point,1]- GLOBAL['Y_POS_KF'],2))
                        if(distance<2):
                            GLOBAL['NEXT_ROUTE_POINT'] = point
                            distance_prev = distance
                            found = True
                            break
                    
                    if(not(GLOBAL['NEXT_ROUTE_POINT'] <GLOBAL['PATH_POINTS'])):
                        print("Done Following Path Baby")
                        GLOBAL['WORKING_CMD'] = 0
                        pubCMD.publish(0)
                        break

                    print("Next point: " +str(GLOBAL['NEXT_ROUTE_POINT']))

                    controller_data = [GLOBAL['X_POS_KF'],
                    GLOBAL['Y_POS_KF'],GLOBAL['ESTIMATED_YAW'],
                    GLOBAL['ROUTE_DATA'][GLOBAL['NEXT_ROUTE_POINT'],0],
                    GLOBAL['ROUTE_DATA'][GLOBAL['NEXT_ROUTE_POINT'],1],
                    GLOBAL['ROUTE_DATA'][GLOBAL['NEXT_ROUTE_POINT'],2],
                    GLOBAL['ROTARY_VELOCITY_GAZEBO'],GLOBAL['NEXT_ROUTE_POINT'],GLOBAL['PATH_POINTS']]
                    controller_data_stdmsgs = Float32MultiArray(data=controller_data)
                    pubControllerData.publish(controller_data_stdmsgs)
                    

                file1.write(str(GLOBAL['SIM_TIME_GAZEBO'])+","+#0
                str(GLOBAL['X_POS_KF'])+","+#1
                str(GLOBAL['Y_POS_KF'])+","+#2
                str(GLOBAL['X_COORDINATE_GPS'])+","+#3
                str(GLOBAL['Y_COORDINATE_GPS'])+","+#4
                str(GLOBAL['ESTIMATED_YAW'])+","+#5
                str(GLOBAL['X_REAL_GAZEBO'])+","+#6
                str(GLOBAL['Y_REAL_GAZEBO'])+","+#7
                str(GLOBAL['YAW_REAL_GAZEBO'])+","+#8
                str(GLOBAL['STEER_ANGLE_AGV_GAZEBO'])+","+#9
                str(transformed_x_os2)+","+
                str(transformed_y_os2)+","+
                str(ERROR_KF)+"\n")#10

            GLOBAL['READY_FLAG_GAZEBO'] = 0
            GLOBAL['READY_FLAG_IMU'] = 0
            GLOBAL['GPS_READY_FLAG'] = 0
            GLOBAL['PREV_SIM_TIME_GAZEBO'] = actual_time

        elif (GLOBAL['GPS_READY_FLAG']==1 and count<=50):
            GLOBAL['GPS_READY_FLAG']=0
            initial_x_gps.pop(0)
            initial_x_gps.append(GLOBAL['X_COORDINATE_GPS'])
            initial_y_gps.pop(0)
            initial_y_gps.append(GLOBAL['Y_COORDINATE_GPS'])
            count = count + 1
        rate.sleep()
    print("Finishing Program and Saving Route File")
    pubCCMD.publish(0)
    file2.close()
    file1.close()
if __name__ == '__main__':
	try:
		MAIN()
	except (KeyboardInterrupt,rospy.ROSInterruptException):
		print("\nEnding execution")
#Equation for matches
#y = -0.8682281 - (-0.02331669/0.01221943)*(1 - e^(-0.01221943*x))