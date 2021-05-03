#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import math

from random import seed
from random import gauss

plt.ion()
PI = 3.14159265358979323846264338327950288
ReadyFlag_1 = 0
ReadyFlag_2 = 0
ReadyFlag_3 = 0
ReadyFlag_4 = 0
ReadyFlag_5 = 0
ReadyFlag_6 = 0
times = 0
SteerAngle = 0
file1 = open("Orientation17082020.txt","w")
#IMU Data
XaccA = 0
YaccA = 0
ZaccA = 0
XaccP = 0
YaccP = 0
ZaccP = 0
IMUX = 0
IMUY = 0
SimTimeA = 0
SimTimeP = 0
YawIMU = 0
#GPS position estimate
X1 = 0
Y1 = 0
#ORB SLAM 2 position estimate 1
X2 = 0
Y2 = 0
Yaw2 = 0
#ORB SLAM 2 position estimate 2
X3 = 0
Y3 = 0
Yaw3 = 0
#ORB SLAM 2 position estimate 3
X4 = 0
Y4 = 0
Yaw4 = 0
#ORB SLAM 2 position estimate 4
X5 = 0
Y5 = 0
Yaw5 = 0
#Real Position
XR = 0
YR = 0
YawR = 0

listOfGlobals = globals()
def positive_angles(angle):
    while(angle < 0):
        angle = angle + 2*PI
    while(angle > 2*PI):
        angle = angle - 2*PI
    return angle

def chatter_callback6(message):
    listOfGlobals['ReadyFlag_6'] = 1
    listOfGlobals['XaccA'] = message.linear_acceleration.x
    listOfGlobals['YaccA'] = message.linear_acceleration.y
    listOfGlobals['ZaccA'] = message.linear_acceleration.z
    q0 = message.orientation.x
    q1 = message.orientation.y
    q2 = message.orientation.z
    q3 = message.orientation.w
    t3 = +2.0*(q3*q2+q0*q1)
    t4 = +1.0 - 2.0*(q1*q1+q2*q2)
    listOfGlobals['YawIMU'] = positive_angles(math.atan2(t3,t4))
def chatter_callback5(message):
    #print("Ready5")
    listOfGlobals['ReadyFlag_5'] = 1
    listOfGlobals['X5'] = message.pose.position.x 
    listOfGlobals['Y5'] = message.pose.position.y  
    listOfGlobals['X5'] = round(listOfGlobals['X5'], 4)
    listOfGlobals['Y5'] = round(listOfGlobals['Y5'], 4)
    YawTemp = math.radians(message.pose.orientation.y)
    listOfGlobals['Yaw5'] = positive_angles(YawTemp)
def chatter_callback4(message):
    #print("Ready4")
    listOfGlobals['ReadyFlag_4'] = 1
    listOfGlobals['X4'] = message.pose.position.x 
    listOfGlobals['Y4'] = message.pose.position.y  
    listOfGlobals['X4'] = round(listOfGlobals['X4'], 4)
    listOfGlobals['Y4'] = round(listOfGlobals['Y4'], 4)
    YawTemp = math.radians(message.pose.orientation.y)
    listOfGlobals['Yaw4'] = positive_angles(YawTemp)
def chatter_callback3(message):
    #print("Ready3")
    listOfGlobals['ReadyFlag_3'] = 1
    listOfGlobals['X3'] = message.pose.position.x 
    listOfGlobals['Y3'] = message.pose.position.y  
    listOfGlobals['X3'] = round(listOfGlobals['X3'], 4)
    listOfGlobals['Y3'] = round(listOfGlobals['Y3'], 4)
    YawTemp = math.radians(message.pose.orientation.y)
    listOfGlobals['Yaw3'] = positive_angles(YawTemp)
def chatter_callback2(message):
    #print("Ready2")
    listOfGlobals['ReadyFlag_2'] = 1
    listOfGlobals['X2'] = message.pose.position.x 
    listOfGlobals['Y2'] = message.pose.position.y  
    listOfGlobals['X2'] = round(listOfGlobals['X2'], 4)
    listOfGlobals['Y2'] = round(listOfGlobals['Y2'], 4)
    YawTemp = math.radians(message.pose.orientation.y)
    listOfGlobals['Yaw2'] = positive_angles(YawTemp)
def chatter_callback(message):
    #print("Ready1")
    #Generating Noisy GPS position values within -+5 meters
    listOfGlobals['ReadyFlag_1'] = 1
    listOfGlobals['XR'] = message.data[0]
    listOfGlobals['YR'] = message.data[1]
    listOfGlobals['X1'] = message.data[0] + gauss(0,1)*1.1
    listOfGlobals['Y1'] = message.data[1] + gauss(0,1)*1.1

    listOfGlobals['SimTimeA']   = message.data[4]
    # -1.571 para coincidir con el marco de referencia de Gazebo
    listOfGlobals['YawR'] = positive_angles(message.data[5]-PI/2)

    listOfGlobals['XR'] = round(listOfGlobals['XR'],4)
    listOfGlobals['YR'] = round(listOfGlobals['YR'],4)   
    listOfGlobals['X1'] = round(listOfGlobals['X1'], 4)
    listOfGlobals['Y1'] = round(listOfGlobals['Y1'], 4)


def talker():
    rospy.init_node('agv_control', anonymous=1)
    rospy.Subscriber("/my_agv/status_pub", Float32MultiArray, chatter_callback)
    pub = rospy.Publisher('/my_agv/status_cmd', Int32, queue_size=1)
    rospy.Subscriber("/ORB_SLAM2_1/pose", PoseStamped, chatter_callback2)
    rospy.Subscriber("/ORB_SLAM2_2/pose", PoseStamped, chatter_callback3)
    rospy.Subscriber("/ORB_SLAM2_3/pose", PoseStamped, chatter_callback4)
    rospy.Subscriber("/ORB_SLAM2_4/pose", PoseStamped, chatter_callback5)
    rospy.Subscriber("/my_agv/imu", Imu, chatter_callback6)
    rate = rospy.Rate(100) 
    i = 0
    file1.write("YawR,YawImu,Yaw2,Yaw3,Yaw4,Yaw5 \n")
    print("Starting Vehicle Monitor")
    while not rospy.is_shutdown(): 
        if(listOfGlobals['ReadyFlag_1']==1 and listOfGlobals['ReadyFlag_2']==1 ):#and listOfGlobals['ReadyFlag_3']==1 and listOfGlobals['ReadyFlag_4']==1 and listOfGlobals['ReadyFlag_5']==1):            
            listOfGlobals['ReadyFlag_1'] = 0
            listOfGlobals['ReadyFlag_2'] = 0
            listOfGlobals['ReadyFlag_3'] = 0
            listOfGlobals['ReadyFlag_4'] = 0
            listOfGlobals['ReadyFlag_5'] = 0
            print(str(YawR) +","+str(YawIMU)+"," + str(Yaw2)+","+str(Yaw3)+","+str(Yaw4)+","+str(Yaw5))
            file1.write(str(YawR) +","+str(YawIMU)+ "," + str(Yaw2)+","+str(Yaw3)+","+str(Yaw4)+","+str(Yaw5)+'\n')
        if(listOfGlobals['ReadyFlag_6']==1):
            DeltaTime = SimTimeA-SimTimeP
            listOfGlobals['XaccP'] = listOfGlobals['XaccA']
            listOfGlobals['YaccP'] = listOfGlobals['YaccA']
            listOfGlobals['ZaccP'] = listOfGlobals['ZaccA']
            listOfGlobals['SimTimeP'] = listOfGlobals['SimTimeA']
        pub.publish(0)
        rate.sleep()
    file1.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
