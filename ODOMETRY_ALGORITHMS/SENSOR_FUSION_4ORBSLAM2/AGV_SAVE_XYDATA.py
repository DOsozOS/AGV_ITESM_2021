#import tensorflow as tf
import numpy as np
import math
import rospy

from geometry_msgs.msg import PoseStamped
#from tensorflow import keras
#from tensorflow.keras import layers
from random import seed
from random import gauss
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

ReadyFlagGazebo = 0
ReadyFlagORBSLAM2 = 0
SimTime = 0
#model_x = tf.keras.models.load_model('modelsx/model_sensor_fusion_x_4OS_11')
#model_y = tf.keras.models.load_model('modelsx/model_sensor_fusion_y_4OS_11')

listOfGlobals = globals()
#USEFUL CONSTANTS
#Camera Rotation Angles
theta1 = 0+np.radians(270)
theta2 = 3.14159+np.radians(270)
theta3 = 0.523596+np.radians(270)
theta4 = 5.75958+np.radians(270)
#Camera Translation offset
offset1x = -0.15
offset1y =  0
offset3x = -0.15
offset3y = +0.16
offset4x = -0.15
offset4y = -0.16
offset2x = +0.15
offset2y =  0
#Pi value
PI = 3.14159265358979323846264338327950288
#SYNCHRONIZATION FLAGS
ReadyFlag_1 = 0
ReadyFlag_2 = 0
ReadyFlag_3 = 0
ReadyFlag_4 = 0
ReadyFlag_5 = 0
ReadyFlag_6 = 0
times = 0
SteerAngle = 0
file1 = open("validate_position.txt","w")
file2 = open("validate_rotation.txt","w")

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
yaw_imu = 0

#ORB SLAM 2 position estimate 1
x_frontal = 0
y_frontal = 0
yaw_frontal = 0
#ORB SLAM 2 position estimate 2
x_rear = 0
y_rear = 0
yaw_rear = 0
#ORB SLAM 2 position estimate 3
x_right = 0
y_right = 0
yaw_right = 0
#ORB SLAM 2 position estimate 4
x_left = 0
y_left = 0
yaw_left = 0

#Real Position
XR = 0
YR = 0
YawR = 0

def callback_imu(message):
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
	listOfGlobals['yaw_imu'] = (math.atan2(t3,t4))

#ORB SLAM 2 ON THE FOURTH CAMERA CALLBACK
def callback_left(message):
	#print("Ready5")
	listOfGlobals['ReadyFlag_5'] = 1
	X= message.pose.position.x 
	Y= message.pose.position.y
	listOfGlobals['x_left'] = round(X, 4)
	listOfGlobals['y_left'] = round(Y, 4)
	YawTemp = math.radians(message.pose.orientation.y)
	listOfGlobals['yaw_left'] = (YawTemp)

#ORB SLAM 2 ON THE THIRD CAMERA CALLBACK
def callback_right(message):
	#print("Ready4")
	listOfGlobals['ReadyFlag_4'] = 1
	X= message.pose.position.x 
	Y= message.pose.position.y
	listOfGlobals['x_right'] = round(X, 4)
	listOfGlobals['y_right'] = round(Y, 4)
	YawTemp = math.radians(message.pose.orientation.y)
	listOfGlobals['yaw_right'] = (YawTemp)

#ORB SLAM 2 ON THE SECOND CAMERA CALLBACK
def callback_rear(message):
	#print("Ready3")
	listOfGlobals['ReadyFlag_3'] = 1
	X= message.pose.position.x 
	Y= message.pose.position.y  
	listOfGlobals['x_rear'] = round(X, 4)
	listOfGlobals['y_rear'] = round(Y, 4)
	YawTemp = math.radians(message.pose.orientation.y)
	listOfGlobals['yaw_rear'] = (YawTemp)

#ORB SLAM 2 ON THE FIRST CAMERA CALLBACK
def callback_front(message):
	#print("Ready2")
	listOfGlobals['ReadyFlag_2'] = 1
	X= message.pose.position.x 
	Y= message.pose.position.y  
	listOfGlobals['x_frontal'] = round(X, 4)
	listOfGlobals['y_frontal'] = round(Y, 4)
	YawTemp = math.radians(message.pose.orientation.y)
	listOfGlobals['yaw_frontal'] = (YawTemp)

#REAL POSITION AND ORIENTATION (DIRECT FROM GAZEBO)
def chatter_callback(message):
	listOfGlobals['ReadyFlag_1'] = 1
	listOfGlobals['XR'] = message.data[0]
	listOfGlobals['YR'] = message.data[1]
	listOfGlobals['SimTimeA']   = message.data[4]
	# -1.571 para coincidir con el marco de referencia de Gazebo
	listOfGlobals['YawR'] = (message.data[5]-PI/2)

	listOfGlobals['XR'] = round(listOfGlobals['XR'],4)
	listOfGlobals['YR'] = round(listOfGlobals['YR'],4)   

def main_function():
	print("\n\n\nStarting ROS Interface")
	rospy.init_node('agv_control', anonymous=1)
	rospy.Subscriber("/my_agv/status_pub", Float32MultiArray, chatter_callback)
	pub = rospy.Publisher('/my_agv/status_cmd', Int32, queue_size=1)
	rospy.Subscriber("/ORB_SLAM2_1/pose", PoseStamped, callback_front)
	rospy.Subscriber("/ORB_SLAM2_2/pose", PoseStamped, callback_rear)
	rospy.Subscriber("/ORB_SLAM2_3/pose", PoseStamped, callback_right)
	rospy.Subscriber("/ORB_SLAM2_4/pose", PoseStamped, callback_left)
	rospy.Subscriber("/my_agv/imu", Imu, callback_imu)
	rate = rospy.Rate(100)
	print("ROS Interface Started")

	print("Starting ROS Main Loop")
	file1.write("SimTime,x_frontal,y_frontal,x_rear,y_rear,x_right,y_right,x_left,y_left,XR,YR\n")
	file2.write("SimTime,yaw_frontal,yaw_rear,yaw_right,yaw_left,yaw_imu")
	while not rospy.is_shutdown():
		pub.publish(0)
		if(listOfGlobals['ReadyFlag_1']==1 and listOfGlobals['ReadyFlag_2']==1 and listOfGlobals['ReadyFlag_3']==1 and listOfGlobals['ReadyFlag_4']==1 and listOfGlobals['ReadyFlag_5']==1 and listOfGlobals['ReadyFlag_6']==1 ):
			predictInputX = np.array([[x_frontal,x_rear,x_right,x_left]])
			predictInputY = np.array([[y_frontal,y_rear,y_right,y_left]])
			print(str(SimTimeA)+","+str(x_frontal)+","+str(y_frontal)+","+str(x_rear)+","+str(y_rear)+","+str(x_right)+","+str(y_right)+","+str(x_left)+","+str(y_left)+","+str(XR)+","+str(YR))
			file1.write(str(SimTimeA)+","+str(x_frontal)+","+str(y_frontal)+","+str(x_rear)+","+str(y_rear)+","+str(x_right)+","+str(y_right)+","+str(x_left)+","+str(y_left)+","+str(XR)+","+str(YR)+"\n")
			file2.write(str(SimTimeA)+","+str(yaw_frontal)+","+str(yaw_rear)+","+str(yaw_right)+","+str(yaw_left)+","+str(yaw_imu)+"\n")
			listOfGlobals['ReadyFlag_1'] = 0
			listOfGlobals['ReadyFlag_2'] = 0
			listOfGlobals['ReadyFlag_3'] = 0
			listOfGlobals['ReadyFlag_4'] = 0
			listOfGlobals['ReadyFlag_5'] = 0
			listOfGlobals['ReadyFlag_6'] = 0
		rate.sleep()
	print("Finishing The Main AGV Program")
	file1.close()
	file2.close()
if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
    	file1.close()
    	file2.close()