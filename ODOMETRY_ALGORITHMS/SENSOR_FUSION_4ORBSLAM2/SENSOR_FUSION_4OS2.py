import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

import numpy as np
import math
import rospy

from geometry_msgs.msg import PoseStamped
from random import seed
from random import gauss
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

ReadyFlagGazebo = 0
ReadyFlagORBSLAM2 = 0
SimTime = 0
model_x = tf.keras.models.load_model('models/model_sensor_fusion_x_4OS_11')
model_y = tf.keras.models.load_model('models/model_sensor_fusion_y_4OS_11')

#USEFUL CONSTANTS
#Camera Rotation Angles
theta_offset_front = 0+np.radians(270)
theta_offset_rear = 3.14159+np.radians(270)
theta_offset_right = 0.523596+np.radians(270)
theta_offset_left = 5.75958+np.radians(270)
#Camera Translation offset
offset_front_x = -0.15
offset_front_y =  0
offset_right_x = -0.15
offset_right_y = +0.16
offset_left_x = -0.15
offset_left_y = -0.16
offset_rear_x = +0.15
offset_rear_y =  0
#Pi value
PI = 3.14159265358979323846264338327950288
#SYNCHRONIZATION FLAGS
ready_gazebo = 0
ready_front = 0
ready_rear = 0
ready_right = 0
ready_left = 0
ready_front_q = 0
ready_rear_q = 0
ready_right_q = 0
ready_left_q = 0
ready_imu = 0
times = 0
SteerAngle = 0
#IMU Data
XaccA = 0
YaccA = 0
ZaccA = 0
XaccP = 0
YaccP = 0
ZaccP = 0
IMUX = 0
IMUY = 0
simulation_time = 0
SimTimeP = 0
yaw_imu = 0

#ORB SLAM 2 position estimate 1
x_frontal = 0
y_frontal = 0
yaw_frontal = 0
quality_measurement_front=0
#ORB SLAM 2 position estimate 2
x_rear = 0
y_rear = 0
yaw_rear = 0
quality_measurement_rear=0
#ORB SLAM 2 position estimate 3
x_right = 0
y_right = 0
yaw_right = 0
quality_measurement_right=0
#ORB SLAM 2 position estimate 4
x_left = 0
y_left = 0
yaw_left = 0
quality_measurement_left=0

#Real Position
x_real = 0
y_real = 0
yaw_real = 0

listOfGlobals = globals()
def callback_quality_left(message):
	listOfGlobals['quality_measurement_left'] = message.data
	listOfGlobals['ready_left_q'] = 1
def callback_quality_right(message):
	listOfGlobals['quality_measurement_right'] = message.data
	listOfGlobals['ready_right_q'] = 1
def callback_quality_rear(message):
	listOfGlobals['quality_measurement_rear'] = message.data
	listOfGlobals['ready_rear_q'] = 1
def callback_quality_front(message):
	listOfGlobals['quality_measurement_front'] = message.data
	listOfGlobals['ready_front_q'] = 1

def callback_imu(message):
	listOfGlobals['ready_imu'] = 1
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
	listOfGlobals['ready_left'] = 1
	X= message.pose.position.x 
	Y= message.pose.position.y
	Xc = X*np.cos(theta_offset_left)-Y*np.sin(theta_offset_left) + offset_left_x
	Yc = (X*np.sin(theta_offset_left)+Y*np.cos(theta_offset_left)) + offset_left_y
	listOfGlobals['x_left'] = round(Xc, 4)
	listOfGlobals['y_left'] = round(Yc, 4)
	YawTemp = math.radians(message.pose.orientation.y)
	listOfGlobals['yaw_left'] = (YawTemp)

#ORB SLAM 2 ON THE THIRD CAMERA CALLBACK
def callback_right(message):
	#print("Ready4")
	listOfGlobals['ready_right'] = 1
	X= message.pose.position.x 
	Y= message.pose.position.y  
	Xc = X*np.cos(theta_offset_right)-Y*np.sin(theta_offset_right) + offset_right_x
	Yc = (X*np.sin(theta_offset_right)+Y*np.cos(theta_offset_right)) + offset_right_y
	listOfGlobals['x_right'] = round(Xc, 4)
	listOfGlobals['y_right'] = round(Yc, 4)
	YawTemp = math.radians(message.pose.orientation.y)
	listOfGlobals['yaw_right'] = (YawTemp)

#ORB SLAM 2 ON THE SECOND CAMERA CALLBACK
def callback_rear(message):
	#print("Ready3")
	listOfGlobals['ready_rear'] = 1
	X= message.pose.position.x 
	Y= message.pose.position.y  
	Xc = X*np.cos(theta_offset_rear)-Y*np.sin(theta_offset_rear) + offset_rear_x
	Yc = (X*np.sin(theta_offset_rear)+Y*np.cos(theta_offset_rear)) + offset_rear_y
	listOfGlobals['x_rear'] = round(Xc, 4)
	listOfGlobals['y_rear'] = round(Yc, 4)
	YawTemp = math.radians(message.pose.orientation.y)
	listOfGlobals['yaw_rear'] = (YawTemp)

#ORB SLAM 2 ON THE FIRST CAMERA CALLBACK
def callback_front(message):
	#print("Ready2")
	listOfGlobals['ready_front'] = 1
	X= message.pose.position.x 
	Y= message.pose.position.y  
	Xc = X*np.cos(theta_offset_front)-Y*np.sin(theta_offset_front) + offset_front_x
	Yc = (X*np.sin(theta_offset_front)+Y*np.cos(theta_offset_front)) + offset_front_y
	listOfGlobals['x_frontal'] = round(Xc, 4)
	listOfGlobals['y_frontal'] = round(Yc, 4)
	YawTemp = math.radians(message.pose.orientation.y)
	listOfGlobals['yaw_frontal'] = (YawTemp)

#REAL POSITION AND ORIENTATION (DIRECT FROM GAZEBO)
def callback_gazebo(message):
	listOfGlobals['ready_gazebo'] = 1
	listOfGlobals['x_real'] = message.data[0]
	listOfGlobals['y_real'] = message.data[1]
	listOfGlobals['simulation_time']   = message.data[4]
	# -1.571 para coincidir con el marco de referencia de Gazebo
	listOfGlobals['yaw_real'] = (message.data[5]-PI/2)
	listOfGlobals['x_real'] = round(listOfGlobals['x_real'],4)
	listOfGlobals['y_real'] = round(listOfGlobals['y_real'],4)   

def main_function():
	print("\n\n\nStarting ROS Interface")
	rospy.init_node('agv_control', anonymous=1)
	rospy.Subscriber("/my_agv/status_pub", Float32MultiArray, callback_gazebo)
	pub = rospy.Publisher('/my_agv/status_cmd', Int32, queue_size=1)
	rospy.Subscriber("/ORB_SLAM2_1/pose", PoseStamped, callback_front)
	rospy.Subscriber("/ORB_SLAM2_2/pose", PoseStamped, callback_rear)
	rospy.Subscriber("/ORB_SLAM2_3/pose", PoseStamped, callback_right)
	rospy.Subscriber("/ORB_SLAM2_4/pose", PoseStamped, callback_left)
	rospy.Subscriber("ORB_SLAM2_1/quality",Int32,callback_quality_front)
	rospy.Subscriber("ORB_SLAM2_2/quality",Int32,callback_quality_rear)
	rospy.Subscriber("ORB_SLAM2_3/quality",Int32,callback_quality_right)
	rospy.Subscriber("ORB_SLAM2_4/quality",Int32,callback_quality_left)
	rospy.Subscriber("/my_agv/imu", Imu, callback_imu)
	rate = rospy.Rate(100)

	print("Starting ROS Main Loop")
	while not rospy.is_shutdown():
		pub.publish(0)

		if(listOfGlobals['ready_left_q']==1 and listOfGlobals['ready_right_q']==1 and listOfGlobals['ready_rear_q']==1 and listOfGlobals['ready_front_q']==1):
			ready_left_q = 0
			ready_right_q = 0
			ready_rear_q = 0
			ready_front_q = 0
			print(str(listOfGlobals['quality_measurement_front']) +","+ str(listOfGlobals['quality_measurement_rear']) +","+ str(listOfGlobals['quality_measurement_right']) +","+ str(listOfGlobals['quality_measurement_left']) )
			print("Quality front = "+str(listOfGlobals['quality_measurement_front']))
			print("Quality rear = "+str(listOfGlobals['quality_measurement_rear']))
			print("Quality right = "+str(listOfGlobals['quality_measurement_right']))
			print("Quality left = "+str(listOfGlobals['quality_measurement_left']))
		if(listOfGlobals['ready_gazebo']==1 and listOfGlobals['ready_front']==1 and listOfGlobals['ready_rear']==1 and listOfGlobals['ready_right']==1 and listOfGlobals['ready_left']==1 and listOfGlobals['ready_imu']==1 ):
			x_input = np.array([[x_frontal,x_rear,x_right,x_left]])
			x_prediction=model_x.predict(x_input)
			y_input = np.array([[y_frontal,y_rear,y_right,y_left]])
			y_prediction=model_y.predict(y_input)
			Error = math.sqrt((x_prediction[0,0]-x_real)*(x_prediction[0,0]-x_real) + (y_prediction[0,0]-y_real)*(y_prediction[0,0]-y_real))
			#print("Location Error = "+str(Error))
			listOfGlobals['ready_gazebo'] = 0
			listOfGlobals['ready_front'] = 0
			listOfGlobals['ready_rear'] = 0
			listOfGlobals['ready_right'] = 0
			listOfGlobals['ready_left'] = 0
			listOfGlobals['ready_imu'] = 0
		rate.sleep()
	print("Finishing The Main AGV Program")
if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
    	print("Finishing the sensor fusion system")