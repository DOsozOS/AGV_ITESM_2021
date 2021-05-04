import serial, time
import rospy
import statistics,math
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int32

ser = serial.Serial('/dev/ttyACM1',115200)  # open serial port
STEER_ANGLE_CMD = 0.0
LINEAR_SPEED = 0.0
STEER_SENSOR = 0.0
LINEAR_SPEED_SENSOR = 0.0
NEW_UPCOMING_SERIAL_DATA = 0.0
TIME_PUBLISH = time.time()
ACTUAL_TIME = time.time()
BRAKE_SETPOINT = 0
VALID_DATA = False
GLOBAL = globals()

def STEERING_CALLBACK(message):
    GLOBAL['STEER_ANGLE_CMD'] = message.data
def SPEED_CALLBACK(message):
    GLOBAL['LINEAR_SPEED'] = message.data
def BRAKE_CALLBACK(message):
    GLOBAL['BRAKE_SETPOINT'] = int(message.data)

def PUBLISH_CMD():
	if(GLOBAL['ACTUAL_TIME']-GLOBAL['TIME_PUBLISH']>0.33):
		ser.write(b',#,'+str(GLOBAL['LINEAR_SPEED'])+',' + str(GLOBAL['STEER_ANGLE_CMD'])+',' + str(GLOBAL['BRAKE_SETPOINT']) + ',$,\n')
		GLOBAL['TIME_PUBLISH'] = GLOBAL['ACTUAL_TIME']
def main_function():
	rospy.init_node('real_agv', anonymous=1)
	rospy.Subscriber("/steer_arduino_cmd",Float32,STEERING_CALLBACK)
	rospy.Subscriber("/speed_arduino_cmd",Float32,SPEED_CALLBACK)
	rospy.Subscriber("/brake_arduino_cmd",Int32,BRAKE_CALLBACK)

	pubSt = rospy.Publisher('/steering_sensor', Float32, queue_size=0)
	pubSp = rospy.Publisher('/rear_speed_sensor', Float32, queue_size=0)
	time.sleep(1)
	print("-----------------Publishing to the ros topics-----------------\n/steering_sensor\n/speed_arduino_cmd\n\n")
	print("-----------------Subscribed to the ros topics-----------------\n/steer_arduino_cmd\n/speed_arduino_cmd\n\n")
	print("---------------------Communication status---------------------\n\n")
	while not (rospy.is_shutdown()): #and ser.is_open):
		try:
			GLOBAL['ACTUAL_TIME'] = time.time()
			PUBLISH_CMD()
			rawString = ser.readline()
			rawData = rawString.split(',')
			Lenght = len(rawData)
			#print(rawString)
			tmp_valid_data = False
			if(Lenght>3):
				if(rawData[1]=='#' and rawData[4]=="$"):
					GLOBAL['LINEAR_SPEED_SENSOR'] = (float(rawData[2]))
					#print("Speed: "+str(GLOBAL['LINEAR_SPEED_SENSOR']))
					GLOBAL['STEER_SENSOR'] = (float(rawData[3]))
					#print("Steer: "+str(GLOBAL['STEER_SENSOR']))
					pubSp.publish(GLOBAL['LINEAR_SPEED_SENSOR'])
					pubSt.publish(GLOBAL['STEER_SENSOR'])
					tmp_valid_data = True
				if(GLOBAL['VALID_DATA']== True and tmp_valid_data == False):
					print("Invalid agv data string")
				elif(GLOBAL['VALID_DATA']== False and tmp_valid_data == True):
					print("Valid agv data string")
				GLOBAL['VALID_DATA'] = tmp_valid_data
					

		except IndexError:
			print("Array index out of range")
		except ValueError:
			print("Invalid sensor data")
		except serial.serialutil.SerialException:
			print("Serial reading error, check device connection")
			break
	print("Closing serial connection")
	ser.close()
	
if __name__ == '__main__':
	try:
		main_function()
	except (KeyboardInterrupt):
		print("Exiting the Arduino ROS Program")
		ser.close()
	except (rospy.ROSInterruptException):
		pass
