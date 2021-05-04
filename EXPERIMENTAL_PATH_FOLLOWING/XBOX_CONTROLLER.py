#XBOX INTERFACE WITH ROS
from inputs import devices
import time
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32

#LIST OF AVAILABLE DEVICES

for device in devices:
    print(device)

#THIS VARIABLE REPRESENTS THE STEERING OF THE ROBOTIC PLATFORM
ABS_X_VALUE = 0
ABS_HAT0Y   = 0
X_BUTTON = 0
Y_BUTTON = 0
B_BUTTON = 0
A_BUTTON = 0
ABS_RY_BRAKE = 0
ABS_RZ_VALUE = 0
WORKING_CMD = 0
STATUS = ''

BRAKE = 0
BRAKE_LIMIT = 8
SPEED = 0.0
SPEED_LIMIT = 1.5
STEER = 0.0
STEER_LIMIT = 32700

EXECUTION_TIME = 0
ROSPUB_TIME = 0

#SECURITY BUTTON IN CASE OF EMERGENCY JEJE
ABS_Z_VALUE = 0

GLOBAL = globals()
def bound(low, high, value):
    return max(low, min(high, value))
def CMD_CALLBACK(message):
    GLOBAL['WORKING_CMD'] = message.data

rospy.init_node('agv_xbox_control', anonymous=True)
rospy.Subscriber("/my_agv_controller/working_cmd", Int32, CMD_CALLBACK)
pubST = rospy.Publisher('/steer_arduino_cmd', Float32, queue_size=1)
pubSP = rospy.Publisher('/speed_arduino_cmd', Float32, queue_size=1)
pubBK = rospy.Publisher('/brake_arduino_cmd', Int32, queue_size=1)
pubcmd = rospy.Publisher('/my_agv_controller/working_cmd', Int32, queue_size=1)
#,#,1.4,0.5,6,$,
def publish_ros():
    if (GLOBAL['EXECUTION_TIME'] - GLOBAL['ROSPUB_TIME']>0.2):
        if(GLOBAL['WORKING_CMD']==6):
            #pubSP.publish(GLOBAL['SPEED'])
            #pubBK.publish(GLOBAL['BRAKE'])
            pass
        else:
            pubST.publish(GLOBAL['STEER'])
            pubBK.publish(GLOBAL['BRAKE'])
            pubSP.publish(GLOBAL['SPEED'])
            GLOBAL['ROSPUB_TIME'] = GLOBAL['EXECUTION_TIME']

def MAIN_PROGRAM():
    from inputs import get_gamepad
    while GLOBAL['WORKING_CMD'] !=9:
        GLOBAL['EXECUTION_TIME'] = time.time()
        publish_ros()
        try:
            events = get_gamepad()
            for event in events:
                if(GLOBAL['ABS_Z_VALUE'] < 100):
                    GLOBAL['STATUS'] = 'WARNING'
                    GLOBAL['BRAKE'] = GLOBAL['BRAKE_LIMIT']
                    GLOBAL['SPEED'] = 0.0
                    print("EMERGENCY!!")
                if(str(event.code)=='ABS_X'):
                    #ABS_X IS THE RIGHT HAT OF THE XBOX 360 CONTROLLER AND THE STEERING OF THE VEHICLE
                    GLOBAL['ABS_X_VALUE'] = event.state*1.0
                    GLOBAL['STEER'] = GLOBAL['ABS_X_VALUE']/GLOBAL['STEER_LIMIT']*25.0
                    print("STEERING: ")
                    print(GLOBAL['STEER'])

                elif (str(event.code) == 'ABS_HAT0Y'):
                    GLOBAL['ABS_HAT0Y'] = event.state

                    if(GLOBAL['STATUS']=='STARTING'):
                        if(GLOBAL['ABS_HAT0Y']==-1):
                            GLOBAL['SPEED'] = bound(0,GLOBAL['SPEED_LIMIT'],GLOBAL['SPEED']+0.5)
                        elif(GLOBAL['ABS_HAT0Y']==1):
                            GLOBAL['SPEED'] = bound(0,GLOBAL['SPEED_LIMIT'],GLOBAL['SPEED']-0.5)
                    
                    elif(GLOBAL['STATUS']=='WARNING'):
                        GLOBAL['SPEED'] = 0

                    print("VEHICLE SPEED: " +str(GLOBAL['SPEED']))

                elif (str(event.code) == 'ABS_HAT0X'):
                    GLOBAL['ABS_HAT0Y'] = event.state
                    print (GLOBAL['ABS_HAT0Y'])
                    print("Flecha de izquierda - derecha")

                elif (str(event.code) == 'BTN_NORTH'):
                    if(event.state == 1):
                        GLOBAL['X_BUTTON'] = 1
                    else:
                        GLOBAL['X_BUTTON'] = 0
                    print("X_BUTTON " + str(GLOBAL['X_BUTTON']))

                elif (str(event.code) == 'BTN_WEST'):
                    if(event.state == 1):
                        GLOBAL['Y_BUTTON'] = 1
                    else:
                        GLOBAL['Y_BUTTON'] = 0

                    if(GLOBAL['STATUS'] == 'STARTING' and GLOBAL['Y_BUTTON'] == 1):
                        print("SUBIENDO LA RAMPA PAPS")
                        GLOBAL['SPEED'] =2

                    print("Y_BUTTON " + str(GLOBAL['Y_BUTTON']))

                elif (str(event.code) == 'BTN_EAST'):
                    if(event.state == 1):
                        GLOBAL['B_BUTTON'] = 1
                    else:
                        GLOBAL['B_BUTTON'] = 0
                    print("B_BUTTON " + str(GLOBAL['B_BUTTON']))
                    GLOBAL['STATUS'] = 'WARNING'
                    GLOBAL['SPEED'] = 0.0
                    pubcmd.publish(0)

                elif (str(event.code) == 'BTN_SOUTH'):
                    #A BUTTON RELEASES EMERGENCY STATUS
                    if(event.state == 1):
                        GLOBAL['A_BUTTON'] = 1
                    else:
                        GLOBAL['A_BUTTON'] = 0
                    print("A_BUTTON " + str(GLOBAL['A_BUTTON']))
                    GLOBAL['STATUS'] = 'OK'

                elif(str(event.code)=='ABS_Z'):
                    #EMERGENCY TRIGGER
                    GLOBAL['ABS_Z_VALUE'] = event.state
                    if(GLOBAL['ABS_Z_VALUE'] < 100):
                        GLOBAL['STATUS'] = 'WARNING'
                        GLOBAL['SPEED'] = 0.0

                elif(str(event.code) == 'ABS_RY'):
                    if(GLOBAL['STATUS'] == 'BRAKING'):
                        GLOBAL['ABS_RY_BRAKE'] = event.state
                        #32700 full brake
                        #20000 full brake-1
                        #10000 full brake-2
                        #5000 full brake-3
                        if(GLOBAL['ABS_RY_BRAKE']>=32700):
                            GLOBAL['BRAKE'] = GLOBAL['BRAKE_LIMIT']
                        elif(GLOBAL['ABS_RY_BRAKE']<32700 and GLOBAL['ABS_RY_BRAKE']>20000):
                            GLOBAL['BRAKE'] = GLOBAL['BRAKE_LIMIT']-1
                        elif(GLOBAL['ABS_RY_BRAKE']<20000 and GLOBAL['ABS_RY_BRAKE']>10000):
                            GLOBAL['BRAKE'] = GLOBAL['BRAKE_LIMIT']-2
                        else:
                            GLOBAL['BRAKE'] = GLOBAL['BRAKE_LIMIT']-3
                        print("BRAKING: " +str(GLOBAL['BRAKE']))

                elif(str(event.code) == 'ABS_RZ'):
                    GLOBAL['ABS_RZ_VALUE'] = event.state
                    print("GATILLO IZQ: "+str(GLOBAL['ABS_RZ_VALUE']))

                elif(str(event.code)=='BTN_SELECT'):
                    if(event.state == 1):
                        #IF THERE IS NO PROBLEM, THEN YOU CAN CHANGE BRAKE POSITION
                        if(GLOBAL['STATUS'] != 'WARNING'):
                            GLOBAL['STATUS'] = 'BRAKING'

                elif(str(event.code)=='BTN_START'):
                    if(event.state == 1):
                        #IF THERE IS NO PROBLEM, THEN YOU CAN START SPEED CONTROLLER AS YOU WISH
                        if(GLOBAL['STATUS'] != 'WARNING'):
                            GLOBAL['STATUS'] = 'STARTING'
                            GLOBAL['BRAKE'] = 0
                else:
                    pass
                    #print(str(event.code) + " " + str(str(event.state)))
                if(GLOBAL['ABS_Z_VALUE'] < 100):
                    GLOBAL['STATUS'] = 'WARNING'
                    GLOBAL['SPEED'] = 0.0
                    print("EMERGENCY!!")
                if(GLOBAL['STATUS'] == 'WARNING'):
                    GLOBAL['BRAKE'] = GLOBAL['BRAKE_LIMIT']
            #time.sleep(0.25)
        except:
            print("XBOX CONTROLLER DISCONNECTED!!!! :o OMG")
            GLOBAL['SPEED'] = 0.0
            GLOBAL['BRAKE'] = GLOBAL['BRAKE_LIMIT']

            #time.sleep(0.2)
    print("MAIN PROGRAM WAS ENDED THROUGH ROS TOPIC \n/my_agv_controller/working_cmd")
    
if __name__ == '__main__':
    try:
        MAIN_PROGRAM()
    except rospy.ROSInterruptException:
        pass
