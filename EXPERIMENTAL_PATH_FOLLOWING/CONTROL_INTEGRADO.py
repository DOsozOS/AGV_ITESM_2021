#!/usr/bin/env python
from std_msgs.msg import Int32
import math
import rospy
import matplotlib.pyplot as plt
import numpy as np
import random
import time
import rospy
import Tkinter
import tkMessageBox

rospy.init_node('agv_control', anonymous=1)
pub = rospy.Publisher('/my_agv_controller/working_cmd', Int32, queue_size=1)
rate = rospy.Rate(10) # 10hz

top = Tkinter.Tk()

def button1Callback():
	pub.publish(1)

def button2Callback():
	pub.publish(2)

def button3Callback():
	pub.publish(3)

def button4Callback():
	pub.publish(4)

def button5Callback():
	pub.publish(5)

def button6Callback():
	pub.publish(6)

def button7Callback():
	pub.publish(7)


def button8Callback():
	pub.publish(8)

def button9Callback():
	pub.publish(9)


Button1 = Tkinter.Button(top, text ="Stop any process", command = button1Callback)
Button2 = Tkinter.Button(top, text ="Save a new route", command = button2Callback)
Button3 = Tkinter.Button(top, text ="Follow the saved route", command = button3Callback)
Button4 = Tkinter.Button(top, text ="Configure Starting Point", command = button4Callback)
Button5 = Tkinter.Button(top, text ="End Program", command = button5Callback)
Button6 = Tkinter.Button(top, text ="Follow a Path", command = button6Callback)
Button7= Tkinter.Button(top, text ="Load a Path", command = button7Callback)
Button8 = Tkinter.Button(top, text ="GPS calculation", command = button8Callback)
Button9 = Tkinter.Button(top, text ="END AGV INTERFACE", command = button9Callback)


Button1.pack()
Button2.pack()
Button3.pack()
Button4.pack()
Button6.pack()
Button7.pack()
Button5.pack()
Button8.pack()
Button9.pack()
top.mainloop()
