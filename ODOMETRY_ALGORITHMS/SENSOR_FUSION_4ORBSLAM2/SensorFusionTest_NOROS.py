import tensorflow as tf
import numpy as np
import math
import matplotlib.pyplot as plt
import csv

from geometry_msgs.msg import PoseStamped
from tensorflow import keras
from tensorflow.keras import layers
from random import seed
from random import gauss

file1 = open("error_xmap_25112020.csv","w");
plt.ion()
ReadyFlagGazebo = 0
ReadyFlagORBSLAM2 = 0
SimTime = 0
model_x = tf.keras.models.load_model('models/model_sensor_fusion_x_4OS_11')
model_y = tf.keras.models.load_model('models/model_sensor_fusion_y_4OS_11')
#Camera Rotation Angles
theta1 = -0.523596+np.radians(270)
theta2 = -5.75958+np.radians(270)
theta3 = -2.617993+np.radians(270)
theta4 = -3.66519+np.radians(270)
#Camera Translation offset
offset1x = -0.15
offset1y =  0
offset3x = -0.15
offset3y = +0.16
offset4x = -0.15
offset4y = -0.16
offset2x = +0.15
offset2y =  0

Header = []
DataX   = []
DataY   = []
line_count = 0
NEURONS = 11

with open('validate_position.txt') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        if line_count == 0:
            Header = row
        else:
          floatRow = [float(row[3-2]),float(row[5-2]),float(row[7-2]),float(row[9-2]),float(row[11-2])]
          DataX.append(floatRow)
        line_count += 1
    print(line_count)

line_count = 0
with open('validate_position.txt') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        if line_count == 0:
            Header = row
        else:
          floatRow = [float(row[4-2]),float(row[6-2]),float(row[8-2]),float(row[10-2]),float(row[12-2])]
          DataY.append(floatRow)
        line_count += 1
    print(line_count)


npDataX = np.array(DataX)
npDataY = np.array(DataY)

npCorrectedY = np.transpose(np.array([(npDataX[:,0]*np.sin(theta1) + npDataY[:,0]*np.cos(theta1))+offset1y,(npDataX[:,1]*np.sin(theta2) + npDataY[:,1]*np.cos(theta2))+offset2y,(npDataX[:,2]*np.sin(theta3) + npDataY[:,2]*np.cos(theta3))+offset3y,(npDataX[:,3]*np.sin(theta4) + npDataY[:,3]*np.cos(theta4))+offset4y,npDataY[:,4]]))
TestInputY  = npCorrectedY[:,0:4]
TestOutputY = npCorrectedY[:,4:5]

npCorrectedX = np.transpose(np.array([npDataX[:,0]*np.cos(theta1) - npDataY[:,0]*np.sin(theta1)+offset1x,npDataX[:,1]*np.cos(theta2) - npDataY[:,1]*np.sin(theta2)+offset2x, npDataX[:,2]*np.cos(theta3) - npDataY[:,2]*np.sin(theta3)+offset3x,npDataX[:,3]*np.cos(theta4) - npDataY[:,3]*np.sin(theta4)+offset4x,npDataX[:,4]]))
TestInputX  = npCorrectedX[:,0:4]
TestOutputX = npCorrectedX[:,4:5]

PredictionY = model_y.predict(TestInputY)
PredictionX = model_x.predict(TestInputX)
PredictedError = np.zeros(PredictionX.shape[0]) 

for index in range (0,PredictionX.shape[0]):
  PredictedError[index] = math.sqrt((PredictionX[index,0]-TestOutputX[index,0])*(PredictionX[index,0]-TestOutputX[index,0]) + (PredictionY[index,0]-TestOutputY[index,0])*(PredictionY[index,0]-TestOutputY[index,0]))
  file1.write(str(PredictionX[index,0]) + ","+str(PredictionY[index,0]) +","+str(TestOutputX[index,0])+","+str(TestOutputY[index,0])+ ","+str(PredictedError[index])+'\n')
file1.close()
print("["+str(NEURONS)+ "] ," + str(np.average(PredictedError)) + "," + str(math.sqrt(np.var(PredictedError)))+"\n")
