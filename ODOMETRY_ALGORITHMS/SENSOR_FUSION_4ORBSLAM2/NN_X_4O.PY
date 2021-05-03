import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import numpy as np
import csv
import math
from numpy import savetxt

#Importing Data obtained from GAZEBO simulations
#0       1    2    3        4        5     6
#SimTime,Xgps,Ygps,Xorbslam,Yorbslam,Xreal,Yreal
Header = []
DataX   = []
DataY   = []
line_count = 0
NEURONS = 11
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

#file2 = open("TestInputSensorData2.txt","w");

class myCallback(tf.keras.callbacks.Callback):
  def on_epoch_end(self, epoch, logs={}):
    if(logs.get('loss')<0.0020):
      print("\nReached 0.002 of Loss so cancelling training!")
      self.model.stop_training = True

callbacks = myCallback()

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

npCorrectedX = np.transpose(np.array([npDataX[:,0]*np.cos(theta1) - npDataY[:,0]*np.sin(theta1)+offset1x,npDataX[:,1]*np.cos(theta2) - npDataY[:,1]*np.sin(theta2)+offset2x, npDataX[:,2]*np.cos(theta3) - npDataY[:,2]*np.sin(theta3)+offset3x,npDataX[:,3]*np.cos(theta4) - npDataY[:,3]*np.sin(theta4)+offset4x,npDataX[:,4]]))
TrainingInput  = npCorrectedX[:,0:4]
TrainingOutput = npCorrectedX[:,4:5]
TestInput  = npCorrectedX[:,0:4]
TestOutput = npCorrectedX[:,4:5]
print("Single Coordinate Training")

model = keras.models.Sequential([keras.layers.Dense(units = 4,input_shape =[4],activation='relu'),keras.layers.Dense(NEURONS,activation='relu'),keras.layers.Dense(4,activation='relu'),keras.layers.Dense(1)])
model.compile(optimizer='RMSprop',loss='mean_squared_error')
model.fit(TrainingInput,TrainingOutput,epochs=100,shuffle=True,callbacks=[callbacks],batch_size=25)#
model.save("models/model_sensor_fusion_x_4OS_"+str(NEURONS)+"")
Prediction = model.predict(TestInput)
Error = np.zeros(Prediction.shape[0])
for index in range (0,Prediction.shape[0]):
  Error[index] = math.sqrt((Prediction[index,0]-TestOutput[index,0])*(Prediction[index,0]-TestOutput[index,0]))
print("["+str(NEURONS)+ "] ," + str(np.average(Error)) + "," + str(math.sqrt(np.var(Error)))+"\n")
