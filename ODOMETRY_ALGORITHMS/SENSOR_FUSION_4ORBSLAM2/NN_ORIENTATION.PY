import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import numpy as np
import csv
import math
from numpy import savetxt

Header = []
RotationData = []
NEURONS = 11
PI = 3.141592653589793238
class myCallback(tf.keras.callbacks.Callback):
  def on_epoch_end(self, epoch, logs={}):
    if(logs.get('loss')<0.0001):
      print("\nReached 99% accuracy so cancelling training!")
      self.model.stop_training = True

callbacks = myCallback()

def positive_angles(angle):
    while(angle < PI):
        angle = angle + 2*PI
    while(angle > PI):
        angle = angle - 2*PI
    return angle

line_count = 0
with open('Orientation28082020_notpositive.txt') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        if line_count == 0:
            Header = row
        else:
          floatRow = [math.sin(float(row[0])),math.sin(positive_angles(float(row[1])-4.71239)),math.sin(positive_angles(-1*float(row[2])+PI/2)),math.sin(positive_angles(-1*float(row[3])+PI/2)),math.sin(positive_angles(-1*float(row[4])+PI/2)),math.sin(positive_angles(-1*float(row[5])+PI/2))]
          RotationData.append(floatRow)
        line_count += 1
    print(line_count)
np.savetxt("RotacionCorregida28082020_notpositive.csv",RotationData,delimiter=",")
npRotationData = np.array(RotationData)
print(npRotationData)
TrainingInput = npRotationData[:,1:6]
TrainingOutput = npRotationData[:,0]

model = keras.models.Sequential([keras.layers.Dense(units = 5,input_shape =[5],activation='relu'),keras.layers.Dense(NEURONS,activation='relu'),keras.layers.Dense(NEURONS-int(NEURONS/2),activation='relu'),keras.layers.Dense(1)])
model.compile(optimizer='RMSprop',loss='mean_squared_error')
model.fit(TrainingInput,TrainingOutput,epochs=10000,shuffle=True,callbacks=[callbacks],batch_size=200)#
model.save("rotation_models/model_rotation_"+str(NEURONS)+"")
Prediction = model.predict(TrainingInput)

Error = np.zeros(Prediction.shape[0])
for index in range (0,Prediction.shape[0]):
  Error[index] = math.sqrt((Prediction[index]-TrainingOutput[index])*(Prediction[index]-TrainingOutput[index]))
print("["+str(NEURONS)+ "] ," + str(np.average(Error)) + "," + str(math.sqrt(np.var(Error)))+"\n")
savetxt('errorR.csv',Error,delimiter=',')