import numpy as np
import math
import matplotlib.pyplot as plt
import csv

Data = []
Data2 = []
data_line_count = 0
line_count = 0
filename =  'PATH_FOLLOW_TEST_2021-04-27_16:31:47.863192_REAL_H3'
filename2 = 'saved_route_6'
#filename = 'odometry_data_recalc_2021-03-31_19:23:09.081168_SIMULATION_kfgpsyaw2'
#filename = 'odometry_data_recalc_2021-03-31_18:54:31.444188_SIMULATION_DM'
#filename = 'odometry_data_recalc_2021-03-31_19:59:26.070805_SIMULATION'
#filename = 'odometry_data_recalc_2021-03-31_20:09:45.007729_SIMULATION'
#filename = 'odometry_data_recalc_2021-03-31_19:04:37.251831_SIMULATION_yawkf2'
start_appending = False
with open(filename+".csv") as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        if line_count == 0:
            pass
        else:
            if(float(row[7])>0):
                start_appending = True
            if(start_appending):
                floatRow = [
                float(row[0]),#EXECUTION_TIME
                float(row[1]),#X_POS_KF
                float(row[2]),#Y_POS_KF
                float(row[3]),#X_COORDINATE_GPS
                float(row[4]),#Y_COORDINATE_GPS
                float(row[5]),#YAW_IMU
                float(row[6]),#ESTIMATED_YAW
                float(row[7]),#velocity_sensor
                float(row[8]),#,#STEER_ANGLE_AGV
                bool(row[9]),#READY_FLAG_GPS_X
                bool(row[10]),#READY_FLAG_GPS_Y
                int(row[11]),#WORKING_CMD
                float(row[14]),#X_FRONT_ZED_TEMP
                float(row[15]),#Y_FRONT_ZED_TEMP

                float(row[26]),#X_FRONT_ZED_TEMP
                float(row[27])#Y_FRONT_ZED_TEMP
                ]
                Data.append(floatRow)
                data_line_count += 1
        line_count = line_count+1
    print(data_line_count)
npData = np.array(Data)
data_line_count2 = 0
line_count2 = 0
with open(filename2+".csv") as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        floatRow = [float(row[0]),#EXECUTION_TIME
        float(row[1]),#X_POS_KF
        float(row[2]),#Y_POS_KF
        ]
        Data2.append(floatRow)
        data_line_count2 += 1
        line_count2 = line_count2+1
    print(data_line_count2)
npData2 = np.array(Data2)
colors = np.array(['black','magenta'])

fig = plt.figure(0,figsize=(9,7))
plt.title('Vehicle position estimate (physical test)')
plt.subplot()
KF, = plt.plot(npData[:,1],npData[:,2],color='cyan',label='Vehicle position estimate')
#GPS = plt.scatter(npData[:,3],npData[:,4],color='greenyellow',label='GPS position',s=5)

image_filename = "base_map_itesm.png"
img = plt.imread(image_filename)
plt.imshow(img, extent=[-122.766+58, 22.862+58, -73.482+20, 16.428+20])
#plt.imshow(img, extent=[-122.766, 22.862, -73.482, 16.428])
ax = plt.axes()
arrow_lenght = 1
X = []
Y = []
U = []
V = []
time = 0
for index in range(0,data_line_count2-2,3):
    t1 = (npData2[index,0])
    t2 = (npData2[index,1])
    t3 = (arrow_lenght*math.cos(npData2[index,2]))
    t4 = (arrow_lenght*math.sin(npData2[index,2]))
    X.append(t1)
    Y.append(t2)
    U.append(t3)
    V.append(t4)
    arrow = ax.arrow(t1,t2,t3,t4, head_width=arrow_lenght, head_length=arrow_lenght, ec='red',fc='red',label="Trajectory orientation")
    time = npData[index,0]
npX = np.array(X)
npY = np.array(Y)
npU = np.array(U)
npV = np.array(V)

#arrow
DESIRED_PATH = plt.scatter(npData2[:,0],npData2[:,1],color='blue',label='Desired path',s=2)
ax.legend(handles=[KF,DESIRED_PATH,arrow],loc='upper left',ncol=3, mode="expand", borderaxespad=0.0)

#plt.xlim(-100, -40)
#plt.ylim(-70, -40)
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.grid()

plt.savefig("PLOTS/"+filename+'.eps', dpi=600)
plt.savefig("PLOTS/"+filename+'.png')


plt.show()
