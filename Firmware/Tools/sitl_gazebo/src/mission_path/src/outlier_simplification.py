import numpy as np
import csv
import math
import matplotlib.pyplot as plt
from utm import utmconv
import cv2
from simplification.cutil import simplify_coords, simplify_coords_vw


time = []
lat = []
lon = []
lat_new = []
lon_new = []
lat_filtered = []
lon_filtered = []
seconds = []
east = []
north = []
zone = []
hemisphere = []
zlet = []
ecluDist_utm = []
timeDiff=[]
velocity = []

#Open file and extract necessary data
with open ('data.txt') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        lat.append(row[0])
        lon.append(row[1])
        time.append(row[6])

#Extract seconds from timestamp
for i in range(len(time)):
    seconds.append(float(time[i][24:29]))

#Remove letters and signs from latitude and longitude data
for i in range(len(lat)):
    lat_new.append(float(lat[i][4:12]))
    lon_new.append(float(lon[i][6:14]))

lat_new = np.asarray(lat_new)
lon_new = np.asarray(lon_new)
seconds = np.asarray(seconds)
convert_utm = utmconv()

for i in range(len(lat_new)):
            [hemi_temp, zone_temp, zlet_temp, east_temp, north_temp] = convert_utm.geodetic_to_utm(lat_new[i], lon_new[i])
            hemisphere.append(hemi_temp) 
            zone.append(zone_temp)
            zlet.append(zlet_temp)
            east.append(east_temp)
            north.append(north_temp)

east_new = np.asarray(east)
north_new = np.asarray(north)

for i in range(len(lat_new)-1):
            east1 = east_new[i]
            east2 = east_new[i+1]
            north1 = north_new[i]
            north2 = north_new[i+1]
            dist = math.sqrt((east1-east2)**2+(north1-north2)**2)
            ecluDist_utm.append(dist)
            


#Calculate time difference
for i in range(len(seconds)-1):
        time1 = seconds[i]
        time2 = seconds[i+1]
        if time2 > time1:
            time_dif = time2 - time1
            timeDiff.append(time_dif)
        elif time2 < time1:
            time_dif = time1 - time2
            timeDiff.append(time_dif)
        elif time2 == time1:
            time_dif = 0.01
            timeDiff.append(time_dif)

#Calculate the velocity
for i in range(len(lat_new)-1):
        speed = float(ecluDist_utm[i])/float(timeDiff[i])
        velocity.append(speed)

#Convert the list into a numpy array for cv2 function
velocity = np.asarray(velocity)

#Find the mean and standard deviation
velocity_mean, velocity_std = cv2.meanStdDev(velocity)

#Set limitations for outliers
velocity_max = velocity_mean + velocity_std
velocity_min = velocity_mean - velocity_std

#Filter the outliers and return filtered values
for i in range(len(velocity)):
    if velocity[i] < velocity_max and velocity[i] > velocity_min:
        lat_temp = lat_new[i]
        lon_temp = lon_new[i]

        lat_filtered.append(lat_temp)
        lon_filtered.append(lon_temp)

lat_filtered = np.asarray(lat_filtered)
lon_filtered = np.asarray(lon_filtered)

coordinates = np.concatenate((lat_filtered.reshape(-1,1),lon_filtered.reshape(-1,1)),axis=1)


#Using Ramer-Douglas-Peucker, epsilon value 0.00005
RDP = simplify_coords(coordinates, 0.00005)
f= open("data_g_d.txt","w+")
for i in range(len(RDP)):
    ao=RDP[i]
    #f.write(ao)
    print(ao)



x1=RDP[:,0]
y1=RDP[:,1]

fig, ax = plt.subplots(figsize=(6, 4))
fig.subplots_adjust(bottom=0.15, left=0.2)
ax.plot(x1, y1)
ax.axis('equal')
ax.set_xlabel('lantitude')
ax.set_ylabel('longtitude')
#plt.gcf().autofmt_xdate()
plt.show() 
print(len(RDP))
