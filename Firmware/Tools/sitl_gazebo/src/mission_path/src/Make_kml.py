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
data=[]


#Open file and extract necessary data
with open ('data.txt') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

    for row in reader:
        lat.append(float(row[0][4:12]))
        lon.append(float(row[1][6:14]))
        #time.append(row[6])

lat = np.asarray(lat)
lon = np.asarray(lon)
    
coordinates = np.concatenate((lat.reshape(-1,1),lon.reshape(-1,1)),axis=1)
print(coordinates)
print(len(coordinates))

f = open('csv2kml.kml', 'w')
data=coordinates
print("start..............")

#Writing the kml file.
f.write("<?xml version='1.0' encoding='UTF-8'?>\n")
f.write("<kml xmlns='http://earth.google.com/kml/2.1'>\n")
f.write("<Document>\n")
f.write("   <name>" +"myfile" + '.kml' +"</name>\n")
j=1
for row in data:
    f.write("   <Placemark>\n")
    print("first......")
    f.write("       <name>" +"point"+str(j)+ "</name>\n")
    f.write("       <description>" + "odense" + "</description>\n")
    f.write("       <Point>\n")
    f.write("           <coordinates>" + str(row[1]) + "," + str(row[0]) + "," + "0.0" + "</coordinates>\n")
    f.write("       </Point>\n")
    f.write("   </Placemark>\n")
    j=j+1
f.write("</Document>\n")
f.write("</kml>\n")
f.close()
print "File Created. "
print "Press ENTER to exit. "
raw_input()

f= open("data_g_g.csv","w+")
for i in range(len(coordinates)):
    ao=coordinates[i]
    f.write(ao)
    print(ao)
 
       

