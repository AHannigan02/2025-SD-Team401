#Coordinate Math for Snake Path without MyCobot
#from pymycobot.mycobot import MyCobot
import math

import serial
import csv
import os
import time
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
#from sympy import false

def write_csv_to_downloads(data, filename):
    """Writes CSV data to a specified directory.

    Args:
        data (list of lists): The data to write to the CSV file.
        filename (str): The name of the CSV file."""

    # Find downloads path
    downloads_path = str(Path.home() / "Downloads")
    # Create the directory if it doesn't exist
    if not os.path.exists(downloads_path):
        print("Could not find Downloads path")

    # Construct the full file path
    filepath = os.path.join(downloads_path, filename)

    # Write the data to the CSV file
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)

probeX = 52.0
# Need three points to find plane
# From origin for naming conventions, beginning point, another corner, then final point
bottomLeft = [-218,-13,100.5,-178.75,10.25,40.64]

bottomRight = [-265.9,59.1,64.8,176.85,4.37,37.5]

topRight = [-305.4,15.7,115.2,177.13,2.5,37.6]

#Create vectors
start = np.array(bottomLeft[:3])
middle = np.array(bottomRight[:3])
finish = np.array(topRight[:3])

# Calculate Side and Up vectors
v1 = middle - start
v2 = finish - middle

# Calculate plane equation coefficients
vNormal = np.cross(v1,v2)
vNormal = -vNormal
a,b,c = vNormal/ np.linalg.norm(vNormal)
ry = math.atan2(-c, math.sqrt(a**2 + b**2)) * 180 / 3.14
rx = math.atan2(c, math.sqrt(a**2 + b**2)) * 180 / 3.14
rz = math.atan2(b,a) * 180 / 3.14
#print(a,b,c)
#print(rx,ry,rz)

d = -np.dot(vNormal,start)

# Calculate magnitude and unit vector for each vector
magUp = np.linalg.norm(v2)
uUp = v2/magUp
magSide = np.linalg.norm(v1)
#uSide = v1 / magSide

numRows = magUp // probeX
lastRow = magUp % probeX

tR = finish
tL = start + v2

coord = start
coord = np.append(coord,[a,b,c])
coords = coord
coord = coord[:3]

fig = plt.figure()
ax = plt.axes(projection = '3d')
x = coord[0]
y = coord[1]
z = coord[2]

pathComplete = False

# Coords should start as bottom left, bottom right, top right
rows = 0
ind = 0 # Even for right, odd for left
path = 0 # Even for side, odd for up

# If needed, integrate smaller movement magnitudes along vector paths
# Fix bug where if up vector is multiple of probeX, it doubles up on coords at end
while not pathComplete:
    # If within numRows then travels up probeX distance, path chooses side or up
    if rows >= numRows and path%2 != 0:
        coord = coord + lastRow * uUp
        coord = np.append(coord,[a,b,c])
        coords = np.vstack([coords, coord])
        coord = coord[:3]
        x = np.append(x, coord[0])
        y = np.append(y, coord[1])
        z = np.append(z, coord[2])
        if ind%2 == 0:
            coord = coord + v1
        elif ind%2 == 1:
            coord = coord - v1
        print("Path Complete")
        pathComplete = True
    elif rows <= numRows and path%2 == 0:
        if ind%2 == 0:
            coord = coord + v1
        else:
            coord = coord - v1
        ind = ind + 1
    elif rows < numRows:
        coord = coord + probeX*uUp
        rows = rows+1

    path = path + 1
    coord = np.append(coord, [a,b,c])
    coords = np.vstack([coords, coord])
    coord = coord[:3]
    x = np.append(x, coord[0])
    y = np.append(y, coord[1])
    z = np.append(z, coord[2])
    if pathComplete:
        break

ax.plot3D(x,y,z)
ax.plot3D([start[0], probeX*a+start[0]],[start[1], probeX*b+start[1]],[start[2], probeX*c+start[2]])
#ax.plot3D([tL[0], tR[0]],[tL[1], tR[1]],[tL[2], tR[2]])
#print(coords)
plt.show()

"""
print("Coordinates")
realC = []
realT = []
timestart = 0
coords = coords.tolist()
for x in coords:
    #print(x)
    time.sleep(1)
    if np.allclose(x, coords[0],0.1):
        timestart = time.perf_counter()
    realC.append(x)
    realT.append([time.perf_counter() - timestart])

write_csv_to_downloads(realC,"scan_coords.csv")
write_csv_to_downloads(realT,"scan_time.csv")
"""