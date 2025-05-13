# Snake path test with MyCobot, no Force Sensors
from pymycobot.mycobot import MyCobot
#import serial
import csv
import os
import time
import numpy as np
from pathlib import Path

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


mc = MyCobot("COM3", 115200)
#mc.set_fresh_mode(1)
mc.sync_send_angles([0, 0, 0, 0, -90, 0], 50, 5)
probeZ = 152.0
probeX = 52.0
mc.set_tool_reference([0, 0, probeZ, 0, 0, 0])
mc.set_end_type(1)
i = 0
#corners = []
"""
ser = serial.Serial('COM5', 115200, timeout=0.1)  # 1/timeout is the frequency at which the port is read
print("Grab probe and drag to corners")
while i < 3:
    data = ser.readline().decode().strip()
    button = data[4]
    if button == 1:
        button_coord = mc.get_coords()
        corners = np.vstack([corners,button_coord])
        print("Coordinate Captured")
        time.sleep(1)
        i = i+1
"""
print("Grab probe")
time.sleep(1)
mc.release_all_servos()
time.sleep(8)
corners = [mc.get_coords()]
print("Captured corner, move to next")
time.sleep(5)
corners = np.vstack([corners, mc.get_coords()])
print("Captured corner, move to next")
time.sleep(5)
corners = np.vstack([corners, mc.get_coords()])
print("Captured corner")

print("Return probe to top and let go")
time.sleep(3)
mc.sync_send_angles([0, 0, 0, 0, 0, 0], 50)

bottomLeft = corners[0]
bottomRight = corners[1]
topRight = corners[2]

#Create vectors
start = np.array(bottomLeft[:3])
middle = np.array(bottomRight[:3])
finish = np.array(topRight[:3])

# Calculate Side and Up vectors
vSide = middle - start
vUp = finish - middle

vNormal = np.cross(vSide, vUp)
#a,b,c = probeX * vNormal / np.linalg.norm(vNormal)
"""
a, b, c = vNormal % 360
if a > 180:
    a = a - 360
if b > 180:
    b = b - 360
if c > 180:
    c = c - 360
"""

a = corners[1][3]
b = corners[1][4]
c = corners[1][5]

# Calculate magnitudes, unit vector, and number of rows until top corner
magUp = np.linalg.norm(vUp)
uUp = vUp / magUp
magSide = np.linalg.norm(vSide)
numRows = magUp // probeX
lastRow = magUp % probeX

tR = finish
tL = start + vUp

coord = start
coord = np.append(coord, [a, b, c])
coords = coord
coord = coord[:3]

x = coord[0]
y = coord[1]
z = coord[2]

pathComplete = False

# Coords should start as bottom left, bottom right, top right
rows = 0
ind = 0  # Even for right, odd for left
path = 0  # Even for side, odd for up

# If needed, integrate smaller movement magnitudes along vector paths
# Calculate Cartesian points of path
while not pathComplete:
    # If within numRows then travels up probeX distance, path chooses side or up
    if rows <= numRows and path % 2 == 0:
        if ind % 2 == 0:
            coord = coord + vSide
        else:
            coord = coord - vSide
        ind = ind + 1
    elif rows < numRows:
        coord = coord + probeX * uUp
        rows = rows + 1
    elif rows >= numRows and path % 2 != 0:
        coord = coord + lastRow * uUp
        coord = np.append(coord, [a, b, c])
        coords = np.vstack([coords, coord])
        coord = coord[:3]
        if ind % 2 == 0:
            coord = coord + vSide
        elif ind % 2 == 1:
            coord = coord - vSide
        pathComplete = True
    path = path + 1
    coord = np.append(coord, [a, b, c])
    coords = np.vstack([coords, coord])
    coord = coord[:3]
    if rows > 10:
        break

real_coords = []
scan_time = []
timestart = 0
coords = coords.tolist()
time.sleep(1)
print(coords)
print("Moving")

for x in coords:
    mc.sync_send_coords(x, 15, 1,5)
    if np.allclose(x, coords[0], 0.1):
        timestart = time.perf_counter()
    real_coords.append(mc.get_coords())
    scan_time.append([time.perf_counter() - timestart])

print("Path Complete")
mc.sync_send_angles([0, 0, 0, 0, -90, 0], 40)

write_csv_to_downloads(real_coords, "scan_coords.csv")
write_csv_to_downloads(scan_time, "scan_time.csv")

"""
#wait = False
    #while not wait:
    
    #if not mc.is_in_position(x,1):
        #mc.send_coords(x,25,1)
"""
