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

a = bottomLeft[3]
b = bottomLeft[4]
c = bottomLeft[5]

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

#fig = plt.figure()
#ax = plt.axes(projection = '3d')

realC = []
realT = []
i = 0

while i <= 50:
    coord = coord + .1 * v2
    a = a + 1
    b = b + 1
    c = c + 1
    coord = np.append(coord, [a, b, c])
    coords = np.vstack([coords, coord])
    coord = coord[:3]
    if i == 20:
        a = a + 40
        b = b - 40
        c = c - 40
    i = i + 1


coords = coords.tolist()

write_csv_to_downloads(coords,"scanss_coords.csv")
