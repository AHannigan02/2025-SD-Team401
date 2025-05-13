# Two force sensor rotation testing code
import serial
import time
import numpy as np
np.set_printoptions(legacy='1.25')

"""
# Connect to MyCobot
mc = MyCobot("COM3", 115200)
# Goal: Test "receiving" data of force on button side of probe and check rotation
#mc.sync_send_angles([0,0,0,0,0,0],50,5)
probeZ = 205.0
mc.set_tool_reference([0,0,probeZ,0,0,0])
mc.set_end_type(0)
mc.release_all_servos()
"""

parts1 = []
north1 = 0
east1 = 0
south1 = 0
west1 = 0

parts = []
moments = []
north = 0
east = 0
south = 0
west = 0

weight = np.array([0, .017, .020, .039, .064, 0.177, 0.608]) # Weight in kilograms
force = weight * 9.81 # Force in N
adc = np.array([0, 63, 122, 289, 450, 480, 1023]) # Raw ADC values
coefficients = np.polyfit(adc, force, 2) # Fit a 2nd degree polynomial

ser = serial.Serial('COM5', 115200, timeout=0.1)  # 1/timeout is the frequency at which the port is read
# Far is south, thick side rotates
# Confirmed end rotates with end's reference frame, north is always button side for probe
while True:
    data1 = ser.readline().decode().strip()
    parts1 = data1.split()
    if parts1:
        north1 = int(parts1[0])# - 400
        east1 = int(parts1[1])# - 200
        south1 = int(parts1[2])# - 400
        west1 = int(parts1[3])# - 200
        #print(north1, east1, south1, west1)

    data = ser.readline().decode().strip()
    # Convert raw data from 5100 ohm resistor to Newtons
    if data:
        parts = list(map(float, data.split()))
        if len(parts) >= 4:
            parts = np.round(np.polyval(coefficients, parts),2)
            moments = np.array(parts) / 0.135 # Moment to Force
            #parts = {'north': round(np.polyval(coefficients, parts[0]),2), 'east': round(np.polyval(coefficients, parts[3]),2),
            #        'south': round(np.polyval(coefficients, parts[2]),2), 'west': round(np.polyval(coefficients, parts[1]),2)}
            print(parts)
            #print(moments)
    time.sleep(0.05)