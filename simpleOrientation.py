# Snake path test with MyCobot, no Force Sensors
from pymycobot.mycobot import MyCobot
import serial
import time
import numpy as np

mc = MyCobot("COM3", 115200)

probeZ = 205.0
probeX = 52.0
mc.set_tool_reference([0, 0, probeZ, 0, 0, 0])
mc.set_end_type(1)
print(mc.get_coords())

north = 0.0
east = 0.0
south = 0.0
west = 0.0

# Newton Thresholds
SCAN_MIN = 1.0  # Minimum force to consider contact
SCAN_MAX = 3.0 # Maximum force during scan before re-orienting
NS_THRESHOLD = 1.0  # Force difference threshold for North to South rotating adjustment
EW_THRESHOLD = 1.0 # Force difference threshold for East to West rotating adjustment
NS_MAX = 3.0  # Maximum allowed force before retracting
EW_MAX = 3.0
weight = np.array([0, .017, .020, .039, .064, 0.177, 0.608]) # Weight in kilograms
force = weight * 9.81 # Force in N
adc = np.array([0, 63, 122, 289, 450, 480, 1023]) # Raw ADC values
coefficients = np.polyfit(adc, force, 2) # Fit a 2nd degree polynomial

ser = serial.Serial('COM5', 115200, timeout=0.1)  # 1/timeout is the frequency at which the port is read

while True:
    data = ser.readline().decode().strip()
    """
    parts = data.split()
    if parts:
        north = int(parts[0])
        east = int(parts[3])
        south = int(parts[2])
        west = int(parts[1])
        print(parts)
    """
    if data:
        parts = list(map(float, data.split()))
        if len(parts) >= 4:
            parts = np.round(np.polyval(coefficients, parts),2)
            north = float(parts[0])
            east = float(parts[1])
            south = float(parts[2])
            west = float(parts[3])
            print(parts)

    if north > south and np.abs(north - south) > NS_THRESHOLD and north > SCAN_MIN:
       # Force data from south (non button side), Rotate towards non button side (rx)
        mc.send_coord(4, mc.get_coords()[3] - 5, 10)
    if south > north and np.abs(north - south) > NS_THRESHOLD and south > SCAN_MIN:
        # Force data from north (button), Rotate towards button side (rx)
        mc.send_coord(4, mc.get_coords()[3] + 5, 10)

    if east > west and np.abs(east - west) > EW_THRESHOLD and east > SCAN_MIN:
       # Force data from east, Rotate towards +ry
       mc.send_coord(5, mc.get_coords()[4] + 5, 10)
    if west > east and np.abs(east - west) > EW_THRESHOLD and west > SCAN_MIN:
        # Force data from west, Rotate towards -ry
        mc.send_coord(5, mc.get_coords()[4] - 5, 10)
    if (north < 1 and south) < 1 or (east < 1 and west < 1):
        mc.send_coord(3, mc.get_coords()[2] - 1, 10)
    if north or east or south or west > SCAN_MAX:
        mc.send_coord(3, mc.get_coords()[2] + 10, 10)
        """
            if north < 100 and south < 100:
                mc.pause()
                mc.send_coord(3, mc.get_coords()[2] - 1, 10)
                mc.resume()
        """
    """
    # If too much pressure, raise in z direction
    if north > 600 and south > 600:
        mc.pause()
        mc.send_coord(3, mc.get_coords()[2] + 5, 10)
        mc.resume()
    """
    time.sleep(0.1)