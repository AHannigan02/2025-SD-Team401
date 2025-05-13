# Coordinate test
from pymycobot.mycobot import MyCobot
import numpy as np
from pymycobot import Coord#, utils
import time

mc = MyCobot("COM3", 115200)

cds = mc.get_coords()
print("Coordinates: ", cds)
print("Angles: ", mc.get_angles())
probeZ = 152.0
probeX = 52.0
mc.set_end_type(0)
mc.set_tool_reference([0,0,probeZ,0,0,0])
mc.set_end_type(1)
mc.send_angles([0,0,0,0,0,0], 50)
time.sleep(3)
#if mc.is_controller_connected() != 1:
#    exit(0)
print("Coords:", mc.get_coords())
#print(mc.get_world_reference())
mc.send_coord(1, -20.0, 50)
time.sleep(2)
x = [-325.4, 9.400000000000006, 210.60000000000002, -160.23, -2.63, 91.65]
mc.sync_send_coords(x, 50, 0, 15)
#time.sleep(3)
print("Coords After:", mc.get_coords())
coord = np.array(cds)
"""
for i in range(3):
    coord[i] = coord[i] - 10
coord = coord.tolist()
print(coord)
time.sleep(2)
mc.send_coords(coord, 30)
time.sleep(3.5)
# Intelligently plan the route, let the head reach the coordinates of [57.0, -107.4, 316.3] in a linear manner,
# and maintain the attitude of [-93.81, -12.71, -123.49], the speed is 20mm/s
mc.sync_send_coords([10.0, 10.0, 100.0, 0.0, 0.0, 0.0], 40, 0, 15)

# Set the wait time to 3.5 seconds
time.sleep(1)

# Intelligently plan the route, let the head reach the coordinates of [-13.7, -107.5, 223.9] in a linear way,
# and maintain the attitude of [165.52, -75.41, -73.52], the speed is 20mm/s
#mc.sync_send_coords([-13.7, -107.5, 223.9, 165.52, -75.41, -73.52], 40, 0, 10)

# Set the wait time to 3.5 seconds
#time.sleep(5)

# To change only the x-coordinate of the head, set the x-coordinate of the head to -40. Let it plan the route
# intelligently and move the head to the changed position, with a speed of 30mm/s
#mc.send_coord(Coord.X.value, -40, 30)
"""
exit(0)