# Angles test
from pymycobot.mycobot import MyCobot
from pymycobot import Angle
import time
#print(utils.get_port_list())
# Initialize MyCobot object
mc = MyCobot("COM3", 115200)

# Power on MyCobot
#print(mc.is_controller_connected())

mc.set_color(255,255,255)
# Get angle values in degrees
print("Angles: ", mc.get_angles())

# Send angles
mc.send_angles([0, 0, 0, 0, 0, 0], 40)
time.sleep(2)

# Print angles
print(mc.get_angles())

# Rotate just J1
mc.send_angle(Angle.J1.value, 90, 40)
time.sleep(5)

mc.send_angles([0, 30, 30, 30, 30, 30], 40)
time.sleep(3)
mc.send_angles([-30, -30, -30, -30, -30, -30], 40)
time.sleep(5)
mc.send_angles([0,0,0,0,0,0], 50)
time.sleep(5)
print(mc.get_angles())
"""
# Move more of the arm
mc.send_angles([0,15,30,0,-90,0],40)
time.sleep(5)

# Get the current coordinates and pose of the head
print("Coordinates: ", mc.get_coords())

exit(0)
"""