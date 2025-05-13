from pymycobot.mycobot import MyCobot
import numpy as np
import time

# Initialize MyCobot
mc = MyCobot("COM3", 115200)
mc.set_fresh_mode(1)  # Enable fresh mode for smoother movements

# Probe dimensions
probeZ = 205.0  # Probe length in mm
probeX = 52.0  # Probe spacing in mm

# Setup tool reference
mc.set_tool_reference([0, 0, probeZ, 0, 0, 0])
mc.set_end_type(1)
#mc.release_all_servos(1)
#print(mc.get_coords())
mc.sync_send_coords([-230.2, -40.6, 60.9, 180.0, 0.1, 90.0],25,1,4)
