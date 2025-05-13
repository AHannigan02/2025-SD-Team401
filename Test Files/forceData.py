# Two force sensor rotation testing code
from pymycobot.mycobot import MyCobot
import serial

mc = MyCobot("COM3", 115200)
# Goal: Test "receiving" data of force on button side of probe and check rotation
#mc.sync_send_angles([0,0,0,0,0,0],50,5)
probeZ = 185.0
mc.set_tool_reference([0,0,probeZ,0,0,0])
mc.set_end_type(1)

# Facing Laptop
#mc.sync_send_coords([-203,262,340,-90,-0,0],70,1,5)
# Facing Away from Laptop
#mc.sync_send_coords([-203,-162,380,-90,-0,-180],70,1,5)

north = 0
east = 0
south = 0
west = 0
ser = serial.Serial('COM5', 115200, timeout=0.1)  # 1/timeout is the frequency at which the port is read
# Far is south, thick side rotates
# Confirmed end rotates with end's reference frame, north is always button side for probe
while True:
    data = ser.readline().decode().strip()
    parts = data.split(" ")
    if parts != [' ']:
        north = int(parts[0])
        east = int(parts[1])
        south = int(parts[2])
        west = int(parts[3])
"""
    #rotNS = mc.get_coords()[3]
    if north > 800.0 and np.abs(north - south) > 300.0:
        # Force data from south (non button side),
        # Rotate towards non button side (rx)
        mc.send_coord(4, mc.get_coords()[3] - 5, 30)

    if south > 800.0 and np.abs(north - south) > 300.0:
        # Force data from north (button),
        # Rotate towards button side (rx)
        mc.send_coord(4, mc.get_coords()[3] + 5, 30)
"""