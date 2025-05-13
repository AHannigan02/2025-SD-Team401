from pymycobot.mycobot import MyCobot
import serial
import time
import numpy as np

mc = MyCobot("COM3", 115200)
# Goal: Test "receiving" data of force on button side of probe and check rotation
mc.sync_send_angles([0,0,0,0,0,0],50,5)
probeZ = 152.0
mc.set_tool_reference([0,0,probeZ,0,0,0])
mc.set_end_type(1)
#mc.sync_send_coords([-163,252,340,-90,-0,0],70,1,5)

def read_serial(comport, baudrate, timestamp=False):

    ser = serial.Serial(comport, baudrate, timeout=0.1)         # 1/timeout is the frequency at which the port is read

    while True:

        data = ser.readline().decode().strip()#.split(" ")
        parts = data.split(" ")
        if parts != ['']:
            north = int(parts[0])
            south = int(parts[1])
        if data and timestamp:
            timestamp = time.strftime('%H:%M:%S')
            print(f'{timestamp} > {data}')
            #print(mc.get_coords())
        elif data:
            print(north)


if __name__ == '__main__':

    read_serial('COM5', 115200, False)

    """
    # Rotation test with tool frame
    mc.set_tool_reference([0,0,probeZ,0,0,0])
    mc.set_end_type(1)
    print("Tool")
    #mc.sync_send_coords([-163,252,340,-90,-0,0],70,1,5)
    #mc.sync_send_coords([-163,252,340,-45,-0,0],70,1,5)
    mc.sync_send_coords([-163,272,350,-90,-0,0],70,1,5)
    # Results: Probe facing opposite way of laptop. Rotating ry -45 twisted gripper, or J6.
    # Rotating rz 135 kept gripper flat, but flange was angled left
    # Rotating rx by -45 instead of -90 had flange below gripper, pointing upwards at 45 degrees

    print("Moving rx")
    # Force data from south (non button side), Rotate towards button side (rx)
    mc.send_coord(4,mc.get_coords()[3]+15,50)
    time.sleep(3)
    # Force data from north (button), Rotate towards non button side (rx)
    mc.send_coord(4,mc.get_coords()[3]-30,50)
    time.sleep(3)
    # Force data from south (non button side), Rotate towards button side (rx)
    mc.send_coord(4,mc.get_coords()[3]+15,50)
    time.sleep(3)

    print("Moving rz")
    # Force data from east, rotate towards west (rz)
    mc.send_coord(6,mc.get_coords()[5]+15,50)
    time.sleep(3)
    # Force data from west, rotate towards east (rz)
    mc.send_coord(6,mc.get_coords()[5]-30,50)
    time.sleep(3)
    # Force data from east, rotate towards west (rz)
    mc.send_coord(6,mc.get_coords()[5]+15,50)
    time.sleep(3)

    print("Moving ry")
    # Force data from nowhere, rotate J6 (ry)
    mc.send_coord(5,mc.get_coords()[5]+25,50)
    time.sleep(5)
    # Force data from nowhere, rotate J6 (ry)
    mc.send_coord(5,mc.get_coords()[5]-25,50)
    time.sleep(5)
    """
