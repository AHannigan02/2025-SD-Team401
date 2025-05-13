# Read arduino serial test
from pymycobot.mycobot import MyCobot
import serial
import time
import numpy as np
#import myCobotFunctions

mc = MyCobot("COM3", 115200)
coords = np.array([])

def read_serial(comport, baudrate, timestamp=False):

    ser = serial.Serial(comport, baudrate, timeout=0.1)         # 1/timeout is the frequency at which the port is read

    while True:
        data = ser.readline().decode().strip()
        northForce = data[0]
        eastForce = data[1]
        southForce = data[2]
        westForce = data[3]
        button = data[4]
        if button == 1:
            coords = np.vstack([coords, mc.get_coords()])



        if data and timestamp:
            timestamp = time.strftime('%H:%M:%S')
            print(f'{timestamp} > {data}')
            print(mc.get_coords())
        elif data:
            print(data)


if __name__ == '__main__':

    read_serial('COM5', 115200, True)
