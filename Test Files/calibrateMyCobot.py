from pymycobot.mycobot import MyCobot
import time

def calibrate320():
    mc = MyCobot("COM3", 115200)
    mc.send_angles([0,0,0,0,-90,0],50)
    time.sleep(1)
    print(mc.get_coords())

if __name__ == '__main__':
    calibrate320()