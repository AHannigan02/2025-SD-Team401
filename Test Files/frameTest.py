from pymycobot.mycobot import MyCobot
import time
# x -350:350, y = x, z -41:523.9, r -180:180
# [2.6, -153.2, 523.8, -89.99, -0.87, 179.56] is all joints theta=0
# r is rotation of end effector, not base
# r = 0 means end effector pointed upright
mc = MyCobot("COM3", 115200)
mc.sync_send_angles([0,0,0,0,0,0],85)
print(mc.get_coords())
print(mc.get_world_reference())
print(mc.get_reference_frame())
#mc.set_reference_frame(1)
mc.send_coords([15.0, 250.0, 223.8, 180.0, 0.0, 180.0], 80, 0)
time.sleep(1.5)
print(mc.get_coords())
