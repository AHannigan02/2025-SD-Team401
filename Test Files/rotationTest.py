from pymycobot.mycobot import MyCobot
import time
mc = MyCobot("COM3", 115200)

mc.sync_send_angles([0,0,0,0,0,0],85)
time.sleep(2)
home = mc.get_coords()

probeZ = 152.0
probeX = 52.0
probeY = 37.0
#mc.set_tool_reference([0,0,probeZ,0,0,0])
#mc.set_end_type(1)

# Goal: Test if Euler Angle changes is different in different coordinate locations
# Result:

mc.send_coords([-200.0,-200.0,250.0, 90.0,0.0,0.0],50,1)
time.sleep(3)
mc.send_coords([-200.0,-200.0,250.0, 90.0,45.0,0.0],50,1)
time.sleep(2)
mc.send_coords([-200.0,-200.0,250.0, 90.0,-45.0,0.0],50,1)
time.sleep(2)
print(mc.get_coords())

mc.send_coords([250.0,250.0,250.0, -90.0,0.0,0.0],50,1)
time.sleep(3)
mc.send_coords([250.0,250.0,250.0, -90.0,45.0,0.0],50,1)
time.sleep(2)
mc.send_coords([250.0,250.0,250.0, -90.0,-45.0,0.0],50,1)
time.sleep(2)

print(mc.get_coords())