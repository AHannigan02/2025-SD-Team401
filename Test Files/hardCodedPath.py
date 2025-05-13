from pymycobot.mycobot import MyCobot
import time
mc = MyCobot("COM3", 115200)

mc.sync_send_angles([0,0,0,0,0,0],50)
time.sleep(1)
home = mc.get_coords()

#Effective arm length: 350 mm or 13.7 inches
probeZ = 152.0
probeX = 52.0
probeY = 37.0

mc.set_end_type(0)
mc.set_tool_reference([0,0,probeZ,0,0,0])
mc.set_end_type(1)
print(mc.get_coords())

bL = [-318.7, 4.2, 250.1, 90.0, 180.0, 0.0]
bR = [-276.1, 96.0, 250.6, 90.0, 0.0, 0.0]
tR = [-321.4, 114.5, 250.1, -90.0, 180.0, 0.0]
tL = [-351.3, 11.1, 250.8, -90.0, 180.0, 0.0]

toolBL = [-270.4, 28.9, 85.0, 180.0, 0.0, 0.0]
mc.sync_send_coords(toolBL, 30, 1, 5)
print(mc.get_coords())
#[-261.5, 21.4, 68.9, 177.8, -3.62, 0.66]
mc.set_end_type(0)
mc.sync_send_coords(bL,30,1,5)
print(mc.get_coords())
#[-271.2, 27.1, 220.5, 177.8, -3.62, 0.66]

coords = []

#mc.sync_send_coords(bL, 30, 1, 5)
#mc.sync_send_coords(bR, 30, 1, 5)
#mc.sync_send_coords(tR, 30, 1, 5)
#mc.sync_send_coords(tL, 30, 1, 5)


"""
mc.init_gripper()
mc.set_gripper_state(0,50,1)
time.sleep(1)
mc.set_gripper_state(1,50,1)
# Begin Scan
# 0 mode is nonlinear, 1 is linear
start_time = time.perf_counter()
mc.sync_send_coords([-175.0,180.0,250.0, 180.0,0.0,90.0],60,1,4)
mc.set_tool_reference([0,0,probeZ,0,0,0])
mc.set_end_type(1)
print(mc.get_coords())
mc.set_end_type(0)
coords.append(mc.get_coords())
coords.append(time.perf_counter())
#print("Coordinates: ", mc.get_coords())
#mc.sync_send_coords([-125.0,-180.0,250.0, 180.0,0.0,90.0],25,1,4)
coords.append(mc.get_coords())
coords.append(time.perf_counter())
mc.sync_send_coords([-75.0,180.0,250.0, 180.0,0.0,90.0],25,1,4)
coords.append(mc.get_coords())
coords.append(time.perf_counter())
mc.sync_send_coords([-75.0,180.0-probeX,250.0, 180.0,0.0,90.0],25,1,4)
coords.append(mc.get_coords())
coords.append(time.perf_counter())
#mc.sync_send_coords([-125.0,-180.0-probeX,250.0, 180.0,0.0,90.0],25,1,4)
coords.append(mc.get_coords())
coords.append(time.perf_counter())
mc.sync_send_coords([-175.0,180.0-probeX,250.0, 180.0,0.0,90.0],25,1,4)
coords.append(mc.get_coords())
coords.append(time.perf_counter())
#print("Coordinates: ", mc.get_coords())
mc.sync_send_coords([-175.0,180.0-probeX-probeX,250.0, 180.0,0.0,90.0],90,1,4)
coords.append(mc.get_coords())
coords.append(time.perf_counter())
#print("Coordinates: ", mc.get_coords())
#mc.sync_send_coords([-125.0,-180.0-probeX-probeX,250.0, 180.0,0.0,90.0],25,1,4)
coords.append(mc.get_coords())
coords.append(time.perf_counter())
mc.sync_send_coords([-75.0,180.0-probeX-probeX,250.0, 180.0,0.0,90.0],25,1,4)
coords.append(mc.get_coords())
coords.append(time.perf_counter())

end_time = time.perf_counter()
elapsed_time = end_time - start_time
coords.append(elapsed_time)
"""