from pymycobot.mycobot import MyCobot
import time
mc = MyCobot("COM3", 115200)
# Goal: Determine if setting end type changes coordinate locations when sent
# Answer: Did not change location for sent coords, but received coords were adjusted
# Need to test further for rotations when tool frame is set
# Results: Does rotate around tools reference frame, and
mc.set_reference_frame(0)
mc.set_end_type(0)
mc.sync_send_angles([0,0,0,0,0,0],50,5)
probeZ = 152.0
probeX = 52.0
probeY = 37.0
mc.sync_send_coords([-150.1,-275.2,300.7,-45,0,180],75,1,5)
print(mc.get_coords())
mc.set_tool_reference([0,0,probeZ,0,0,0])
mc.set_end_type(1)
#print(mc.get_tool_reference())
#mc.sync_send_coords([-250,-203,420.7,180,0,90],75,1,5)
#mc.sync_send_coords([-75.0,-180.0,250.0, 180.0,0.0,90.0],25,1,4) - Coords for no tool frame
#mc.sync_send_coords([-166.2, -176.4, 82.9, 175.99, -1.38, 90.03],25,1,5)
print("Position with Tool Reference:", mc.get_coords())
#mc.sync_send_coords([-163,-392,340,-50,0,180],15,1,5)
#mc.sync_send_coords([-163,-392,340,-90,0,180],50,1,5)
mc.sync_send_coords([-163,-392,340,-90,-45,180],50,1,5)
# Gripper vertical
#mc.sync_send_coords([-163,-192,500,0,0,0],25,1,5)
mc.set_end_type(0)
print(mc.get_coords())
"""
# Rotation test with base frame
mc.set_end_type(0)
print("Base")
mc.sync_send_coords([-150.1,275.2,300.7,-90,0,0],55,1,5)
mc.sync_send_coords([-150.1,275.2,300.7,-135,0,0],50,1,5)
# Result: Went from flat facing laptop to pointed down by an additional 45 degrees, but rotation centered around end-effector flange
"""
"""
mc.set_reference_frame(0)
mc.set_end_type(0)
mc.sync_send_angles([0,0,0,0,-90,0],85,5)
mc.sync_send_coords([-175,-153,420.7,-90,0,180],75,1,5)
print("Position with No Reference: ", mc.get_coords())
probeZ = 152.0
probeX = 52.0
probeY = 37.0
mc.set_tool_reference([0,0,probeZ,0,0,0])
mc.set_end_type(1)
print(mc.get_tool_reference())
mc.sync_send_coords([-250,-203,420.7,-90,0,180],75,1,5)
print("Position with Tool Reference:", mc.get_coords())
mc.set_end_type(0)
mc.sync_send_coords([-250,-153,420.7,-90,0,180],75,1,5)
print(mc.get_coords())
"""