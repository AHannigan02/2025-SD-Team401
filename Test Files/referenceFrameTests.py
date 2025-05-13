from pymycobot.mycobot import MyCobot
import time
mc = MyCobot("COM3", 115200)
mc.set_end_type(1)
"""
#length of tool: 170 mm
mc.set_gripper_value(0,70)
time.sleep(1.5)
mc.set_gripper_value(255,70)
time.sleep(1.5)
print(mc.get_gripper_value())
"""
mc.set_reference_frame(0)
mc.sync_send_angles([0,0,0,0,0,0],65)
home = mc.get_coords()

#Effective arm length: 350 mm or 13.7 inches
probeY = 152.0
gripY = 170.0
probeX = 52.0
probeZ = 37.0
tool = home
tool[1] = tool[1] - gripY
print(tool)
mc.set_tool_reference(tool)

print("World Ref: ", mc.get_world_reference())
print("Tool Ref: ", mc.get_tool_reference())
mc.set_world_reference(tool)
mc.set_reference_frame(1)
print(mc.get_reference_frame())
print("World Ref: ", mc.get_world_reference())
print("Tool Ref: ", mc.get_tool_reference())
mc.send_angle(3,45,50)
#mc.sync_send_coords([-20,-25,-10,90,0,0],25,1,5)
#mc.sync_send_coords([-75.0,-200.0,250.0, 180.0,0.0,180.0],75,1,5)
# 0 mode is nonlinear, 1 is linear
print("Coordinates: ", mc.get_coords())