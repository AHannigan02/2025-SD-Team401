# Snake path test with MyCobot, no Force Sensors
from pymycobot.mycobot import MyCobot
from springTests import backupFunctions as bF
import time
import numpy as np

mc = MyCobot("COM3", 115200)
mc.sync_send_angles([0, 0, 0, 0, -90, 0], 30, 5)
probeZ = 185.0
probeX = 52.0
mc.set_tool_reference([0, 0, probeZ, 0, 0, 0])
mc.set_end_type(1)

# corners = []
print("Grab probe and drag to bottom left")
time.sleep(1)
mc.release_all_servos()
time.sleep(8)
corners = [mc.get_coords()]
print("Captured corner, move to bottom right")
time.sleep(5)
corners = np.vstack([corners, mc.get_coords()])
print("Captured corner, move to top right")
time.sleep(5)
corners = np.vstack([corners, mc.get_coords()])
print("Captured corner")

print("Return probe to top and let go")
time.sleep(3)
mc.sync_send_angles([0, 0, 0, 0, -90, 0], 30)

"""Generate a snake path based on calibration points"""
# Extract vectors from calibration points
start = np.array(corners[0][:3])
middle = np.array(corners[1][:3])
finish = np.array(corners[2][:3])

# Calculate movement vectors
v_side = middle - start  # Horizontal movement vector
v_up = finish - middle  # Vertical movement vector

# Calculate path parameters
u_up = v_up / np.linalg.norm(v_up)  # Unit vector for upward movement
scan_height = np.linalg.norm(v_up)
scan_width = np.linalg.norm(v_side)

# Determine number of passes needed
num_passes = int(scan_height // probeX)
remaining_height = scan_height % probeX

# Generate path coordinates
path_coords = []
current_pos = start.copy()
orientation = corners[1][3:]  # Maintain original orientation

for pass_num in range(num_passes + 1):
    # Determine movement direction for this pass (alternating)
    direction = 1 if pass_num % 2 == 0 else -1

    # Calculate end position for this pass
    if pass_num < num_passes:
        end_pos = current_pos + v_side * direction
        vertical_move = probeX * u_up
    else:
        end_pos = current_pos + v_side * direction
        vertical_move = remaining_height * u_up

        # Add coordinates to path
    path_coords.append(np.append(current_pos, orientation))
    path_coords.append(np.append(end_pos, orientation))

    # Move up for next pass (except after last pass)
    if pass_num < num_passes:
        current_pos = end_pos + vertical_move
        path_coords.append(np.append(current_pos, orientation))

recorded_coords = []
timestamps = []
time.sleep(1)
scanStarted = False
timeStart = time.perf_counter()
# Start scanning
for i, target_coord in enumerate(path_coords):
    print(f"Moving to point {i + 1}/{len(path_coords)}")

    # Send movement command
    mc.send_coords(target_coord, 30, 1)

    # Record position during movement
    move_start = time.perf_counter()
    while time.perf_counter() - move_start < 3:  # Timeout after 5s
        current_coord = mc.get_coords()
        if scanStarted and scanStarted:
            recorded_coords.append(current_coord)
            timestamps.append([time.perf_counter() - timeStart])
        # Check if reached first point so start scan
        if np.allclose(path_coords[0], current_coord,1) and not scanStarted:
            timeStart = time.perf_counter()
            scanStarted = True
            print("Starting Point")
        # Check if reached point target (with position tolerance)
        if np.allclose(target_coord[:3], current_coord[:3], atol=1):
            break
        time.sleep(0.107)  # Sampling interval

    # Small pause at turn points
    #if i % 3 == 1:  # At turn points in snake pattern
    #    time.sleep(0.5)

print("Scan complete, stop recording!")
time.sleep(1)
mc.sync_send_angles([0, 0, 0, 0, -90, 0], 40)

bF.write_csv_to_downloads(path_coords, "path_coords.csv")
bF.write_csv_to_downloads(recorded_coords, "scan_coords.csv")
bF.write_csv_to_downloads(timestamps, "scan_time.csv")