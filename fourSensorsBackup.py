from pymycobot.mycobot import MyCobot
import serial
import numpy as np
import time

# Initialize MyCobot
mc = MyCobot("COM3", 115200)
mc.set_fresh_mode(1)  # Enable fresh mode for smoother movements

# Initialize force sensor serial connection
ser = serial.Serial('COM5', 115200, timeout=0.1)
np.set_printoptions(legacy='1.25')

# Probe dimensions
probeZ = 205.0  # Probe length in mm
probeX = 52.0  # Probe spacing in mm
shellZ = 135.0  # Shell height in mm

# Setup tool reference
mc.set_tool_reference([0, 0, probeZ, 0, 0, 0])
mc.set_end_type(1)

# Newton Thresholds
SCAN_MIN = 0.3  # Minimum force to consider contact
SCAN_MAX = 3.0 # Maximum force during scan before re-orienting
NS_THRESHOLD = 2.0  # Force difference threshold for North to South rotating adjustment
EW_THRESHOLD = 1.5 # Force difference threshold for East to West rotating adjustment
NS_MAX = 5.5  # Maximum allowed force before retracting
EW_MAX = 4.0

# Calibration phase
def calibrate_surface():
    print("=== Surface Calibration ===")
    mc.release_all_servos()
    print("Move probe to bottom left corner and press Enter")
    input()
    bottom_left = mc.get_coords()

    print("Move probe to bottom right corner and press Enter")
    input()
    bottom_right = mc.get_coords()

    print("Move probe to top right corner and press Enter")
    input()
    top_right = mc.get_coords()

    return np.array([bottom_left, bottom_right, top_right])


# Read force sensors
def read_force_sensors():
    data = ser.readline().decode().strip()
    # Coefficients to convert raw data from 5100 ohm resistor to Newtons
    weight = np.array([0, .017, .020, .039, .064, 0.177, 0.608])  # Weight in kilograms
    force = weight * 9.81  # Force in N
    adc = np.array([0, 63, 122, 289, 450, 480, 1023])  # Raw ADC values
    coefficients = np.polyfit(adc, force, 2)  # Fit a 2nd degree polynomial

    if data:
        parts = list(map(float, data.split()))
        parts = np.round(np.polyval(coefficients, parts), 2)
        return {'north': parts[0], 'east': np.round(parts[1], 2),
                'south': np.round(parts[2], 2), 'west': parts[3]}


# Adjust orientation based on force feedback
def adjust_orientation(current_coords, forces):
    rx, ry = current_coords[3], current_coords[4]
    count = 0
    for values in forces.values():
        if 1.25 > values > SCAN_MIN:
            count += 1
        if count >= 4:
            print("No Change")
            return rx, ry
    vert_adjustment = 1.0  # Degrees of adjustment per step
    horiz_adjustment = 1.0  # Degrees of adjustment per step

    # Calculate force differences
    total_force = sum(forces.values())
    vert_diff = forces['north'] - forces['south'] / (forces['north'] + forces['south'] + 1e-06)
    horiz_diff = forces['east'] - forces['west'] / (forces['east'] + forces['west'] + 1e-06)

    # Adjust roll (rx) based on vertical force difference
    if abs(vert_diff) > NS_THRESHOLD:
        rx += vert_adjustment * -np.sign(vert_diff)
    elif forces['north'] > SCAN_MAX or forces['south'] > SCAN_MAX:
        rx += round(vert_adjustment * -np.sign(vert_diff) * 10, 2)
        print("SCAN_MAX INITIATED")
    # Adjust pitch (ry) based on horizontal force difference
    if abs(horiz_diff) > EW_THRESHOLD:
        ry += horiz_adjustment * -np.sign(horiz_diff)
    elif forces['east'] > SCAN_MAX or forces['west'] > SCAN_MAX:
        ry += round(horiz_adjustment * -np.sign(horiz_diff) * 10, 2)
        print("SCAN_MAX INITIATED")

    return rx, ry


# Adjust Z position based on any reading being too high
def adjust_z_position(current_coords, forces):
    z = current_coords[2]
    z_step = 1.0  # Millimeter adjustment step
    if (forces['north'] or forces['south']) > NS_MAX:
        print("Emergency Release")
        return z + z_step  # Move away from surface
    count = 0
    if forces:
        for values in forces.values():
            if values < SCAN_MIN:
                count += 1  # Move toward surface
    if count > 2:
        z = z - z_step
    return z


# Generate snake path coordinates
def generate_snake_path(corners, step_size):
    start = np.array(corners[0][:3])
    middle = np.array(corners[1][:3])
    finish = np.array(corners[2][:3])

    # Calculate vectors
    v_side = middle - start
    v_up = finish - middle

    # Calculate unit vectors and magnitudes
    mag_up = np.linalg.norm(v_up)
    u_up = v_up / mag_up
    mag_side = np.linalg.norm(v_side)

    # Calculate number of rows and steps
    num_passes = int(mag_up // step_size)
    remaining_height = mag_up % step_size
    steps_per_row = int(mag_side // step_size)

    # Initialize path
    path = []
    current_pos = start.copy()
    orientation = corners[0][3:]  # Initial orientation

    # Generate snake pattern
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
        path.append(np.append(current_pos, orientation))
        path.append(np.append(end_pos, orientation))

        # Move up for next pass (except after last pass)
        if pass_num < num_passes:
            current_pos = end_pos + vertical_move
            path.append(np.append(current_pos, orientation))
    return path


# Main scanning function
def surface_scan():
    # Calibrate surface
    corners = calibrate_surface()

    # Generate initial path
    path = generate_snake_path(corners, probeX)

    # Scan parameters
    move_speed = 20
    move_mode = 1  # 0 for angular, 1 for linear

    # Data collection
    real_coords = []
    scan_time = []
    force_data = []
    not_scanning = []

    # Start scanning
    print("=== Starting Surface Scan ===")
    start_time = time.perf_counter()
    scanning_active = False
    for index, target in enumerate(path):
        # Send movement command
        if index == 0:
            move_speed = 30
            timeout = 10
        elif index % 3 == 1:
            move_speed = 30
            timeout = 3
        else:
            move_speed = 10
            timeout = 5

        mc.send_coords(target, move_speed, move_mode)

        # Read force sensors
        forces = read_force_sensors()
        # print(forces)
        # Get current position
        current = mc.get_coords()
        # print(current)

        while not scanning_active:
            forces = read_force_sensors()
            current = mc.get_coords()
            if np.allclose(path[0][:3], current[:3], atol=5.0):
                scanning_active = True
                start_time = time.perf_counter()
                print("Contact established - scanning started")
            elif not mc.is_moving():
                mc.send_coords(target, move_speed, move_mode)
            time.sleep(0.1)

        # Movement control loop
        # Only process force data if we have valid readings
        while forces and scanning_active:
            forces = read_force_sensors()
            current = mc.get_coords()
            if np.allclose(target[:3], current[:3], atol=5.0):
                print("Point reached!")
                break
            # Adjust position based on force feedback
            new_z = adjust_z_position(current, forces)
            new_rx, new_ry = adjust_orientation(current, forces)

            # Create adjusted coordinates
            adjusted = current.copy()
            adjusted[2] = new_z
            adjusted[3] = new_rx
            adjusted[4] = new_ry

            # Send adjustment
            mc.send_coords(adjusted, move_speed, move_mode)

            # Record data
            real_coords.append(adjusted)
            scan_time.append(time.perf_counter() - start_time)
            force_data.append(forces)
        else:
            # Not scanning yet - move closer to surface
            mc.send_coord(3, current[2] - 1, 20)
            not_scanning.append(time.perf_counter() - start_time)

        time.sleep(0.1)  # Control loop frequency


def save_scan_data(coords, times, forces, not_scanning):
    # Convert data to CSV format and save
    import csv
    from datetime import datetime

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Save coordinates and time
    with open(f'scan_coords_{timestamp}.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'])
        writer.writerows(coords)

    # Save force data
    if forces:
        with open(f'scan_forces_{timestamp}.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time', 'North', 'South', 'East', 'West'])
            for i, force in enumerate(forces):
                if force:
                    writer.writerow([times[i], force['north'], force['south'], force['east'], force['west']])

    # Save not scanning intervals
    with open(f'scan_interruptions_{timestamp}.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Start', 'End'])
        for i in range(0, len(not_scanning), 2):
            if i + 1 < len(not_scanning):
                writer.writerow([not_scanning[i], not_scanning[i + 1]])


if __name__ == "__main__":
    try:
        surface_scan()
    except KeyboardInterrupt:
        print("Scan interrupted by user")
        #mc.sync_send_angles([0, 0, 0, 0, -90, 0], 20)
    except Exception as e:
        print(f"Error during scan: {e}")
        #mc.sync_send_angles([0, 0, 0, 0, -90, 0], 20)
    finally:
        ser.close()

"""
# Main scanning function
def surface_scan():
    # Calibrate surface
    corners = calibrate_surface()

    # Generate initial snake path corners
    path = generate_snake_path(corners, probeX)

    # Scan parameters
    move_speed = 20
    move_mode = 1  # 0 for angular, 1 for linear

    # Movement control loop
    scanning_active = False
    while True:
        # Read force sensors
        forces = read_force_sensors()

        # Get current position
        current = mc.get_coords()

        # Only process force data if we have valid readings
        if forces:
            # Check if reached first corner
            if not scanning_active:
                scanning_active = True
                print("Contact established - scanning started")

            if scanning_active:
                adjusted = current.copy()

                # Adjust position based on force feedback
                new_z = adjust_z_position(current, forces)
                new_rx, new_ry = adjust_orientation(current, forces)

                # Create adjusted coordinates
                adjusted[2] = new_z
                adjusted[3] = float(new_rx)
                adjusted[4] = float(new_ry)

                # Send adjustment
                mc.send_coords(adjusted, 20, move_mode)
                print(forces)
                #print(adjusted)
            else:
                # Not scanning yet - move closer to surface
                mc.send_coord(3, current[2] - 1, 20)
                #not_scanning.append(time.perf_counter() - start_time)

            time.sleep(1)  # Control loop frequency
            
# Snake path test with MyCobot, no Force Sensors
from pymycobot.mycobot import MyCobot
import serial
import backupFunctions as bF
import time
import numpy as np

mc = MyCobot("COM3", 115200)
mc.sync_send_angles([0, 0, 0, 0, -90, 0], 50, 5)
probeZ = 185.0
probeX = 52.0
mc.set_tool_reference([0, 0, probeZ, 0, 0, 0])
mc.set_end_type(1)
i = 0
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
mc.sync_send_angles([0, 0, 0, 0, 0, 0], 50)

bottomLeft = corners[0]
bottomRight = corners[1]
topRight = corners[2]

# Create vectors
start = np.array(bottomLeft[:3])
middle = np.array(bottomRight[:3])
finish = np.array(topRight[:3])

# Calculate Side and Up vectors
vSide = middle - start
vUp = finish - middle

a = corners[1][3]
b = corners[1][4]
c = corners[1][5]

# Calculate magnitudes, unit vector, and number of rows until top corner
magUp = np.linalg.norm(vUp)
uUp = vUp / magUp
magSide = np.linalg.norm(vSide)
numRows = magUp // probeX
lastRow = magUp % probeX

tR = finish
tL = start + vUp

coord = start
coord = np.append(coord, [a, b, c])
coords = coord
coord = coord[:3]

x = coord[0]
y = coord[1]
z = coord[2]

pathComplete = False

# Coords should start as bottom left, bottom right, top right
rows = 0
ind = 0  # Even for right, odd for left
path = 0  # Even for side, odd for up

# If needed, integrate smaller movement magnitudes along vector paths
# Calculate Cartesian points of path
while not pathComplete:
    # If within numRows then travels up probeX distance, path chooses side or up
    if rows <= numRows and path % 2 == 0:
        if ind % 2 == 0:
            coord = coord + vSide
        else:
            coord = coord - vSide
        ind = ind + 1
    elif rows < numRows:
        coord = coord + probeX * uUp
        rows = rows + 1
    elif rows >= numRows and path % 2 != 0:
        coord = coord + lastRow * uUp
        coord = np.append(coord, [a, b, c])
        coords = np.vstack([coords, coord])
        coord = coord[:3]
        if ind % 2 == 0:
            coord = coord + vSide
        elif ind % 2 == 1:
            coord = coord - vSide
        pathComplete = True
    path = path + 1
    coord = np.append(coord, [a, b, c])
    coords = np.vstack([coords, coord])
    coord = coord[:3]
    if rows > 10:
        break

real_coords = []
scan_time = []
timeStart = time.perf_counter()
coords = coords.tolist()
time.sleep(1)

# Start scanning
north = 0.0
east = 0.0
south = 0.0
west = 0.0
ser = serial.Serial('COM5', 115200, timeout=0.1)  # 1/timeout is the frequency at which the port is read
notScanning = []
startedScan = False
timeout = 5
print("Start scanning when probe makes contact")
for x in coords:

    # Start scan
    t = time.perf_counter()
    mc.send_coords(x, 30, 1)

    # While moving, read force sensors
    while time.perf_counter() - t < timeout:
        data = ser.readline().decode().strip()
        parts = data.split(" ")
        if parts:
            north = int(parts[0])
            east = int(parts[1])
            south = int(parts[2])
            west = int(parts[3])
        # If force difference is greater than 500, rotate to alleviate pressure
        if north > 800.0 and np.abs(north - south) > 300.0:
            # Force data from south (non button side), Rotate towards non button side (rx)
            mc.pause()
            mc.send_coord(4, mc.get_coords()[3] - 5, 30)
            mc.resume()
        if south > 800.0 and np.abs(north - south) > 300.0:
            # Force data from north (button), Rotate towards button side (rx)
            mc.pause()
            mc.send_coord(4, mc.get_coords()[3] + 5, 30)
            mc.resume()
        # If too much pressure, raise in z direction
        if north > 900 and south > 900:
            mc.pause()
            mc.send_coord(3, mc.get_coords()[2] + 5, 30)
            mc.resume()
        # If force sensor is not making contact, note time and don't record coords
        if north < 1 and south < 1 and startedScan:
            notScanning.append(time.perf_counter()-timeStart)
            while north < 1 and south < 1:
                mc.send_coord(3, mc.get_coords()[2] - 5, 30)
                data = ser.readline().decode().strip()
                parts = data.split(" ")
                if parts != ['']:
                    north = int(parts[0])
                    south = int(parts[1])
            # Record when back to scanning
            notScanning.append(time.perf_counter()-timeStart)
        if north < 100 and south < 100:
            mc.pause()
            mc.send_coord(3, mc.get_coords()[2] - 1, 30)
            mc.resume()
        # If reaches coords, break while loop
        if np.allclose(x, mc.get_coords(),0.1):
            if np.allclose(mc.get_coords(), coords[0], 0.1):
                timeStart = time.perf_counter()
                startedScan = True
                print("Starting Point")
            break
        time.sleep(0.25)

    # MyCobot reached destination
    real_coords.append(mc.get_coords())
    scan_time.append([time.perf_counter() - timeStart])

print("Path Complete")
mc.sync_send_angles([0, 0, 0, 0, -90, 0], 40)

bF.write_csv_to_downloads(real_coords, "scans_coords.csv")
bF.write_csv_to_downloads(scan_time, "scans_time.csv")
bF.write_csv_to_downloads(notScanning, "not_scanning.csv")
"""