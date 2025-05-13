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
NS_THRESHOLD = 1.2  # Force difference threshold for North to South rotating adjustment
EW_THRESHOLD = 1.5 # Force difference threshold for East to West rotating adjustment
NS_MAX = 5.5  # Maximum allowed force before retracting
EW_MAX = 4.0

# Global variables
ry_integral = 0
MAX_INTEGRAL = 3.0  # Max cumulative adjustment (degrees)

# Calibration phase
def calibrate_surface():
    print("=== Surface Calibration ===")
    mc.release_all_servos(1)
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

def calculate_motor_torque():
    voltage = np.array(mc.get_servo_voltages())
    amps = np.array(mc.get_servo_currents())
    power = voltage * amps
    deg_s = np.array(mc.get_servo_speeds()) * 0.0879
    torque = power / (deg_s * 3.14 / 180)
    return torque

# With deadband and absolute normalization:
def get_normalized_diff(a, b, threshold):
    diff = a - b
    if abs(diff) < threshold:  # Deadband to ignore small noise
        return 0
    return diff / max(a + b, 1)  # Normalized to [ -1, 1 ]

# Adjust orientation based on force feedback
def adjust_orientation(current_coords, forces):
    rx, ry = current_coords[3], current_coords[4]
    """
    count = 0
    for values in forces.values():
        if 1.25 > values > SCAN_MIN:
            count += 1
        if count >= 4:
            print("No Change")
            return rx, ry
    """

    # Calculate force differences
    total_force = sum(forces.values())
    vert_diff = get_normalized_diff(forces['north'], forces['south'], NS_THRESHOLD)
    horiz_diff = get_normalized_diff(forces['east'], forces['west'], EW_THRESHOLD)

    # Replace fixed adjustments with:
    vert_adjustment = 0.5 * abs(vert_diff) / NS_THRESHOLD  # Scales from 0° to 0.5°
    horiz_adjustment = 0.5 * abs(horiz_diff) / EW_THRESHOLD

    # Adjust roll (rx) based on vertical force difference
    if abs(vert_diff) > NS_THRESHOLD:
        rx += vert_adjustment * -np.sign(vert_diff)
    elif forces['north'] > SCAN_MAX or forces['south'] > SCAN_MAX:
        rx += round(vert_adjustment * -np.sign(vert_diff) * 10, 2)
        print("SCAN_MAX INITIATED")
    # Adjust pitch (ry) based on horizontal force difference
    global ry_integral
    if abs(horiz_diff) > EW_THRESHOLD:
        ry_integral += horiz_adjustment * -np.sign(horiz_diff)
        ry_integral = np.clip(ry_integral, -MAX_INTEGRAL, MAX_INTEGRAL)
        ry = current_coords[4] + ry_integral

    return rx, ry

# Adjust Z position based on any reading being too high
def adjust_z_position(current_coords, forces):
    z = current_coords[2]
    z_step = 0.5  # Millimeter adjustment step
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
            print(forces)
            current = mc.get_coords()
            if np.allclose(target[:3], current[:3], atol=5.0):
                print("Point reached!")
                break
            # Adjust position based on force feedback
            new_z = adjust_z_position(current, forces)
            new_rx, new_ry = adjust_orientation(current, forces)

            # Create adjusted coordinates
            adjusted = target.copy()
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

    # Record final position
    current = mc.get_coords()
    real_coords.append(current)
    scan_time.append(time.perf_counter() - start_time)
    if forces:
        force_data.append(forces.values())

    print("=== Scan Complete ===")


def save_scan_data(data, times, forces):
    # Convert data to CSV format and save
    import os
    import csv
    from datetime import datetime
    from pathlib import Path
    # Find downloads path
    downloads_path = str(Path.home() / "Downloads")
    # Create the directory if it doesn't exist

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f'scan_coords_{timestamp}.csv'

    # Construct the full file path
    filepath = os.path.join(downloads_path, filename)

    # Write the data to the CSV file
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)

    # Save force data
    if forces:
        with open(f'scan_forces_{timestamp}.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time', 'North', 'South', 'East', 'West'])
            for i, force in enumerate(forces):
                if force:
                    writer.writerow([times[i], force['north'], force['south'], force['east'], force['west']])
    """
    # Save not scanning intervals
    with open(f'scan_interruptions_{timestamp}.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Start', 'End'])
        for i in range(0, len(not_scanning), 2):
            if i + 1 < len(not_scanning):
                writer.writerow([not_scanning[i], not_scanning[i + 1]])
    """

if __name__ == "__main__":
    try:
        surface_scan()
    except KeyboardInterrupt:
        print("Scan interrupted by user")
        mc.stop()
        #mc.sync_send_angles([0, 0, 0, 0, -90, 0], 20)
    #except Exception as e:
     #   print(f"Error during scan: {e}")
        #mc.sync_send_angles([0, 0, 0, 0, -90, 0], 20)
    finally:
        ser.close()