from pymycobot.mycobot import MyCobot
import serial
import numpy as np
import time
import os
import csv
from datetime import datetime
from pathlib import Path

# Initialize MyCobot
mc = MyCobot("COM3", 115200)
mc.set_fresh_mode(1)

# Initialize force sensor
ser = serial.Serial('COM5', 115200, timeout=0.1)
np.set_printoptions(legacy='1.25')

# Probe dimensions
probeZ = 205.0
probeX = 52.0
shellZ = 135.0

# Tool setup
mc.set_tool_reference([0, 0, probeZ, 0, 0, 0])
mc.set_end_type(1)

# Max emergency force limits
NS_MAX = 5.5
EW_MAX = 4.0

# PID gains
Kp_rx = 0.6
Ki_rx = 0.2
Kp_ry = 0.6
Ki_ry = 0.2
Kp_z = 1.0
Ki_z = 0.5

# Integral limits
MAX_ANGLE_INTEGRAL = 5.0
MAX_Z_INTEGRAL = 2.0

# Integrals
integral_rx = 0.0
integral_ry = 0.0
integral_z = 0.0

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

# Force reading
def read_force_sensors():
    data = ser.readline().decode().strip()
    weight = np.array([0, .017, .020, .039, .064, 0.177, 0.608])
    force = weight * 9.81
    adc = np.array([0, 63, 122, 289, 450, 480, 1023])
    coefficients = np.polyfit(adc, force, 2)

    if data:
        parts = list(map(float, data.split()))
        parts = np.round(np.polyval(coefficients, parts), 2)
        return {'north': parts[0], 'east': np.round(parts[1], 2),
                'south': np.round(parts[2], 2), 'west': parts[3]}

def estimate_dynamic_thresholds(samples=10):
    forces_list = []
    for _ in range(samples):
        forces = read_force_sensors()
        if forces:
            forces_list.append(forces)
        time.sleep(0.05)

    avg = {k: np.mean([f[k] for f in forces_list]) for k in ['north', 'south', 'east', 'west']}
    std = {k: np.std([f[k] for f in forces_list]) for k in ['north', 'south', 'east', 'west']}

    thresholds = {
        'NS': max(std['north'], std['south']) * 1.5 + 0.1,
        'EW': max(std['east'], std['west']) * 1.5 + 0.1,
    }
    return thresholds

def adjust_orientation(current_coords, forces, ns_thresh, ew_thresh, scan_max):
    global integral_rx, integral_ry
    rx, ry = current_coords[3], current_coords[4]

    vert_error = forces['north'] - forces['south']
    horiz_error = forces['east'] - forces['west']

    if abs(vert_error) < ns_thresh:
        vert_error = 0
    if abs(horiz_error) < ew_thresh:
        horiz_error = 0

    integral_rx += vert_error
    integral_rx = np.clip(integral_rx, -MAX_ANGLE_INTEGRAL, MAX_ANGLE_INTEGRAL)
    delta_rx = Kp_rx * vert_error + Ki_rx * integral_rx
    rx -= delta_rx

    integral_ry += horiz_error
    integral_ry = np.clip(integral_ry, -MAX_ANGLE_INTEGRAL, MAX_ANGLE_INTEGRAL)
    delta_ry = Kp_ry * horiz_error + Ki_ry * integral_ry
    ry -= delta_ry

    return rx, ry

def adjust_z_position(current_coords, forces, scan_min, ns_max):
    global integral_z
    z = current_coords[2]
    avg_force = np.mean(list(forces.values()))

    if any(f > ns_max for f in forces.values()):
        print("Emergency force detected — retracting!")
        return z + 2.0

    if avg_force < scan_min:
        error = scan_min - avg_force
    else:
        desired_force = 1.5
        error = desired_force - avg_force

    if abs(error) < 0.1:
        error = 0

    integral_z += error
    integral_z = np.clip(integral_z, -MAX_Z_INTEGRAL, MAX_Z_INTEGRAL)

    delta_z = Kp_z * error + Ki_z * integral_z
    z += delta_z
    return z

def generate_snake_path(corners, step_size):
    start = np.array(corners[0][:3])
    middle = np.array(corners[1][:3])
    finish = np.array(corners[2][:3])

    v_side = middle - start
    v_up = finish - middle

    mag_up = np.linalg.norm(v_up)
    u_up = v_up / mag_up
    mag_side = np.linalg.norm(v_side)

    num_passes = int(mag_up // step_size)
    remaining_height = mag_up % step_size
    steps_per_row = int(mag_side // step_size)

    path = []
    current_pos = start.copy()
    orientation = corners[0][3:]

    for pass_num in range(num_passes + 1):
        direction = 1 if pass_num % 2 == 0 else -1
        end_pos = current_pos + v_side * direction
        vertical_move = probeX * u_up if pass_num < num_passes else remaining_height * u_up

        path.append(np.append(current_pos, orientation))
        path.append(np.append(end_pos, orientation))

        if pass_num < num_passes:
            current_pos = end_pos + vertical_move
            path.append(np.append(current_pos, orientation))
    return path

def surface_scan():
    corners = calibrate_surface()
    path = generate_snake_path(corners, probeX)
    move_mode = 1

    real_coords = []
    scan_time = []
    force_data = []
    not_scanning = []

    print("=== Starting Surface Scan ===")
    start_time = time.perf_counter()
    scanning_active = False

    # Initial force threshold estimation (static or slow update)
    thresholds = estimate_dynamic_thresholds()
    ns_thresh = thresholds['NS']
    ew_thresh = thresholds['EW']
    scan_min = 0.3
    scan_max = 3.0

    for index, target in enumerate(path):
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
        current = mc.get_coords()
        forces = read_force_sensors()

        while not scanning_active:
            current = mc.get_coords()
            forces = read_force_sensors()
            if np.allclose(path[0][:3], current[:3], atol=5.0):
                scanning_active = True
                start_time = time.perf_counter()
                print("Contact established - scanning started")
            elif not mc.is_moving():
                mc.send_coords(target, move_speed, move_mode)
            time.sleep(0.1)

        step_start = time.perf_counter()

        while scanning_active:
            forces = read_force_sensors()
            current = mc.get_coords()

            if np.allclose(target[:3], current[:3], atol=5.0):
                print("Point reached!")
                break

            # Re-estimate thresholds every N seconds (e.g., every 5 seconds)
            if time.perf_counter() - step_start > 5:
                thresholds = estimate_dynamic_thresholds()
                ns_thresh = thresholds['NS']
                ew_thresh = thresholds['EW']
                step_start = time.perf_counter()

            new_z = adjust_z_position(current, forces, scan_min, NS_MAX)
            new_rx, new_ry = adjust_orientation(current, forces, ns_thresh, ew_thresh, scan_max)

            adjusted = target.copy()
            adjusted[2] = new_z
            adjusted[3] = new_rx
            adjusted[4] = new_ry

            mc.send_coords(adjusted, move_speed, move_mode)

            real_coords.append(adjusted)
            scan_time.append(time.perf_counter() - start_time)
            force_data.append(forces)

            time.sleep(0.1)

        else:
            mc.send_coord(3, current[2] - 1, 20)
            not_scanning.append(time.perf_counter() - start_time)

    print("=== Scan Complete ===")
    save_scan_data(real_coords, scan_time, force_data)


def save_scan_data(data, times, forces):
    downloads_path = str(Path.home() / "Downloads")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f'scan_coords_{timestamp}.csv'
    filepath = os.path.join(downloads_path, filename)

    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)

    if forces:
        with open(f'scan_forces_{timestamp}.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time', 'North', 'South', 'East', 'West'])
            for i, force in enumerate(forces):
                if force:
                    writer.writerow([times[i], force['north'], force['south'], force['east'], force['west']])

if __name__ == "__main__":
    try:
        surface_scan()
    except KeyboardInterrupt:
        print("Scan interrupted by user")
        mc.stop()
    finally:
        ser.close()

"""
def surface_scan():
    corners = calibrate_surface()
    path = generate_snake_path(corners, probeX)
    move_mode = 1

    real_coords = []
    scan_time = []
    force_data = []
    not_scanning = []

    print("=== Starting Surface Scan ===")
    start_time = time.perf_counter()
    scanning_active = False

    for index, target in enumerate(path):
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

        forces = read_force_sensors()
        current = mc.get_coords()

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

        while forces and scanning_active:
            forces = read_force_sensors()
            current = mc.get_coords()
            if np.allclose(target[:3], current[:3], atol=5.0):
                print("Point reached!")
                break

            thresholds = estimate_dynamic_thresholds()
            ns_thresh = thresholds['NS']
            ew_thresh = thresholds['EW']
            scan_min = 0.3
            scan_max = 3.0

            new_z = adjust_z_position(current, forces, scan_min, NS_MAX)
            new_rx, new_ry = adjust_orientation(current, forces, ns_thresh, ew_thresh, scan_max)

            adjusted = target.copy()
            adjusted[2] = new_z
            adjusted[3] = new_rx
            adjusted[4] = new_ry

            mc.send_coords(adjusted, move_speed, move_mode)

            real_coords.append(adjusted)
            scan_time.append(time.perf_counter() - start_time)
            force_data.append(forces)

            time.sleep(0.1)

        else:
            mc.send_coord(3, current[2] - 1, 20)
            not_scanning.append(time.perf_counter() - start_time)

    current = mc.get_coords()
    real_coords.append(current)
    scan_time.append(time.perf_counter() - start_time)
    if forces:
        force_data.append(forces.values())

    print("=== Scan Complete ===")
    save_scan_data(real_coords, scan_time, force_data)
"""