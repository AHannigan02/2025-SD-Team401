import numpy as np
import time
from pymycobot.mycobot import MyCobot
import serial
import cvxpy as cp
import os
import sys
import csv
from datetime import datetime
from pathlib import Path

# Initialization
mc = MyCobot("COM3", 115200)
mc.set_fresh_mode(1)
ser = serial.Serial('COM5', 115200, timeout=0.1)
np.set_printoptions(precision=2, suppress=True)

# Constants
PROBE_LENGTH = 205.0
FSR_OFFSET = 70.0
TIP_OFFSET = 135.0
DISK_DIAMETER = 100.0
FSR_SPACING = (DISK_DIAMETER / 2) - 9.5
TOOL_FRAME = [0, 0, PROBE_LENGTH, 0, 0, 0]
mc.set_tool_reference(TOOL_FRAME)
mc.set_end_type(1)

# Tool frame direction
FSR_POSITIONS = {
    'north': np.array([0, -FSR_SPACING, 0]),
    'south': np.array([0, FSR_SPACING, 0]),
    'east':  np.array([FSR_SPACING, 0, 0]),
    'west':  np.array([-FSR_SPACING, 0, 0]),
}

def wait_for_continue_signal():
    print("Waiting for GUI input...")
    signal_file = "continue_signal.txt"
    while not os.path.exists(signal_file):
        time.sleep(0.1)
    os.remove(signal_file)

def calibrate_surface(model='flat'):
    mc.release_all_servos()
    print("Position probe at bottom-left and press Enter")
    wait_for_continue_signal()
    bl = mc.get_coords()

    print("Position probe at bottom-right and press Enter")
    wait_for_continue_signal()
    br = mc.get_coords()

    print("Position probe at top-right and press Enter")
    wait_for_continue_signal()
    tr = mc.get_coords()

    if model == 'sphere':
        print("Position probe at top-center")
        wait_for_continue_signal()
        top = mc.get_coords()
        return [bl, br, tr, top]
    return [bl, br, tr]


# Force Reading
def read_force_sensors():
    data = ser.readline().decode().strip()
    weight = np.array([0, .017, .020, .039, .064, 0.177, 0.608])
    force = weight * 9.81
    adc = np.array([0, 63, 122, 289, 450, 480, 1023])
    coeff = np.polyfit(adc, force, 2)
    if data:
        parts = list(map(float, data.split()))
        forces = np.round(np.polyval(coeff, parts) - 3.5, 2)
        return {
            'north': forces[0], 'east': forces[1],
            'south': forces[2], 'west': forces[3]
        }

# Surface Normal Calculation
def get_surface_normal(position, corners, model='flat', center=np.array([0, 0, 0])):
    if model == 'flat':
        p0, p1, p2 = map(lambda c: np.array(c[:3]), corners)
        side_vec = p1 - p0
        up_vec = p2 - p1
        normal = np.cross(side_vec, up_vec)
        return normal / np.linalg.norm(normal)
    elif model == 'sphere':
        direction = position - center
        return direction / np.linalg.norm(direction)
    else:
        raise ValueError("Unknown surface model")

# QP Controller
def compute_qp_adjustments(forces, normal_vec):
    force_vec = np.array([
        forces['north'], forces['south'],
        forces['east'], forces['west']
    ])
    net_force = np.mean(force_vec)
    moment = np.zeros(3)
    for key in FSR_POSITIONS:
        r = FSR_POSITIONS[key]
        f = np.array([0, 0, forces[key]])
        moment += np.cross(r, f)

    u = cp.Variable(3)  # dz, d_rx, d_ry
    normal_target = np.array([0, 0, 1]) + normal_vec
    J = np.array([
        [0, -PROBE_LENGTH, 0],
        [0, 0, PROBE_LENGTH],
        [1, 0, 0]])
    alignment_error = J @ u - normal_target
    objective = cp.Minimize(cp.norm2(alignment_error) + 0.1 * cp.norm2(u))
    constraints = [
        cp.abs(u[0]) <= 2.0,
        cp.abs(u[1]) <= 5*np.pi/180,
        cp.abs(u[2]) <= 5*np.pi/180
    ]
    prob = cp.Problem(objective, constraints)
    prob.solve()

    return u.value[0], np.degrees(u.value[1]), np.degrees(u.value[2])

# Radial Grid for Semi-Sphere
def generate_radial_sphere_path(center, radius, rings=4, points_per_ring=12):
    path = []
    for i in range(rings + 1):
        theta = np.pi / 2 * i / rings  # from top (0) to the equator (pi/2)
        r = radius * np.sin(theta)
        z = radius * np.cos(theta)
        for j in range(points_per_ring):
            angle = 2 * np.pi * j / points_per_ring
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            pos = center + np.array([x, y, -z])
            path.append(np.concatenate([pos, [180, 0, 90]]))  # probe pointing down
    return path

# Flat Surface Snake Path
def generate_snake_path(corners, step):
    probe_width = 52.0
    p0, p1, p2 = map(lambda c: np.array(c[:3]), corners)
    o = corners[0][3:]  # Assume orientation constant

    side_vec = p1 - p0
    up_vec = p2 - p1

    total_width = np.linalg.norm(side_vec)
    total_height = np.linalg.norm(up_vec)

    width = int(np.floor(total_width / step))  # Number of points per row
    height = int(np.floor(total_height / probe_width))  # Full rows

    u_side = side_vec / np.linalg.norm(side_vec)
    u_up = up_vec / np.linalg.norm(up_vec)

    path = []

    for row in range(height + 1):
        for col in range(width + 1):
            offset = col if row % 2 == 0 else (width - col)
            point = p0 + offset * step * u_side + row * probe_width * u_up
            path.append(np.concatenate([point, o]))

    # Add final partial row if needed
    remaining_height = total_height - height * probe_width
    if remaining_height > 1e-2:  # Tolerance to avoid floating point issues
        row += 1
        for col in range(width + 1):
            offset = col if row % 2 == 0 else (width - col)
            point = p0 + offset * step * u_side + total_height * u_up
            path.append(np.concatenate([point, o]))

    return path

# Scanning
def surface_scan(model='sphere'):
    corners = calibrate_surface(model)
    center = np.mean([np.array(c[:3]) for c in corners], axis=0)
    radius = np.linalg.norm(np.array(corners[-1][:3]) - center)
    coordinates = []
    move_speed = 30
    move_mode = 1
    scanning_active = False

    if model == 'sphere':
        path = generate_radial_sphere_path(center, radius, rings=6, points_per_ring=16)
    else:
        path = generate_snake_path(corners, step=10)

    print("Starting scan...")
    for index, target in enumerate(path):
        if model == 'flat':
            if index == 0:
                move_speed = 50
            elif index % 3 == 1:
                move_speed = 40
            else:
                move_speed = 20

        mc.send_coords(target, move_speed, move_mode)
        while mc.is_moving():
            time.sleep(1/28)
            if index > 0:
                coordinates.append(mc.get_coords())
        if index == 0:
            coordinates.append(mc.get_coords())
        forces = read_force_sensors()
        current = mc.get_coords()
        tip_pos = np.array(current[:3])
        normal = -get_surface_normal(tip_pos, corners, model=model, center=center)
        dz, drx, dry = compute_qp_adjustments(forces, normal)
        adjusted = current.copy()
        adjusted[2] += dz
        adjusted[3] += drx
        adjusted[4] += dry
        mc.send_coords(adjusted, 10, 1)
        time.sleep(0.2)

    print("Scan complete")
    save_scan_data(coordinates, None, None)

def save_scan_data(data, times, forces):
    downloads_path = str(Path.home() / "Downloads")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f'scan_coords_{timestamp}.csv'
    filepath = os.path.join(downloads_path, filename)
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)

    if forces is not None and times is not None:
        with open(f'scan_forces_{timestamp}.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time', 'North', 'South', 'East', 'West'])
            for i, force in enumerate(forces):
                if force:
                    writer.writerow([times[i], force['north'], force['south'], force['east'], force['west']])

if __name__ == "__main__":
    try:
        surface_scan(model='flat')  # 'sphere' or 'flat'
    except KeyboardInterrupt:
        mc.stop()
    finally:
        ser.close()

