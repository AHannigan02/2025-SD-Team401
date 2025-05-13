from pymycobot.mycobot import MyCobot
import serial
import numpy as np
import time
from simple_pid import PID

# Initialize MyCobot
mc = MyCobot("COM3", 115200)
mc.set_fresh_mode(1)  # Enable fresh mode for smoother movements

# Initialize force sensor serial connection
ser = serial.Serial('COM5', 115200, timeout=0.1)
np.set_printoptions(legacy='1.25')

# Probe dimensions
probeZ = 205.0  # Total 3D part length in mm
shellZ = 135.0 # Total height from FSRs to tip of probe
probeX = 52.0  # Probe spacing in mm

# Setup tool reference
mc.set_tool_reference([0, 0, probeZ, 0, 0, 0])
mc.set_end_type(1)
"""
# Raw force sensor thresholds
CONTACT_THRESHOLD = 100  # Minimum force to consider contact
NS_THRESHOLD = 400  # Force difference threshold for North to South rotating adjustment
EW_THRESHOLD = 150 # Force difference threshold for East to West rotating adjustment
MAX_FORCE = 700  # Maximum allowed force before retracting
"""
# Newton Thresholds
CONTACT_THRESHOLD = 1.0  # Minimum force to consider contact
NS_THRESHOLD = 0.5  # Force difference threshold for North to South rotating adjustment
EW_THRESHOLD = 0.5 # Force difference threshold for East to West rotating adjustment
MAX_FORCE = 4.0  # Maximum allowed force before retracting

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
    weight = np.array([0, .017, .020, .039, .064, 0.177, 0.608])  # Weight in kilograms
    force = weight * 9.81  # Force in N
    adc = np.array([0, 63, 122, 289, 450, 480, 1023])  # Raw ADC values
    coefficients = np.polyfit(adc, force, 2)  # Fit a 2nd degree polynomial
    data = ser.readline().decode().strip()
    # Convert raw data from 5100 ohm resistor to Newtons
    if data:
        parts = list(map(float, data.split()))
        if len(parts) >= 4:
            return {'north': np.round(np.polyval(coefficients, parts[0]),2), 'east': np.round(np.polyval(coefficients, parts[3]),2),
                    'south': np.round(np.polyval(coefficients, parts[2]),2), 'west': np.round(np.polyval(coefficients, parts[1]),2)}

    return None


# Initialize PID controllers for roll (Rx) and pitch (Ry)
pid_rx = PID(Kp=0.5, Ki=0.01, Kd=0.1, setpoint=0)  # Tune these gains!
pid_ry = PID(Kp=0.5, Ki=0.01, Kd=0.1, setpoint=0)

# Set output limits (degrees)
pid_rx.output_limits = (-10, 10)  # Max ±10° adjustment
pid_ry.output_limits = (-10, 10)


def adjust_orientation(current_coords, forces):
    """Adjusts Rx/Ry to balance forces using PID control."""
    # Calculate force errors (normalized)
    vert_error = (forces['north'] - forces['south']) / (forces['north'] + forces['south'] + 1e-6)
    horiz_error = (forces['east'] - forces['west']) / (forces['east'] + forces['west'] + 1e-6)

    # Update PID controllers
    rx_adjustment = pid_rx(vert_error)
    ry_adjustment = pid_ry(horiz_error)

    # Apply adjustments
    new_rx = current_coords[3] + rx_adjustment
    new_ry = current_coords[4] + ry_adjustment

    return new_rx, new_ry
"""
# Adjust orientation based on force feedback
def adjust_orientation(current_coords, forces):
    rx, ry = current_coords[3], current_coords[4]
    adjustment = 5.0  # Degrees of adjustment per step

    # Calculate force differences
    vert_diff = forces['north'] - forces['south'] / shellZ
    horiz_diff = forces['east'] - forces['west'] / shellZ

    # Adjust roll (rx) based on vertical force difference
    if abs(vert_diff) > NS_THRESHOLD:
        rx += -np.sign(vert_diff) * adjustment

    # Adjust pitch (ry) based on horizontal force difference
    if abs(horiz_diff) > EW_THRESHOLD:
        ry += -np.sign(horiz_diff) * adjustment

    return rx, ry
"""

# Adjust Z position based on any reading being too high
def adjust_z_position(current_coords, forces):
    z = current_coords[2]
    z_step = 1.0  # Millimeter adjustment step
    count = 0
    for values in forces.values():
        if values > MAX_FORCE:
            return z + z_step  # Move away from surface
        elif values < CONTACT_THRESHOLD:
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
    num_rows = int(mag_up // step_size)
    last_row = mag_up % step_size
    steps_per_row = int(mag_side // step_size)

    # Initialize path
    path = []
    current_pos = start.copy()
    orientation = corners[0][3:]  # Initial orientation

    # Generate snake pattern
    for row in range(num_rows + 1):
        # Determine direction for this row (alternating)
        direction = 1 if row % 2 == 0 else -1

        # Calculate number of steps for this row
        if row < num_rows:
            row_steps = steps_per_row
            row_step_size = step_size
        else:
            row_steps = 1  # Last partial row
            row_step_size = last_row

        # Add row steps to path
        for step in range(row_steps + 1):
            if step == 0:
                # First step in row - just add current position
                pass
            else:
                # Move in side direction
                current_pos += direction * (row_step_size / row_steps) * v_side

            # Add position with current orientation
            path.append(np.concatenate([current_pos, orientation]))

        # Move up (except after last row)
        if row < num_rows:
            current_pos += step_size * u_up
    print(path)
    print(corners)
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
            move_speed = 50
            timeout = 10
        elif index % 3 == 1:
            move_speed = 30
            timeout = 3
        else:
            move_speed = 10
            timeout = 5
        t = time.perf_counter()
        mc.send_coords(target, move_speed, move_mode)

        # Movement control loop
        while t - time.perf_counter() < timeout:
            # Read force sensors
            forces = read_force_sensors()
            #print(forces)
            # Get current position
            current = mc.get_coords()
            #print(current)
            # Check if we've reached first position
            if not mc.is_moving():
                mc.send_coords(target, move_speed, move_mode)
            elif np.allclose(target[:3], current[:3], atol=1.0):
                print("Point reached!")
                break

            # Only process force data if we have valid readings
            if forces:
                # Check if reached first corner
                if not scanning_active and np.allclose(path[0][:3], current[:3], atol=1.0):
                    scanning_active = True
                    print("Contact established - scanning started")

                if scanning_active:
                    # Adjust position based on force feedback
                    new_z = adjust_z_position(current, forces)
                    new_rx, new_ry = adjust_orientation(current, forces)

                    # Create adjusted coordinates
                    adjusted = current.copy()
                    adjusted[2] = new_z
                    adjusted[3] = new_rx
                    adjusted[4] = new_ry

                    # Send adjustment
                    mc.send_coords(adjusted, 20, move_mode)

                    # Record data
                    real_coords.append(adjusted)
                    scan_time.append(time.perf_counter() - start_time)
                    force_data.append(forces)
                else:
                    # Not scanning yet - move closer to surface
                    mc.send_coord(3, current[2] - 1, 20)
                    not_scanning.append(time.perf_counter() - start_time)


            time.sleep(0.05)  # Control loop frequency

    # Record final position
    current = mc.get_coords()
    real_coords.append(current)
    scan_time.append(time.perf_counter() - start_time)
    if forces:
        force_data.append(forces.values())

    print("=== Scan Complete ===")
    mc.sync_send_angles([0, 0, 0, 0, -90, 0], 40)

    # Save data
    save_scan_data(real_coords, scan_time, force_data, not_scanning)


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
    #except Exception as e:
     #   print(f"Error during scan: {e}")
        #mc.sync_send_angles([0, 0, 0, 0, -90, 0], 20)
    finally:
        ser.close()