from pymycobot.mycobot import MyCobot
import serial
import numpy as np
import time

# Initialize MyCobot
mc = MyCobot("COM3", 115200)
mc.set_fresh_mode(1)  # Enable fresh mode for smoother movements
mc.focus_servo(5)
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
NS_THRESHOLD = 0.5  # Force difference threshold for North to South rotating adjustment
EW_THRESHOLD = 0.5 # Force difference threshold for East to West rotating adjustment
NS_MAX = 5.5  # Maximum allowed force before retracting
EW_MAX = 4.0

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
        parts = np.round(np.polyval(coefficients, parts) - 3.0, 2)
        return {'north': parts[0], 'east': parts[1],
                'south': parts[2], 'west': parts[3]}


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
    vert_adjustment = 0.1  # Degrees of adjustment per step
    horiz_adjustment = 0.1  # Degrees of adjustment per step

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

# Main control loop
def surface_scan():
    # Scan parameters
    move_speed = 20
    move_mode = 1  # 0 for angular, 1 for linear

    # Movement control loop
    scanning_active = False
    mc.set_fresh_mode(1)
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

            time.sleep(.1)  # Control loop frequency

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
        mc.stop()
        #mc.sync_send_angles([0, 0, 0, 0, -90, 0], 20)
    except Exception as e:
        print(f"Error during scan: {e}")
        #mc.sync_send_angles([0, 0, 0, 0, -90, 0], 20)
    finally:
        ser.close()
