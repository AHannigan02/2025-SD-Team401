from pymycobot.mycobot import MyCobot
import time
import os

from springTests.backupFunctions import write_csv_to_downloads

mc = MyCobot("COM3", 115200)
#mc.sync_send_angles([0, 0, 0, 0, -90, 0], 10, 5)
probeZ = 205.0
mc.set_tool_reference([0, 0, probeZ, 0, 0, 0])
mc.set_end_type(1)

data = []
def wait_for_continue_signal():
    print("Waiting for GUI input...")
    signal_file = "continue_signal.txt"
    while not os.path.exists(signal_file):
        time.sleep(0.1)
    os.remove(signal_file)

def manual_scan():
    mc.release_all_servos(1)
    slp = 1/28
    print("Position at Start then press Enter")
    wait_for_continue_signal()
    mc.stop()
    for i in range(3,0,-1):
        print("Scanning in ",i)
        time.sleep(1)
    mc.release_all_servos(1)
    t = time.perf_counter()
    while time.perf_counter() - t < 10:
        data.append(mc.get_coords())
        time.sleep(slp)
    mc.stop()
    print("Stop!")
    save_man_scan_data(data)

def save_man_scan_data(coords):
    # Convert data to CSV format and save
    import os
    import csv
    from datetime import datetime
    from pathlib import Path
    # Find downloads path
    downloads_path = str(Path.home() / "Downloads")
    # Create the directory if it doesn't exist

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f'manual_scan_coords_{timestamp}.csv'

    # Construct the full file path
    filepath = os.path.join(downloads_path, filename)

    # Write the data to the CSV file
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(coords)

if __name__ == "__main__":
    try:
        manual_scan()
    except KeyboardInterrupt:
        print("Scan interrupted by user")
        mc.stop()
