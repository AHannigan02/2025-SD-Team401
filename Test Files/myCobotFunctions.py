from pymycobot.mycobot import MyCobot
import serial
import csv
import os
import time
from pathlib import Path
import numpy as np

# Importing this initializes MyCobot
mc = MyCobot("COM3", 115200)

def calibrate320():
    mc.set_encoders([2048, 2048, 2048, 2048, 2048, 2048], 50)
    # mc.release_all_servos()
    # mc.set_tool_reference([0, 0, 152.0, 0, 0, 0])
    # mc.set_end_type(1)
    time.sleep(1)
    print(mc.get_coords())

def set_probe_frame():
    mc.set_tool_reference([0, 0, 152, 0, 0, 0])
    mc.set_end_type(1)

def write_csv_to_downloads(data, filename):
    """Writes CSV data to a specified directory.

    Args:
        data (list of lists): The data to write to the CSV file.
        filename (str): The name of the CSV file."""

    # Find downloads path
    downloads_path = str(Path.home() / "Downloads")
    # Create the directory if it doesn't exist
    if not os.path.exists(downloads_path):
        print("Could not find Downloads path")

    # Construct the full file path
    filepath = os.path.join(downloads_path, filename)

    # Write the data to the CSV file
    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)

#if __name__ == '__main__':