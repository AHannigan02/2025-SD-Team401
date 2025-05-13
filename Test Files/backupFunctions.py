from pymycobot.mycobot import MyCobot
import serial
import csv
import os
import time
import math
from pathlib import Path
import numpy as np

def is_close(coord1, coord2, tolerance=1):
    """
    Checks if two coordinates are within a specified tolerance of each other.

    Args:
        coord1 (tuple): The first coordinate (x, y, z).
        coord2 (tuple): The second coordinate (x, y, z).
        tolerance (float): The maximum allowed difference between the coordinates.

    Returns:
        bool: True if the coordinates are within the tolerance, False otherwise.
    """
    distance = math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2 + (coord1[2] - coord2[2]) ** 2)
    return distance <= tolerance

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
