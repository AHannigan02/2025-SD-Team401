import csv
import os
import time
from pathlib import Path

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

#Usage
s = time.perf_counter()
time.sleep(1)
e = time.perf_counter()
name = "coords.csv"
coords = [[-175.0, -180.0, 250.0, 180.0, 0.0, 90.0], [-125.0, -180.0, 250.0, 180.0, 0.0, 90.0],
          [-75.0, -180.0, 250.0, 180.0, 0.0, 90.0], [0, 0, 0, 0, 0, 0], [e - s]]
write_csv_to_downloads(coords, name)

print(f"CSV file '{name}' created successfully.")

