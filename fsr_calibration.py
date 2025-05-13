import numpy as np
import serial
import time
import joblib
from sklearn.linear_model import LinearRegression

# --- Serial setup ---
ser = serial.Serial('COM5', 115200, timeout=0.1)
time.sleep(2)

# --- Constants ---
FSR_POSITIONS = {
    'north': np.array([0, -45, -135]),  # mm
    'south': np.array([0,  45, -135]),
    'east':  np.array([ 45, 0, -135]),
    'west':  np.array([-45, 0, -135]),
}
g = 9.81  # gravity

# --- Helper function to read FSR data ---
def read_force_sensors():
    while True:
        data = ser.readline().decode().strip()
        if data:
                parts = list(map(float, data.split()))
                return {'north': parts[0],
                    'east':  parts[1],
                    'south': parts[2],
                    'west':  parts[3]}

# --- Collect one sample of FSR readings ---
def collect_sample(n_avg=10):
    samples = []
    for _ in range(n_avg):
        sample = read_force_sensors()
        vec = [sample['north'], sample['south'], sample['east'], sample['west']]
        samples.append(vec)
        time.sleep(0.05)
    return np.mean(samples, axis=0)

# --- Build training data ---
X = []
Y = []

# === 📏 Calibration Steps ===
# You will need to manually apply known weights and follow the prompts

# Define calibration cases
calibration_steps = [
    {"desc": "Center force", "pos": np.array([0, 0, 0]), "mass": 0.2},
    {"desc": "East torque",  "pos": np.array([60, 0, 0]), "mass": 0.2},
    {"desc": "West torque",  "pos": np.array([-60, 0, 0]), "mass": 0.2},
    {"desc": "North torque", "pos": np.array([0, -60, 0]), "mass": 0.2},
    {"desc": "South torque", "pos": np.array([0, 60, 0]), "mass": 0.2},
    {"desc": "No load",      "pos": np.array([0, 0, 0]), "mass": 0.0}
]

for step in calibration_steps:
    input(f"Place {step['mass']} kg at: {step['desc']} and press Enter.")
    fsr_vec = collect_sample()

    # Target output
    Fz = step['mass'] * g
    r = step['pos']  # position vector in mm
    F = np.array([0, 0, Fz])
    M = np.cross(r, F)  # Mx, My, Mz

    target = [Fz, M[0], M[1]]
    print(f"Read FSR: {fsr_vec} → Target: {target}")

    X.append(fsr_vec)
    Y.append(target)

# --- Fit Linear Model ---
X = np.array(X)
Y = np.array(Y)
model = LinearRegression()
model.fit(X, Y)

# --- Save model ---
joblib.dump(model, "fsr_force_model.pkl")
print("Calibration complete. Model saved to fsr_force_model.pkl")

# --- Optional: test prediction ---
test = collect_sample()
predicted = model.predict([test])[0]
print(f"\nTest sample FSR: {test}")
print(f"→ Predicted: Fz={predicted[0]:.2f} N, Mx={predicted[1]:.2f} N·mm, My={predicted[2]:.2f} N·mm")
