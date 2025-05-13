import tkinter as tk
from tkinter import messagebox, scrolledtext
import subprocess
import threading
import os

SCRIPTS = {
    "Run Ultrasound Scan": "QP_controller.py",
    "Visualize Volume (MATLAB)": "ultrasoundGeneration.m",
    "Handheld Scan": "manualScan.py"
}

def run_script(script_name, output_box, status_label):
    def task():
        output_box.delete('1.0', tk.END)
        status_label.config(text=f"Running: {script_name}")
        try:
            if script_name.endswith('.m'):
                cmd = ['matlab', '-batch', f"{os.path.splitext(script_name)[0]}"]
            else:
                cmd = [r'C:\Users\ahann\venv\robotics\Scripts\python.exe', script_name]
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

            def stream_output():
                for line in process.stdout:
                    output_box.insert(tk.END, line)
                    output_box.see(tk.END)
                for line in process.stderr:
                    output_box.insert(tk.END, line)
                    output_box.see(tk.END)
                process.wait()
                status_label.config(text="Completed" if process.returncode == 0 else "Error")
            threading.Thread(target=stream_output, daemon=True).start()
        except Exception as e:
            output_box.insert(tk.END, f"Execution failed: {e}\n")
            status_label.config(text="Execution failed.")

    threading.Thread(target=task, daemon=True).start()

def create_continue_signal():
    with open("continue_signal.txt", "w") as f:
        f.write("continue")

def create_ui():
    window = tk.Tk()
    window.title("Ultrasound Project Launcher")
    window.geometry("600x600")
    window.configure(bg="#f0f0f0")

    tk.Label(window, text="Ultrasound Project Scripts", font=("Arial", 16), bg="#f0f0f0").pack(pady=10)

    status_label = tk.Label(window, text="Ready", fg="green", bg="#f0f0f0")
    status_label.pack()

    output_box = scrolledtext.ScrolledText(window, width=70, height=15)
    output_box.pack(padx=10, pady=10)

    for label, script in SCRIPTS.items():
        tk.Button(window, text=label, width=40, command=lambda s=script: run_script(s, output_box, status_label)).pack(pady=5)

    tk.Button(window, text="Send Enter Signal", bg="#007bff", fg="white", command=create_continue_signal).pack(pady=5)
    tk.Button(window, text="Exit", bg="#dc3545", fg="white", command=window.quit).pack(pady=10)

    window.mainloop()

if __name__ == "__main__":
    create_ui()
