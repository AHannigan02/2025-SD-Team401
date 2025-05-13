import tkinter as tk
from tkinter import messagebox
import subprocess
import os
import sys


def run_external_script():
    try:
        # Get the directory where this script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        script_path = os.path.join(script_dir, "noSensors.py")

        # Check if the script exists
        if not os.path.exists(script_path):
            messagebox.showerror("Error", f"Cannot find noSensors.py in:\n{script_dir}")
            return

        # Run the external script
        result = subprocess.run(
            [sys.executable, script_path],  # Uses the same Python interpreter
            capture_output=True,
            text=True,
            check=True
        )

        # Show output
        output = f"Script Output:\n{result.stdout}"
        if result.stderr:
            output += f"\nErrors:\n{result.stderr}"

        messagebox.showinfo("Script Completed", output)

    except subprocess.CalledProcessError as e:
        messagebox.showerror("Script Error", f"Script failed with error:\n{e.stderr}")
    except Exception as e:
        messagebox.showerror("Error", f"An error occurred: {str(e)}")


# Create the main window
root = tk.Tk()
root.title("Script Runner")
root.geometry("400x200")

# Add a label
label = tk.Label(root, text="Click the button to scan", font=('Arial', 12))
label.pack(pady=20)

# Add the run button
run_button = tk.Button(
    root,
    text="Start Scan",
    command=run_external_script,
    bg="#4CAF50",  # Green color
    fg="white",
    font=('Arial', 10, 'bold'),
    height=2,
    width=20
)
run_button.pack()

# Add an exit button
exit_button = tk.Button(
    root,
    text="Exit",
    command=root.destroy,
    bg="#f44336",  # Red color
    fg="white",
    height=1,
    width=10
)
exit_button.pack(pady=10)

# Run the application
root.mainloop()