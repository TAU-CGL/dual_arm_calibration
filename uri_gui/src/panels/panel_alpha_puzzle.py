import tkinter as tk
import tkinter.ttk as ttk
import sys
import importlib.util
from pathlib import Path
import threading

import uri_if

from uri_gui.src.config import *
from uri_gui.src.control import *

# Add the alpha_puzzle directory to Python path
alpha_puzzle_dir = Path(__file__).resolve().parents[2] / "alpha_puzzle"
if str(alpha_puzzle_dir) not in sys.path:
    sys.path.insert(0, str(alpha_puzzle_dir))

dismantle_puzzle_path = alpha_puzzle_dir / "dismantle_puzzle.py"
dismantle_spec = importlib.util.spec_from_file_location("dismantle_puzzle", dismantle_puzzle_path)
if dismantle_spec is None or dismantle_spec.loader is None:
    raise ImportError(f"Cannot load module from {dismantle_puzzle_path}")
dismantle_puzzle = importlib.util.module_from_spec(dismantle_spec)
dismantle_spec.loader.exec_module(dismantle_puzzle)

# Host IPs (should match dismantle_puzzle.py)
URI_HOST = "192.168.56.101"
AYAL_HOST = "192.168.57.101"


def add_alpha_puzzle_panel(root, parent=None):
    if parent is None:
        parent = root
    frame = tk.Frame(parent, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)

    # Label
    tk.Label(frame, text="Alpha Puzzle: ", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    # Buttons
    prepare_button = tk.Button(frame, text="Prepare", font=DEFAULT_FONT)
    prepare_button.pack(side=tk.LEFT, padx=(0, 3))

    forward_button = tk.Button(frame, text="Forward", font=DEFAULT_FONT)
    forward_button.pack(side=tk.LEFT, padx=(0, 3))

    reversed_button = tk.Button(frame, text="Reversed", font=DEFAULT_FONT)
    reversed_button.pack(side=tk.LEFT, padx=(0, 3))

    finish_button = tk.Button(frame, text="Finish", font=DEFAULT_FONT)
    finish_button.pack(side=tk.LEFT, padx=(0, 3))

    # Repeat cycles section
    tk.Label(frame, text="Cycles: ", font=DEFAULT_FONT).pack(side=tk.LEFT, padx=(20, 0))
    cycles_entry = tk.Entry(frame, width=3, font=DEFAULT_FONT)
    cycles_entry.pack(side=tk.LEFT, padx=(0, 3))
    cycles_entry.insert(0, "1")

    repeat_button = tk.Button(frame, text="Fwd->Rev", font=DEFAULT_FONT)
    repeat_button.pack(side=tk.LEFT, padx=(0, 3))

    # Status label
    status_label = tk.Label(frame, text="Ready", font=DEFAULT_FONT, fg="green")
    status_label.pack(side=tk.LEFT, padx=(20, 0))

    # Background fields
    root.puzzle_uri = None
    root.puzzle_ayal = None
    root.puzzle_running = False

    def connect_robots():
        """Connect to both robots"""
        try:
            if root.puzzle_uri is None:
                root.puzzle_uri = uri_if.RMPLAB_Uri(URI_HOST)
                root.puzzle_uri.connect(False)
            if root.puzzle_ayal is None:
                root.puzzle_ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
                root.puzzle_ayal.connect(False)
            return True
        except Exception as e:
            print(f"Failed to connect robots: {e}")
            status_label.config(text=f"Error: {e}", fg="red")
            return False

    def disconnect_robots():
        """Disconnect from both robots"""
        try:
            if root.puzzle_uri is not None:
                root.puzzle_uri.disconnect()
                root.puzzle_uri = None
            if root.puzzle_ayal is not None:
                root.puzzle_ayal.disconnect()
                root.puzzle_ayal = None
        except Exception as e:
            print(f"Failed to disconnect robots: {e}")

    def update_button_states(enabled):
        """Enable/disable all buttons"""
        state = tk.NORMAL if enabled else tk.DISABLED
        prepare_button.config(state=state)
        forward_button.config(state=state)
        reversed_button.config(state=state)
        finish_button.config(state=state)

    def run_command(command_name, command_func):
        """Run a command in a background thread"""
        def thread_func():
            update_button_states(False)
            status_label.config(text="Running...", fg="blue")
            root.update()
            try:
                if connect_robots():
                    command_func()
                    status_label.config(text="Complete", fg="green")
                    print(f"{command_name} completed successfully")
                else:
                    status_label.config(text="Failed", fg="red")
            except KeyboardInterrupt:
                print("Interrupted!")
                if root.puzzle_uri:
                    root.puzzle_uri.control.servoStop()
                    root.puzzle_uri.control.stopScript()
                if root.puzzle_ayal:
                    root.puzzle_ayal.control.servoStop()
                    root.puzzle_ayal.control.stopScript()
                status_label.config(text="Stopped", fg="orange")
            except Exception as e:
                print(f"{command_name} failed: {e}")
                status_label.config(text=f"Failed: {e}", fg="red")
            finally:
                disconnect_robots()
                update_button_states(True)

        thread = threading.Thread(target=thread_func, daemon=True)
        thread.start()

    def prepare_press():
        run_command("Prepare", lambda: dismantle_puzzle.prepare_robots(root.puzzle_uri, root.puzzle_ayal))

    def forward_press():
        alpha_puzzle_paths = alpha_puzzle_dir / "paths" / "alphaZ"
        run_command("Forward", lambda: dismantle_puzzle.forward_trajectory(
            root.puzzle_uri, root.puzzle_ayal,
            dismantle_puzzle.csv_to_arm_path(str(alpha_puzzle_paths / "improved_dynamic_ik_trajectory.csv")),
            dismantle_puzzle.csv_to_arm_path(str(alpha_puzzle_paths / "improved_static_ik_trajectory.csv"))
        ))

    def reversed_press():
        alpha_puzzle_paths = alpha_puzzle_dir / "paths" / "alphaZ"
        run_command("Reversed", lambda: dismantle_puzzle.reversed_trajectory(
            root.puzzle_uri, root.puzzle_ayal,
            dismantle_puzzle.csv_to_arm_path(str(alpha_puzzle_paths / "improved_dynamic_ik_trajectory.csv")),
            dismantle_puzzle.csv_to_arm_path(str(alpha_puzzle_paths / "improved_static_ik_trajectory.csv"))
        ))

    def finish_press():
        run_command("Finish", lambda: dismantle_puzzle.finish_robots(root.puzzle_uri, root.puzzle_ayal))

    def repeat_cycles_press():
        try:
            cycles = int(cycles_entry.get())
            if cycles < 1:
                print("Cycles must be >= 1")
                return
        except ValueError:
            print("Invalid cycles value")
            return

        alpha_puzzle_paths = alpha_puzzle_dir / "paths" / "alphaZ"
        dynamic_traj = dismantle_puzzle.csv_to_arm_path(str(alpha_puzzle_paths / "improved_dynamic_ik_trajectory.csv"))
        static_traj = dismantle_puzzle.csv_to_arm_path(str(alpha_puzzle_paths / "improved_static_ik_trajectory.csv"))

        def cycle_sequence():
            for cycle in range(cycles):
                print(f"--- Cycle {cycle + 1}/{cycles} ---")
                dismantle_puzzle.forward_trajectory(root.puzzle_uri, root.puzzle_ayal, dynamic_traj, static_traj)
                dismantle_puzzle.reversed_trajectory(root.puzzle_uri, root.puzzle_ayal, dynamic_traj, static_traj)

        run_command(f"Repeat ({cycles}x)", cycle_sequence)

    # Wire buttons
    prepare_button.config(command=prepare_press)
    forward_button.config(command=forward_press)
    reversed_button.config(command=reversed_press)
    finish_button.config(command=finish_press)
    repeat_button.config(command=repeat_cycles_press)
    
    return frame
