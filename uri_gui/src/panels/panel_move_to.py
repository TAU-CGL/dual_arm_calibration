import tkinter as tk
import tkinter.ttk as ttk
import sys
from pathlib import Path
import pyperclip
import ast
import json

import uri_if

# Add the calibration directory to Python path
calibration_dir = Path(__file__).resolve().parents[2] / "calibration"
if calibration_dir not in sys.path:
    sys.path.insert(0, str(calibration_dir))
import gemini_calc_v2 as gemini

from uri_gui.src.config import *
from uri_gui.src.control import *

# Calibration persistence
CALIBRATION_FILE = Path(__file__).resolve().parent.parent / "calibration.json"

def save_calibration(calibration_pose):
    """Save calibration to persistent storage."""
    try:
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(list(calibration_pose), f)
    except Exception as e:
        print(f"Failed to save calibration: {e}")

def load_calibration():
    """Load calibration from persistent storage."""
    try:
        if CALIBRATION_FILE.exists():
            with open(CALIBRATION_FILE, 'r') as f:
                data = json.load(f)
                return tuple(float(v) for v in data) if len(data) == 6 else None
    except Exception as e:
        print(f"Failed to load calibration: {e}")
    return None


def add_move_to_panel(root, parent=None):
    if parent is None:
        parent = root
    frame = tk.Frame(parent, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)

    # Panel label
    tk.Label(frame, text="Move To:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    # Buttons
    calibrate_button = tk.Button(frame, text="Calibrate", font=DEFAULT_FONT)
    calibrate_button.pack(side=tk.LEFT, padx=(0, 3))

    paste_cali_button = tk.Button(frame, text="Paste_Cali", font=DEFAULT_FONT)
    paste_cali_button.pack(side=tk.LEFT, padx=(0, 3))

    move_uri2ayal_button = tk.Button(frame, text="Move_URI->AYAL", font=DEFAULT_FONT)
    move_uri2ayal_button.pack(side=tk.LEFT, padx=(0, 3))

    move_ayal2uri_button = tk.Button(frame, text="Move_AYAL->URI", font=DEFAULT_FONT)
    move_ayal2uri_button.pack(side=tk.LEFT, padx=(0, 3))

    print_cali_button = tk.Button(frame, text="Print_Cali", font=DEFAULT_FONT)
    print_cali_button.pack(side=tk.LEFT, padx=(0, 3))

    tk.Label(frame, text="  Offset:", font=DEFAULT_FONT).pack(side=tk.LEFT, padx=(10, 0))
    tk.Label(frame, text="X:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    offset_x_entry = tk.Entry(frame, width=6, font=DEFAULT_FONT)
    offset_x_entry.insert(0, "0")
    offset_x_entry.pack(side=tk.LEFT, padx=(0, 5))
    tk.Label(frame, text="Y:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    offset_y_entry = tk.Entry(frame, width=6, font=DEFAULT_FONT)
    offset_y_entry.insert(0, "0")
    offset_y_entry.pack(side=tk.LEFT, padx=(0, 5))
    tk.Label(frame, text="Z:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    offset_z_entry = tk.Entry(frame, width=6, font=DEFAULT_FONT)
    offset_z_entry.insert(0, "0")
    offset_z_entry.pack(side=tk.LEFT, padx=(0, 5))

    # Background fields exposed on root
    root.move_to_calibrate_ack = False
    root.move_to_calibration_pose = None
    
    # Load saved calibration if available
    saved_calibration = load_calibration()
    if saved_calibration is not None:
        root.move_to_calibration_pose = saved_calibration
        root.move_to_calibrate_ack = True
        print("Loaded calibration from storage:", root.move_to_calibration_pose)

    def update_button_colors():
        color = 'green' if root.move_to_calibrate_ack else 'red'
        move_uri2ayal_button.config(bg=color)
        move_ayal2uri_button.config(bg=color)

    def toggle_conn_press(uri_name):
        """Toggle connection for specified robot (uri_name: 'uri1' or 'uri2')"""
        uri_host = URI1_HOST if uri_name == "uri1" else URI2_HOST
        
        if not hasattr(root, uri_name) or getattr(root, uri_name) is None:
            setattr(root, uri_name, uri_if.RMPLAB_Uri(uri_host))
            getattr(root, uri_name).teachmode = False
        
        toggle_connect(getattr(root, uri_name), calibrate=False)

    def calibrate_press():
        # 1. connect both
        toggle_conn_press("uri1")
        toggle_conn_press("uri2")

        # 2. get tcp pose from gemini (same as panel_calibrate)
        try:
            relative_pose = gemini.calculate_ayal_in_uri(root.uri1.recieve.getActualTCPPose(), root.uri2.recieve.getActualTCPPose())
        except Exception as e:
            print(f"Calibrate failed: {e}")
            toggle_conn_press("uri1")
            toggle_conn_press("uri2")
            return

        if relative_pose is None:
            print("Calibrate: no result")
            toggle_conn_press("uri1")
            toggle_conn_press("uri2")
            return

        # store calibration (uri2 in uri1 frame)
        root.move_to_calibration_pose = tuple(float(v) for v in relative_pose)
        root.move_to_calibrate_ack = True
        save_calibration(root.move_to_calibration_pose)
        update_button_colors()
        print("Calibration acquired:", root.move_to_calibration_pose)

        # 3. disconnect
        toggle_conn_press("uri1")
        toggle_conn_press("uri2")

    def paste_cali_press():
        s = pyperclip.paste()
        try:
            vals = eval(s)
            if not isinstance(vals, (list, tuple)) or len(vals) != 6:
                raise ValueError("paste must be list/tuple of 6 numbers")
            vals = tuple(float(v) for v in vals)
        except Exception as e:
            print(f"Paste_Cali failed: {e}")
            return
        root.move_to_calibration_pose = vals
        root.move_to_calibrate_ack = True
        save_calibration(root.move_to_calibration_pose)
        update_button_colors()
        print("Calibration (from clipboard) set:", root.move_to_calibration_pose)

    def get_offset():
        """Read X/Y/Z offset entries, defaulting to 0."""
        def safe(e):
            try:
                return float(e.get())
            except ValueError:
                return 0.0
        return safe(offset_x_entry), safe(offset_y_entry), safe(offset_z_entry)

    def move_ayal2uri_press():
        # 1. connect
        toggle_conn_press("uri1")
        toggle_conn_press("uri2")

        if not root.move_to_calibrate_ack:
            print("Move_AYAL->URI: calibration not acknowledged")
            toggle_conn_press("uri1")
            toggle_conn_press("uri2")
            return

        try:
            P_source = root.uri1.recieve.getActualTCPPose()
            P_target = list(gemini.calculate_mirror_position(P_source, root.move_to_calibration_pose, flip_trans=True))
            ox, oy, oz = get_offset()
            P_target[0] += ox
            P_target[1] += oy
            P_target[2] += oz
            # move uri2 to corresponding pose
            tcp_movej(root.uri2, *P_target)
            print("Moved URI2 to:", P_target)
        except Exception as e:
            print(f"Move_AYAL->URI failed: {e}")

        # 3. disconnect
        toggle_conn_press("uri1")
        toggle_conn_press("uri2")

    def move_uri2ayal_press():
        # 1. connect
        toggle_conn_press("uri1")
        toggle_conn_press("uri2")

        if not root.move_to_calibrate_ack:
            print("Move_URI->AYAL: calibration not acknowledged")
            toggle_conn_press("uri1")
            toggle_conn_press("uri2")
            return

        try:
            P_source = root.uri2.recieve.getActualTCPPose()
            P_target = list(gemini.calculate_mirror_position(P_source, root.move_to_calibration_pose))
            ox, oy, oz = get_offset()
            P_target[0] += ox
            P_target[1] += oy
            P_target[2] += oz
            # move uri1 to corresponding pose
            tcp_movej(root.uri1, *P_target)
            print("Moved URI1 to:", P_target)
        except Exception as e:
            print(f"Move_URI->AYAL failed: {e}")

        # 3. disconnect
        toggle_conn_press("uri1")
        toggle_conn_press("uri2")

    def print_cali_press():
        if root.move_to_calibration_pose is not None:
            print("Current calibration:", root.move_to_calibration_pose)
        else:
            print("No calibration loaded")

    # Wire buttons
    calibrate_button.config(command=calibrate_press)
    paste_cali_button.config(command=paste_cali_press)
    move_uri2ayal_button.config(command=move_uri2ayal_press)
    move_ayal2uri_button.config(command=move_ayal2uri_press)
    print_cali_button.config(command=print_cali_press)

    # initial color update
    update_button_colors()
    
    return frame
