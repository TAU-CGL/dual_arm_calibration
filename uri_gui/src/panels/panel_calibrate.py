import tkinter as tk 
import tkinter.ttk as ttk
import sys
from pathlib import Path
import json
import pyperclip

import uri_if

# Add the calibration directory to Python path
calibration_dir = Path(__file__).resolve().parents[2] / "calibration"
if calibration_dir not in sys.path:
    sys.path.insert(0, str(calibration_dir))
import gemini_calc_v2 as gemini

# Calibration persistence file
CALIBRATION_FILE = Path(__file__).resolve().parent.parent / "calibration.json"

# Calibration samples file (for optimal calibration)
CALIBRATION_SAMPLES_FILE = Path(__file__).resolve().parent.parent / "calibration_samples.json"

from uri_gui.src.config import *
from uri_gui.src.control import *

def add_calibrate_panel(root, parent=None):

    if parent is None:
        parent = root
    frame = tk.Frame(parent, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)

    # Panel label
    tk.Label(frame, text="Calibrate:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    # data entries
    tk.Label(frame, text="X [m]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    x_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
    x_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    tk.Label(frame, text="Y [m]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    y_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
    y_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    tk.Label(frame, text="Z [m]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    z_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
    z_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    tk.Label(frame, text="RX [rad]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    rx_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
    rx_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    tk.Label(frame, text="RY [rad]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    ry_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
    ry_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    tk.Label(frame, text="RZ [rad]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    rz_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
    rz_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    #buttons
    calibrate_button = tk.Button(frame, text="Calibrate", font=DEFAULT_FONT)
    calibrate_button.pack(side=tk.LEFT, padx=(0, 3))
    
    print_tcp_button = tk.Button(frame, text="Print TCP", font=DEFAULT_FONT)
    print_tcp_button.pack(side=tk.LEFT, padx=(0, 3))
    
    copy_button = tk.Button(frame, text="Copy", font=DEFAULT_FONT)
    copy_button.pack(side=tk.LEFT, padx=(0, 3))

    teach_both_button = tk.Button(frame, text="Start Teach Both", font=DEFAULT_FONT)
    teach_both_button.pack(side=tk.LEFT, padx=(0, 3))

    base_both_button = tk.Button(frame, text="Base Both", font=DEFAULT_FONT)
    base_both_button.pack(side=tk.LEFT, padx=(0, 3))

    add_sample_button = tk.Button(frame, text="Add Sample", font=DEFAULT_FONT)
    add_sample_button.pack(side=tk.LEFT, padx=(0, 3))

    guide2sample_button = tk.Button(frame, text="Guide2Sample Off", font=DEFAULT_FONT)
    guide2sample_button.pack(side=tk.LEFT, padx=(0, 3))

    def toggle_conn_press():
        # Only create new Uri objects if they don't exist
        if not hasattr(root, 'uri1') or root.uri1 is None:
            root.uri1 = uri_if.RMPLAB_Uri(URI1_HOST)
            root.uri1.teachmode = False
        toggle_connect(root.uri1, calibrate=False)
        
        if not hasattr(root, 'uri2') or root.uri2 is None:
            root.uri2 = uri_if.RMPLAB_Uri(URI2_HOST)
            root.uri2.teachmode = False
        toggle_connect(root.uri2, calibrate=False)

    def ensure_connected():
        """Connect both robots if not already connected (no toggling)."""
        if not hasattr(root, 'uri1') or root.uri1 is None:
            root.uri1 = uri_if.RMPLAB_Uri(URI1_HOST)
            root.uri1.teachmode = False
        if not root.uri1.is_connected():
            toggle_connect(root.uri1, calibrate=False)
        
        if not hasattr(root, 'uri2') or root.uri2 is None:
            root.uri2 = uri_if.RMPLAB_Uri(URI2_HOST)
            root.uri2.teachmode = False
        if not root.uri2.is_connected():
            toggle_connect(root.uri2, calibrate=False)

    def calibrate_press():
        print("calibrate_press:: root.in_teach_mode_both:", root.in_teach_mode_both)
        # 1. connect
        in_teach_mode = root.in_teach_mode_both
        if in_teach_mode:
            teach_both_press() 
        toggle_conn_press()

        # 2. get tcp pose from gemini
        relative_pose = gemini.calculate_ayal_in_uri(root.uri1.recieve.getActualTCPPose(), root.uri2.recieve.getActualTCPPose())
        if relative_pose is None:
            return
        x, y, z, rx, ry, rz = relative_pose
        set_text(x_entry, "{:.6f}".format(x))
        set_text(y_entry, "{:.6f}".format(y))
        set_text(z_entry, "{:.6f}".format(z))
        set_text(rx_entry, "{:.6f}".format(rx))
        set_text(ry_entry, "{:.6f}".format(ry))
        set_text(rz_entry, "{:.6f}".format(rz))
        

        # 3. copy to clipboard
        q_copy = [float(x_entry.get()), float(y_entry.get()), float(z_entry.get()), float(rx_entry.get()), float(ry_entry.get()), float(rz_entry.get())]
        #Covert the list of joints poses to string and copy to clipboard
        pyperclip.copy(str(q_copy))
        print("Calibration result copied to clipboard.")
        
        #  4. disconnect
        toggle_conn_press()
        if in_teach_mode:
            teach_both_press()
        

    # State for periodic TCP printing
    root._print_tcp_active = False
    root._print_tcp_after_id = None
    TCP_LOG_FILE = Path(__file__).resolve().parent.parent / "tcp_positions.json"

    def print_tcp_tick():
        """Print TCP poses once, append to JSON, and schedule next tick."""
        try:
            tcp1 = root.uri1.recieve.getActualTCPPose()
            tcp2 = root.uri2.recieve.getActualTCPPose()
            fmt = lambda p: f"[{', '.join(f'{v:.4f}' for v in p)}]"
            print(f"URI1 TCP: {fmt(tcp1)}")
            print(f"URI2 TCP: {fmt(tcp2)}")

            # Append to JSON file
            from datetime import datetime
            entry = {
                "timestamp": datetime.now().isoformat(),
                "uri1_tcp": list(tcp1),
                "uri2_tcp": list(tcp2),
            }
            data = []
            if TCP_LOG_FILE.exists():
                try:
                    with open(TCP_LOG_FILE, "r") as f:
                        data = json.load(f)
                except (json.JSONDecodeError, Exception):
                    data = []
            data.append(entry)
            with open(TCP_LOG_FILE, "w") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"Failed to get TCP poses: {e}")
        if root._print_tcp_active:
            root._print_tcp_after_id = root.after(2000, print_tcp_tick)

    def print_tcp_press():
        if root._print_tcp_active:
            # Stop printing
            root._print_tcp_active = False
            if root._print_tcp_after_id is not None:
                root.after_cancel(root._print_tcp_after_id)
                root._print_tcp_after_id = None
            print_tcp_button.config(text="Print TCP", fg="black")
            # Disconnect (only if teach mode is not active)
            if not (hasattr(root, 'uri1') and root.uri1 is not None and root.uri1.teachmode):
                toggle_conn_press()
            print("TCP printing stopped.")
        else:
            # Start printing
            ensure_connected()
            root._print_tcp_active = True
            print_tcp_button.config(text="Stop TCP", fg="green")
            print("TCP printing started (every 2s).")
            print_tcp_tick()

    def copy_press():
        #Get the current joint poses parameters in the GUI:
        q_copy = [float(x_entry.get()), float(y_entry.get()), float(z_entry.get()), float(rx_entry.get()), float(ry_entry.get()), float(rz_entry.get())]
        #Covert the list of joints poses to string and copy to clipboard
        pyperclip.copy(str(q_copy))

    def teach_both_press():
        # 1. ensure connected (don't toggle – robots may already be connected for TCP printing)
        ensure_connected()

        # 2. toggle teachmode for both uris
        toggle_teachmode(root.uri1)
        toggle_teachmode(root.uri2)
        if root.uri1.teachmode and root.uri2.teachmode:
            teach_both_button.config(text="End Teach Both")
            root.in_teach_mode_both = True
        elif not root.uri1.teachmode and not root.uri2.teachmode:
            root.in_teach_mode_both = False
            teach_both_button.config(text="Start Teach Both")
        else:
            #One of the uris is in teachmode and the other is not
            root.in_teach_mode_both = False
            teach_both_button.config(text="ERROR", fg='red')
        print("root.in_teach_mode_both:", root.in_teach_mode_both)
        # 3. disconnect only if TCP printing is not active
        if not root._print_tcp_active:
            toggle_conn_press()

    def base_both_press():
        # 1. connect
        toggle_conn_press()

        # 2. move both uris to base position
        home_position_joints = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        q_movej(root.uri1, *home_position_joints)
        q_movej(root.uri2, *home_position_joints)

        # 3. disconnect
        toggle_conn_press()

    def add_sample_press():
        # 1. connect
        toggle_conn_press()

        try:
            # 2. get current TCP poses from both robots
            P_uri_when_calib = root.uri1.recieve.getActualTCPPose()
            P_ayal_when_calib = root.uri2.recieve.getActualTCPPose()
            
            if P_uri_when_calib is None or P_ayal_when_calib is None:
                print("Failed to get TCP poses from robots")
                toggle_conn_press()
                return
            
            # 3. Create new sample
            new_sample = {
                "P_uri_when_calib": list(P_uri_when_calib),
                "P_ayal_when_calib": list(P_ayal_when_calib)
            }
            
            # 4. Load existing samples or create new list
            samples = []
            if CALIBRATION_SAMPLES_FILE.exists():
                try:
                    with open(CALIBRATION_SAMPLES_FILE, 'r') as f:
                        samples = json.load(f)
                except Exception as e:
                    print(f"Failed to load existing samples: {e}")
                    samples = []
            
            # 5. Append new sample
            samples.append(new_sample)
            
            # 6. Save to file
            with open(CALIBRATION_SAMPLES_FILE, 'w') as f:
                json.dump(samples, f, indent=2)
            
            print(f"Sample added. Total samples: {len(samples)}")
            print(f"P_uri: {P_uri_when_calib}")
            print(f"P_ayal: {P_ayal_when_calib}")
            print(f"Saved to {CALIBRATION_SAMPLES_FILE}")
        
        except Exception as e:
            print(f"Error adding sample: {e}")
        
        finally:
            # 7. disconnect
            toggle_conn_press()

    calibrate_button.config(command=calibrate_press)
    print_tcp_button.config(command=print_tcp_press)
    copy_button.config(command=copy_press)
    teach_both_button.config(command=teach_both_press)
    base_both_button.config(command=base_both_press)
    add_sample_button.config(command=add_sample_press)

    def guide2sample_toggle_press():
        """Toggle guide mode: uri1 in teachMode, uri2 in forceMode"""
        if not hasattr(root, 'guide2sample_active'):
            root.guide2sample_active = False

        if root.guide2sample_active:
            # Stop both modes
            try:
                # Stop teach mode on uri1
                if root.uri1.teachmode:
                    toggle_teachmode(root.uri1)
                
                # Stop force mode on uri2
                if hasattr(root.uri2.control, "forceModeStop"):
                    root.uri2.control.forceModeStop()
                else:
                    root.uri2.control.stopScript()
                
                root.guide2sample_active = False
                guide2sample_button.config(text="Guide2Sample Off", fg="black")
                print("Guide2Sample stopped")
            except Exception as e:
                print(f"Failed to stop Guide2Sample: {e}")
            return

        # Connect both robots
        try:
            if not hasattr(root, 'uri1') or root.uri1 is None or not root.uri1.is_connected():
                root.uri1 = uri_if.RMPLAB_Uri(URI1_HOST)
                root.uri1.teachmode = False
                toggle_connect(root.uri1, calibrate=False)
            
            if not hasattr(root, 'uri2') or root.uri2 is None or not root.uri2.is_connected():
                root.uri2 = uri_if.RMPLAB_Uri(URI2_HOST)
                root.uri2.teachmode = False
                toggle_connect(root.uri2, calibrate=False)
        except Exception as e:
            print(f"Failed to connect robots: {e}")
            return

        try:
            # Start teach mode on uri1
            if not root.uri1.teachmode:
                toggle_teachmode(root.uri1)
                print("Guide2Sample: URI1 teach mode ON")
            
            # Start force mode on uri2 with default parameters
            task_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            selection_vector = [1, 1, 1, 1, 1, 1]
            wrench = [0.0, 0.0, -5.0, 0.0, 0.0, 0.0]
            force_type = 2
            limits = [0.1, 0.1, 0.15, 0.1, 0.1, 0.1]
            
            root.uri2.control.forceMode(task_frame, selection_vector, wrench, force_type, limits)
            root.guide2sample_active = True
            guide2sample_button.config(text="Guide2Sample On", fg="green")
            print("Guide2Sample started - URI1 in teach mode, URI2 in force mode")
        except Exception as e:
            print(f"Failed to start Guide2Sample: {e}")

    guide2sample_button.config(command=guide2sample_toggle_press)
    
    return frame
