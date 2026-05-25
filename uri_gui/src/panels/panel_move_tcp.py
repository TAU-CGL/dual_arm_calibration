import tkinter as tk 
import tkinter.ttk as ttk
import subprocess
from pathlib import Path

import pyperclip
import uri_if
import sys

from uri_gui.src.config import *
from uri_gui.src.control import *

# Add the calibration directory to Python path
calibration_dir = Path(__file__).resolve().parents[2] / "calibration"
if calibration_dir not in sys.path:
    sys.path.insert(0, str(calibration_dir))
import gemini_calc_v2 as gemini

def add_move_tcp_panel(root, parent=None, ctx=None):
    if parent is None:
        parent = root
    if ctx is None:
        ctx = root
    frame = tk.Frame(parent, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)

    # Panel label
    tk.Label(frame, text="Move TCP by:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

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

    movej_button = tk.Button(frame, text="MoveJ_IK", font=DEFAULT_FONT)
    movej_button.pack(side=tk.LEFT, padx=(0, 3))
    
    movel_button = tk.Button(frame, text="MoveL", font=DEFAULT_FONT)
    movel_button.pack(side=tk.LEFT, padx=(0, 3))

    movej_tcp_frame_button = tk.Button(frame, text="MoveJ_IK (TCP Frame)", font=DEFAULT_FONT)
    movej_tcp_frame_button.pack(side=tk.LEFT, padx=(0, 3))

    def get_tcp_addition_entry():
        try:
            x = float(x_entry.get())
        except ValueError:
            x = 0.0
        try:
            y = float(y_entry.get())
        except ValueError:
            y = 0.0
        try:
            z = float(z_entry.get())
        except ValueError:
            z = 0.0
        try:
            rx = float(rx_entry.get())
        except ValueError:
            rx = 0.0
        try:
            ry = float(ry_entry.get())
        except ValueError:
            ry = 0.0
        try:
            rz = float(rz_entry.get())
        except ValueError:
            rz = 0.0
        return x, y, z, rx, ry, rz

    def movej_press():
        tcp_addition = get_tcp_addition_entry()
        if tcp_addition is None:
            return
        x_add, y_add, z_add, rx_add, ry_add, rz_add = tcp_addition
        x, y, z, rx, ry, rz = ctx.uri.recieve.getActualTCPPose()
        tcp_movej(ctx.uri, x+x_add, y+y_add, z+z_add, rx+rx_add, ry+ry_add, rz+rz_add)
    
    def movej_tcp_frame_press():
        P_tcp_addition = get_tcp_addition_entry()
        if P_tcp_addition is None:
            return
        T_tcp_addition = gemini.pose_to_T(P_tcp_addition)
        T_tcp = gemini.pose_to_T(ctx.uri.recieve.getActualTCPPose())
        T_tcp_new = T_tcp @ T_tcp_addition
        P_tcp_new = gemini.T_to_pose(T_tcp_new)
        tcp_movej(ctx.uri, *P_tcp_new)

    def movel_press():
        tcp_addition = get_tcp_addition_entry()
        if tcp_addition is None:
            return
        
        x_add, y_add, z_add, rx_add, ry_add, rz_add = tcp_addition
        x, y, z, rx, ry, rz = ctx.uri.recieve.getActualTCPPose()
        tcp_movel(ctx.uri, x+x_add, y+y_add, z+z_add, rx+rx_add, ry+ry_add, rz+rz_add)
    
    movej_button.config(command=movej_press)
    movel_button.config(command=movel_press)
    movej_tcp_frame_button.config(command=movej_tcp_frame_press)
    
    return frame