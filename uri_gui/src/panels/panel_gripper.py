import tkinter as tk 
import tkinter.ttk as ttk

import uri_if

from uri_gui.src.config import *
from uri_gui.src.control import *

def add_gripper_panel(root, parent=None):
    if parent is None:
        parent = root
    frame = tk.Frame(parent, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)

    # Panel label
    tk.Label(frame, text="Gripper:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    tk.Label(frame, text="Gripper Pos [0-255]: ", font=DEFAULT_FONT).pack(side=tk.LEFT)
    pos_entry = tk.Entry(frame, width=5, font=DEFAULT_FONT)
    pos_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    tk.Label(frame, text="Speed [0-255]: ", font=DEFAULT_FONT).pack(side=tk.LEFT)
    speed_entry = tk.Entry(frame, width=5, font=DEFAULT_FONT)
    speed_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)
    set_text(speed_entry, str(DEFAULT_GRIPPER_SPEED))

    tk.Label(frame, text="Force [0-255]: ", font=DEFAULT_FONT).pack(side=tk.LEFT)
    force_entry = tk.Entry(frame, width=5, font=DEFAULT_FONT)
    force_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)
    set_text(force_entry, str(DEFAULT_GRIPPER_FORCE))

    get_button = tk.Button(frame, text="Get", font=DEFAULT_FONT)
    get_button.pack(side=tk.LEFT, padx=(0, 3))

    move_button = tk.Button(frame, text="Move", font=DEFAULT_FONT)
    move_button.pack(side=tk.LEFT, padx=(0, 3))
    
    close_button = tk.Button(frame, text="Close", font=DEFAULT_FONT)
    close_button.pack(side=tk.LEFT, padx=(0, 3))

    open_button = tk.Button(frame, text="Open", font=DEFAULT_FONT)
    open_button.pack(side=tk.LEFT, padx=(0, 3))

    def get_press():
        pos = get_gripper_pos(root.uri)
        if pos is None:
            return 
        set_text(pos_entry, str(pos))

    def move_press():
        pos = int(pos_entry.get())
        speed = int(speed_entry.get())
        force = int(force_entry.get())
        move_gripper(root.uri, pos, speed, force)
        get_press()

    def close_press():
        speed = int(speed_entry.get())
        force = int(force_entry.get())
        close_gripper(root.uri, speed, force)
        get_press()

    def open_press():
        speed = int(speed_entry.get())
        force = int(force_entry.get())
        open_gripper(root.uri, speed, force)
        get_press()

    get_button.config(command=get_press)
    move_button.config(command=move_press)
    close_button.config(command=close_press)
    open_button.config(command=open_press)
    
    return frame