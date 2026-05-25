import tkinter as tk 
import tkinter.ttk as ttk
import ast
import json

import pyperclip
import uri_if

from uri_gui.src.config import *
from uri_gui.src.control import *

def add_actual_q_panel(root, parent=None, config_path=None):
    if parent is None:
        parent = root
    container = tk.Frame(parent)

    # Row 1: entries + buttons
    frame = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame.pack(anchor="w", pady=5)

    tk.Label(frame, text="Actual Q:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    joint_names = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"]
    entries = []
    for name in joint_names:
        tk.Label(frame, text=f"{name} [deg]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
        e = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
        e.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)
        entries.append(e)

    get_button = tk.Button(frame, text="Get", font=DEFAULT_FONT)
    get_button.pack(side=tk.LEFT, padx=(0, 3))

    movej_button = tk.Button(frame, text="MoveJ", font=DEFAULT_FONT)
    movej_button.pack(side=tk.LEFT, padx=(0, 3))

    movel_button = tk.Button(frame, text="MoveL_FK", font=DEFAULT_FONT)
    movel_button.pack(side=tk.LEFT, padx=(0, 3))

    copy_button = tk.Button(frame, text="Copy", font=DEFAULT_FONT)
    copy_button.pack(side=tk.LEFT, padx=(0, 3))

    paste_button = tk.Button(frame, text="Paste", font=DEFAULT_FONT)
    paste_button.pack(side=tk.LEFT, padx=(0, 3))

    set_default_button = tk.Button(frame, text="Set Default", font=DEFAULT_FONT)
    set_default_button.pack(side=tk.LEFT, padx=(0, 3))

    # Row 2: sliders (hidden by default)
    slider_frame = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    sliders_visible = [False]

    sliders = []
    slider_vars = []
    joint_limits = [
        ("Base",     -360, 360),
        ("Shoulder", -360, 360),
        ("Elbow",    -180, 180),
        ("Wrist1",   -360, 360),
        ("Wrist2",   -360, 360),
        ("Wrist3",   -360, 360),
    ]

    tk.Label(slider_frame, text="Sliders:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    def _on_slider_change(*_):
        """Live jog: send servoJ as sliders are dragged."""
        uri = getattr(root, 'uri', None)
        if uri is None or not uri.is_connected():
            return
        try:
            q = [sv.get() / 180 * np.pi for sv in slider_vars]
            uri.control.servoJ(q)
        except Exception:
            pass

    for name, lo, hi in joint_limits:
        tk.Label(slider_frame, text=f"{name}:", font=DEFAULT_FONT).pack(side=tk.LEFT)
        var = tk.DoubleVar(value=0.0)
        s = tk.Scale(slider_frame, from_=lo, to=hi, resolution=0.5,
                     orient=tk.HORIZONTAL, length=100, font=("Arial", 9),
                     variable=var, command=_on_slider_change)
        s.pack(side=tk.LEFT, padx=(0, 5))
        sliders.append(s)
        slider_vars.append(var)

    slider_movej_btn = tk.Button(slider_frame, text="MoveJ", font=DEFAULT_FONT)
    slider_movej_btn.pack(side=tk.LEFT, padx=(5, 3))

    def toggle_sliders():
        if sliders_visible[0]:
            slider_frame.pack_forget()
            toggle_btn.config(text="Show Sliders")
            sliders_visible[0] = False
        else:
            slider_frame.pack(anchor="w", pady=5)
            for i, sv in enumerate(slider_vars):
                try:
                    sv.set(float(entries[i].get()))
                except ValueError:
                    pass
            sliders_visible[0] = True
            toggle_btn.config(text="Hide Sliders")

    toggle_btn = tk.Button(frame, text="Show Sliders", font=DEFAULT_FONT,
                           command=toggle_sliders)
    toggle_btn.pack(side=tk.LEFT, padx=(5, 3))

    # --- Functions ---

    def get_q_pose_entry():
        try:
            return tuple(float(e.get()) / 180 * np.pi for e in entries)
        except ValueError:
            return None

    def get_q_from_sliders():
        try:
            return tuple(sv.get() / 180 * np.pi for sv in slider_vars)
        except ValueError:
            return None

    def get_press():
        q_pose = get_q_pose(root.uri)
        if q_pose is None:
            return
        for e, val in zip(entries, q_pose):
            set_text(e, "{:.6f}".format(val))
        if sliders_visible[0]:
            for sv, val in zip(slider_vars, q_pose):
                sv.set(round(val, 2))

    def movej_press():
        q_pose = get_q_pose_entry()
        if q_pose is None:
            return
        q_movej(root.uri, *q_pose)
        get_press()

    def slider_movej_press():
        q_pose = get_q_from_sliders()
        if q_pose is None:
            return
        q_movej(root.uri, *q_pose)
        for e, sv in zip(entries, slider_vars):
            set_text(e, "{:.6f}".format(sv.get()))

    def movel_press():
        q_pose = get_q_pose_entry()
        if q_pose is None:
            return
        q_movel(root.uri, *q_pose)
        get_press()

    def copy_press():
        try:
            q_copy = [float(e.get()) for e in entries]
            pyperclip.copy(str(q_copy))
            get_press()
        except ValueError:
            pass

    def paste_press():
        try:
            raw = pyperclip.paste()
            values = ast.literal_eval(raw)
            if not isinstance(values, (list, tuple)) or len(values) != 6:
                return
            for e, val in zip(entries, values):
                set_text(e, "{:.6f}".format(float(val)))
        except Exception:
            pass

    def set_default_press():
        if config_path is None:
            return
        q_pose = get_q_pose_entry()
        if q_pose is None:
            return
        try:
            with open(config_path, "r") as f:
                cfg = json.load(f)
            host = getattr(root, 'uri', None) and root.uri.host
            for name, rcfg in cfg.get("robots", {}).items():
                if rcfg.get("host") == host:
                    rcfg["q_start_pose"] = list(q_pose)
                    break
            with open(config_path, "w") as f:
                json.dump(cfg, f, indent=2)
                f.write("\n")
            print(f"[actual_q] Saved q_start_pose for {host}")
        except Exception as e:
            print(f"[actual_q] Failed to save: {e}")

    get_button.config(command=get_press)
    movej_button.config(command=movej_press)
    movel_button.config(command=movel_press)
    copy_button.config(command=copy_press)
    paste_button.config(command=paste_press)
    set_default_button.config(command=set_default_press)
    slider_movej_btn.config(command=slider_movej_press)

    return container
