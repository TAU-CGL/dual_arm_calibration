import tkinter as tk
import json

from uri_gui.src.config import *


def add_sim_settings_panel(root, parent=None, config_path=None):
    if parent is None:
        parent = root

    container = tk.Frame(parent)

    try:
        import uri_if
        modes = uri_if.MOVEJ_MODES
    except (ImportError, AttributeError):
        tk.Label(container, text="(not in sim mode)", font=DEFAULT_FONT).pack(side=tk.LEFT)
        return container

    # Row 1: mode selector
    frame1 = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame1.pack(anchor="w", pady=5)

    tk.Label(frame1, text="Sim Settings:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    tk.Label(frame1, text="moveJ mode:", font=DEFAULT_FONT).pack(side=tk.LEFT, padx=(0, 5))

    mode_var = tk.StringVar(value=uri_if.get_movej_mode())
    mode_menu = tk.OptionMenu(frame1, mode_var, *modes,
                              command=lambda v: uri_if.set_movej_mode(v))
    mode_menu.config(font=DEFAULT_FONT, width=16)
    mode_menu.pack(side=tk.LEFT, padx=(0, 15))

    descriptions = {
        "Teleport Interp": "Smooth trajectory, instant positioning (no collision mid-move)",
        "PD Interp": "Smooth trajectory with physics (collisions work mid-move)",
        "Pure Physics": "Set target and let PD controller converge (most realistic dynamics)",
    }

    desc_label = tk.Label(frame1, text=descriptions.get(mode_var.get(), ""),
                          font=DEFAULT_FONT, fg="gray")
    desc_label.pack(side=tk.LEFT, padx=(0, 10))

    def on_mode_change(v):
        uri_if.set_movej_mode(v)
        desc_label.config(text=descriptions.get(v, ""))

    mode_var.trace_add("write", lambda *_: on_mode_change(mode_var.get()))

    # Row 2: physics parameters
    frame2 = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame2.pack(anchor="w", pady=5)

    tk.Label(frame2, text="Physics Params:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    from uri_gui.src.config import DEFAULT_SPEED, DEFAULT_ACCELERATION

    tk.Label(frame2, text="Speed:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    speed_entry = tk.Entry(frame2, width=6, font=DEFAULT_FONT)
    speed_entry.insert(0, str(DEFAULT_SPEED))
    speed_entry.pack(side=tk.LEFT, padx=(0, 8))

    tk.Label(frame2, text="Accel:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    accel_entry = tk.Entry(frame2, width=6, font=DEFAULT_FONT)
    accel_entry.insert(0, str(DEFAULT_ACCELERATION))
    accel_entry.pack(side=tk.LEFT, padx=(0, 15))

    current = uri_if.get_physics_params()
    param_entries = {}

    param_defs = [
        ("Force Mult:", "force_multiplier", 8),
        ("Pos Gain:", "position_gain", 6),
        ("Vel Gain:", "velocity_gain", 6),
        ("Conv Thresh:", "converge_threshold", 6),
        ("Max Steps:", "max_converge_steps", 6),
    ]

    for label, key, width in param_defs:
        tk.Label(frame2, text=label, font=DEFAULT_FONT).pack(side=tk.LEFT)
        e = tk.Entry(frame2, width=width, font=DEFAULT_FONT)
        e.insert(0, str(current.get(key, "")))
        e.pack(side=tk.LEFT, padx=(0, 8))
        param_entries[key] = e

    def apply_params():
        import uri_gui.src.config as gui_config
        try:
            gui_config.DEFAULT_SPEED = float(speed_entry.get())
        except ValueError:
            pass
        try:
            gui_config.DEFAULT_ACCELERATION = float(accel_entry.get())
        except ValueError:
            pass
        params = {}
        for key, entry in param_entries.items():
            try:
                val = float(entry.get())
                if key == "max_converge_steps":
                    val = int(val)
                params[key] = val
            except ValueError:
                pass
        uri_if.set_physics_params(params)

    tk.Button(frame2, text="Apply", font=DEFAULT_FONT,
              command=apply_params).pack(side=tk.LEFT, padx=5)

    def save_default():
        apply_params()
        if config_path is None:
            return
        try:
            with open(config_path, "r") as f:
                cfg = json.load(f)
            cfg["movej_mode"] = mode_var.get()
            cfg["physics_params"] = uri_if.get_physics_params()
            try:
                cfg["default_speed"] = float(speed_entry.get())
                cfg["default_acceleration"] = float(accel_entry.get())
            except ValueError:
                pass
            with open(config_path, "w") as f:
                json.dump(cfg, f, indent=2)
                f.write("\n")
            print(f"[sim_settings] Saved settings to config")
        except Exception as e:
            print(f"[sim_settings] Failed to save: {e}")

    tk.Button(frame2, text="Set Default", font=DEFAULT_FONT,
              command=save_default).pack(side=tk.LEFT, padx=5)

    return container
