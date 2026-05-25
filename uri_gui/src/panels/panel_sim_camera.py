import tkinter as tk
import json

from uri_gui.src.config import *


def add_sim_camera_panel(root, parent=None, cam_yaw=None, cam_pitch=None,
                         cam_dist=None, cam_target=None, config_path=None,
                         env=None, config=None):
    if parent is None:
        parent = root

    container = tk.Frame(parent)

    if cam_yaw is None:
        tk.Label(container, text="(not in sim mode)", font=DEFAULT_FONT).pack(side=tk.LEFT)
        return container

    # Row 1: sliders and target
    frame1 = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame1.pack(anchor="w", pady=5)

    tk.Label(frame1, text="Camera:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    def make_slider(parent_frame, label, var, from_, to, resolution=1.0):
        tk.Label(parent_frame, text=label, font=DEFAULT_FONT).pack(side=tk.LEFT)
        scale = tk.Scale(parent_frame, from_=from_, to=to, resolution=resolution,
                         orient=tk.HORIZONTAL, length=120, font=DEFAULT_FONT,
                         command=lambda v: _update(var, float(v)))
        scale.set(var[0])
        scale.pack(side=tk.LEFT, padx=(0, 10))
        return scale

    def _update(var, val):
        var[0] = val

    yaw_scale = make_slider(frame1, "Yaw:", cam_yaw, -180, 180)
    pitch_scale = make_slider(frame1, "Pitch:", cam_pitch, -89, 89)
    dist_scale = make_slider(frame1, "Dist:", cam_dist, 0.5, 5.0, resolution=0.1)

    tk.Label(frame1, text="Target X:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    tx_entry = tk.Entry(frame1, width=6, font=DEFAULT_FONT)
    tx_entry.insert(0, f"{cam_target[0]:.2f}")
    tx_entry.pack(side=tk.LEFT, padx=(0, 5))

    tk.Label(frame1, text="Y:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    ty_entry = tk.Entry(frame1, width=6, font=DEFAULT_FONT)
    ty_entry.insert(0, f"{cam_target[1]:.2f}")
    ty_entry.pack(side=tk.LEFT, padx=(0, 5))

    tk.Label(frame1, text="Z:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    tz_entry = tk.Entry(frame1, width=6, font=DEFAULT_FONT)
    tz_entry.insert(0, f"{cam_target[2]:.2f}")
    tz_entry.pack(side=tk.LEFT, padx=(0, 5))

    def apply_target():
        try:
            cam_target[0] = float(tx_entry.get())
            cam_target[1] = float(ty_entry.get())
            cam_target[2] = float(tz_entry.get())
        except ValueError:
            pass

    tk.Button(frame1, text="Set Target", font=DEFAULT_FONT,
              command=apply_target).pack(side=tk.LEFT, padx=5)

    def save_as_default():
        apply_target()
        if config_path is None:
            return
        try:
            with open(config_path, "r") as f:
                cfg = json.load(f)
            cfg["camera"] = {
                "yaw": cam_yaw[0],
                "pitch": cam_pitch[0],
                "distance": cam_dist[0],
                "target": [cam_target[0], cam_target[1], cam_target[2]],
            }
            with open(config_path, "w") as f:
                json.dump(cfg, f, indent=2)
                f.write("\n")
            print(f"[sim_camera] Saved camera defaults")
        except Exception as e:
            print(f"[sim_camera] Failed to save: {e}")

    tk.Button(frame1, text="Set Default", font=DEFAULT_FONT,
              command=save_as_default).pack(side=tk.LEFT, padx=5)

    # Row 2: presets
    frame2 = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame2.pack(anchor="w", pady=5)

    tk.Label(frame2, text="Presets:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    def apply_preset(yaw, pitch, dist=None):
        yaw_scale.set(yaw)
        pitch_scale.set(pitch)
        if dist is not None:
            dist_scale.set(dist)

    presets = [
        ("Top",    0,    -89),
        ("Front",  0,      0),
        ("Back",   180,    0),
        ("Left",   90,     0),
        ("Right",  -90,    0),
        ("Iso",    45,   -30),
    ]

    for name, yaw, pitch in presets:
        tk.Button(frame2, text=name, font=DEFAULT_FONT, width=6,
                  command=lambda y=yaw, p=pitch: apply_preset(y, p)
                  ).pack(side=tk.LEFT, padx=3)

    if env and config:
        import pybullet as _p
        import numpy as _np

        tk.Label(frame2, text="  |  ", font=DEFAULT_FONT).pack(side=tk.LEFT)

        robot_names = list(config.get("robots", {}).keys())
        for idx, rname in enumerate(robot_names):
            def _focus(i=idx, name=rname):
                if i >= len(env.robots):
                    return
                robot = env.robots[i]
                grip_len = config["robots"].get(name, {}).get("gripper_length", 0.0)
                state = _p.getLinkState(robot.id, robot.eef_id)
                ee_pos = _np.array(state[0])
                ee_rot = _np.array(_p.getMatrixFromQuaternion(state[1])).reshape(3, 3)
                tip = ee_pos + ee_rot[:, 0] * grip_len
                cam_target[0] = round(float(tip[0]), 3)
                cam_target[1] = round(float(tip[1]), 3)
                cam_target[2] = round(float(tip[2]), 3)
                tx_entry.delete(0, tk.END); tx_entry.insert(0, f"{cam_target[0]:.2f}")
                ty_entry.delete(0, tk.END); ty_entry.insert(0, f"{cam_target[1]:.2f}")
                tz_entry.delete(0, tk.END); tz_entry.insert(0, f"{cam_target[2]:.2f}")

            tk.Button(frame2, text=f"Focus {rname}", font=DEFAULT_FONT,
                      command=_focus).pack(side=tk.LEFT, padx=3)

    return container
