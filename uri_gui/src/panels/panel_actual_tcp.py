import sys
import json
import tkinter as tk 
import tkinter.ttk as ttk
import subprocess
import ast
import time

from pathlib import Path
import uri_if

# Add the calibration directory to Python path
calibration_dir = Path(__file__).resolve().parents[2] / "calibration"
if str(calibration_dir) not in sys.path:
    sys.path.insert(0, str(calibration_dir))


from uri_gui.src.config import *
from uri_gui.src.control import *

def add_actual_tcp_panel(root, parent=None, ctx=None, config_path=None):
    if parent is None:
        parent = root
    if ctx is None:
        ctx = root
    frame = tk.Frame(parent, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    # Panel label
    tk.Label(frame, text="Actual TCP:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))
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

    get_button = tk.Button(frame, text="Get", font=DEFAULT_FONT)
    get_button.pack(side=tk.LEFT, padx=(0, 3))

    movej_button = tk.Button(frame, text="MoveJ_IK", font=DEFAULT_FONT)
    movej_button.pack(side=tk.LEFT, padx=(0, 3))
    
    copy_both_button = tk.Button(frame, text="Copy Both", font=DEFAULT_FONT)
    copy_both_button.pack(side=tk.LEFT, padx=(0, 3))


    tk.Label(frame, text="Speed:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    speed_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
    speed_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)
    speed_entry.insert(0, str(DEFAULT_SPEED))
    
    tk.Label(frame, text="Accel:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    acceleration_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
    acceleration_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)
    acceleration_entry.insert(0, str(DEFAULT_ACCELERATION))
    
    movel_button = tk.Button(frame, text="MoveL", font=DEFAULT_FONT)
    movel_button.pack(side=tk.LEFT, padx=(0, 3))

    copy_button = tk.Button(frame, text="Copy", font=DEFAULT_FONT)
    copy_button.pack(side=tk.LEFT, padx=(0, 3))

    paste_button = tk.Button(frame, text="Paste", font=DEFAULT_FONT)
    paste_button.pack(side=tk.LEFT, padx=(0, 3))

    def get_tcp_pose_entry():
        try:
            x = float(x_entry.get())
            y = float(y_entry.get())
            z = float(z_entry.get())
            rx = float(rx_entry.get())
            ry = float(ry_entry.get())
            rz = float(rz_entry.get())
            return x, y, z, rx, ry, rz
        except ValueError:
            return None

    def _round_pose_4(pose):
        return tuple(round(float(v), 4) for v in pose)

    def get_press():
        tcp_pose = get_tcp_pose(ctx.uri)
        if tcp_pose is None:
            return
        x, y, z, rx, ry, rz = tcp_pose
        set_text(x_entry, "{:.4f}".format(x))
        set_text(y_entry, "{:.4f}".format(y))
        set_text(z_entry, "{:.4f}".format(z))
        set_text(rx_entry, "{:.4f}".format(rx))
        set_text(ry_entry, "{:.4f}".format(ry))
        set_text(rz_entry, "{:.4f}".format(rz))
        
    def movej_press():
        tcp_pose = get_tcp_pose_entry()
        if tcp_pose is None:
            return
        x, y, z, rx, ry, rz = tcp_pose
        try:
            speed = float(speed_entry.get())
            acceleration = float(acceleration_entry.get())
        except ValueError:
            return
        ctx.uri.control.moveJ_IK([x, y, z, rx, ry, rz], speed, acceleration, False)
    
    def movel_press():
        tcp_pose = get_tcp_pose_entry()
        if tcp_pose is None:
            return
        try:
            speed = float(speed_entry.get())
            acceleration = float(acceleration_entry.get())
        except ValueError:
            return
        x, y, z, rx, ry, rz = tcp_pose
        ctx.uri.control.moveL([x, y, z, rx, ry, rz], speed, acceleration, False)

    def copy_both_press():
        print("Copy Both Pressed")
        temp_robots = []

        def _resolve_robot_by_host(host):
            current = getattr(ctx, 'uri', None)
            if current is not None and current.host == host and current.is_connected():
                return current

            for name in ("uri1", "uri2"):
                robot = getattr(ctx, name, None)
                if robot is not None and robot.host == host and robot.is_connected():
                    return robot

            robot = uri_if.RMPLAB_Uri(host)
            robot.connect(False)
            temp_robots.append(robot)
            return robot

        try:
            uri_robot = _resolve_robot_by_host(URI1_HOST)
            ayal_robot = _resolve_robot_by_host(URI2_HOST)

            uri_tcp_pose = get_tcp_pose(uri_robot)
            print(f"URI TCP Pose: {uri_tcp_pose}")
            ayal_tcp_pose = get_tcp_pose(ayal_robot)
            print(f"AYAL TCP Pose: {ayal_tcp_pose}")
            tcp_copy = _round_pose_4(uri_tcp_pose), _round_pose_4(ayal_tcp_pose)
            print(f"Copying TCP Poses: {tcp_copy}")

            #Convert the list of ROTVEC to string and copy to clipboard
            copy2clip(str(tcp_copy))
            get_press()
        except Exception as e:
            print(f"Copy failed: {e}")
        finally:
            for robot in temp_robots:
                try:
                    robot.disconnect()
                except Exception:
                    pass

    def copy_press():
        try:
            #Get the current rotvec parameters in the GUI:
            tcp_copy = _round_pose_4([
                float(x_entry.get()),
                float(y_entry.get()),
                float(z_entry.get()),
                float(rx_entry.get()),
                float(ry_entry.get()),
                float(rz_entry.get()),
            ])

            #Convert the list of ROTVEC to string and copy to clipboard
            copy2clip(str(tcp_copy))
            get_press()
        except ValueError:
            # Handle case where entry fields contain invalid values
            pass
        except Exception as e:
            print(f"Copy failed: {e}")

    def paste_press():
        try:
            raw = frame.clipboard_get()
            values = ast.literal_eval(raw)
            if not isinstance(values, (list, tuple)) or len(values) != 6:
                return
            x, y, z, rx, ry, rz = [float(v) for v in values]
            set_text(x_entry, "{:.4f}".format(x))
            set_text(y_entry, "{:.4f}".format(y))
            set_text(z_entry, "{:.4f}".format(z))
            set_text(rx_entry, "{:.4f}".format(rx))
            set_text(ry_entry, "{:.4f}".format(ry))
            set_text(rz_entry, "{:.4f}".format(rz))
        except Exception:
            pass

    def set_default_press():
        if config_path is None:
            return
        tcp_pose = get_tcp_pose_entry()
        if tcp_pose is None:
            return
        try:
            with open(config_path, "r") as f:
                cfg = json.load(f)
            host = getattr(ctx, 'uri', None) and ctx.uri.host
            for name, rcfg in cfg.get("robots", {}).items():
                if rcfg.get("host") == host:
                    rcfg["tcp_start_pose"] = list(tcp_pose)
                    break
            with open(config_path, "w") as f:
                json.dump(cfg, f, indent=2)
                f.write("\n")
            print(f"[actual_tcp] Saved tcp_start_pose for {host}")
        except Exception as e:
            print(f"[actual_tcp] Failed to save: {e}")

    set_default_button = tk.Button(frame, text="Set Default", font=DEFAULT_FONT,
                                   command=set_default_press)
    set_default_button.pack(side=tk.LEFT, padx=(0, 3))

    get_button.config(command=get_press)
    movej_button.config(command=movej_press)
    movel_button.config(command=movel_press)
    copy_button.config(command=copy_press)
    paste_button.config(command=paste_press)
    copy_both_button.config(command=copy_both_press)
    return frame
