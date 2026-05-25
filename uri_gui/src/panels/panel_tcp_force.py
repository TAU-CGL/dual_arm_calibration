import tkinter as tk
import tkinter.ttk as ttk
import json
import sys
import time
from pathlib import Path
from datetime import datetime

import uri_if

from uri_gui.src.config import *
from uri_gui.src.control import *

# Add calibration directory to Python path for gemini helpers
calibration_dir = Path(__file__).resolve().parents[2] / "calibration"
if calibration_dir not in sys.path:
    sys.path.insert(0, str(calibration_dir))
import gemini_calc_v2 as gemini

playground_src_dir = Path(__file__).resolve().parents[2] / "playground" / "src"
if str(playground_src_dir) not in sys.path:
    sys.path.insert(0, str(playground_src_dir))
from uri_calibration.src.utils import transform_wrench_to_tcp

# TCP Force data storage file
TCP_FORCE_DATA_FILE = Path(__file__).resolve().parent.parent / "tcp_force_data.json"
FORCE_AVG_SAMPLES = 100
FORCE_SAMPLE_DELAY_S = 0.002
AUTO_REFRESH_INTERVAL_MS = 1000
DEFAULT_FORCE_FRAME_TCP = True
TCP_TIP_OFFSET_MM = 0
TCP_TIP_OFFSET = [0.0, 0.0, TCP_TIP_OFFSET_MM / 1000.0]

def add_tcp_force_panel(root, parent=None, ctx=None):
    if parent is None:
        parent = root
    if ctx is None:
        ctx = root
    container = tk.Frame(parent)
    frame = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame.pack(anchor="w", pady=10)

    # Panel label
    tk.Label(frame, text="TCP Force:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    # Dynamic unit labels for forces
    fx_unit_label = tk.Label(frame, text="FX [N]:", font=DEFAULT_FONT, width=8)
    fx_unit_label.pack(side=tk.LEFT)
    fx_label = tk.Label(frame, text="0.0", font=DEFAULT_FONT, width=10)
    fx_label.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    fy_unit_label = tk.Label(frame, text="FY [N]:", font=DEFAULT_FONT, width=8)
    fy_unit_label.pack(side=tk.LEFT)
    fy_label = tk.Label(frame, text="0.0", font=DEFAULT_FONT, width=10)
    fy_label.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    fz_unit_label = tk.Label(frame, text="FZ [N]:", font=DEFAULT_FONT, width=8)
    fz_unit_label.pack(side=tk.LEFT)
    fz_label = tk.Label(frame, text="0.0", font=DEFAULT_FONT, width=10)
    fz_label.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    # Dynamic unit labels for moments
    mx_unit_label = tk.Label(frame, text="MX [Nm]:", font=DEFAULT_FONT, width=8)
    mx_unit_label.pack(side=tk.LEFT)
    mx_label = tk.Label(frame, text="0.0", font=DEFAULT_FONT, width=10)
    mx_label.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    my_unit_label = tk.Label(frame, text="MY [Nm]:", font=DEFAULT_FONT, width=8)
    my_unit_label.pack(side=tk.LEFT)
    my_label = tk.Label(frame, text="0.0", font=DEFAULT_FONT, width=10)
    my_label.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    mz_unit_label = tk.Label(frame, text="MZ [Nm]:", font=DEFAULT_FONT, width=8)
    mz_unit_label.pack(side=tk.LEFT)
    mz_label = tk.Label(frame, text="0.0", font=DEFAULT_FONT, width=10)
    mz_label.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    # Second frame for controls
    frame2 = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame2.pack(anchor="w", pady=10)

    refresh_button = tk.Button(frame2, text="Refresh", font=DEFAULT_FONT)
    refresh_button.pack(side=tk.LEFT, padx=(10, 3))

    # Auto-refresh flag
    ctx.tcp_force_auto_refresh = False
    auto_refresh_button = tk.Button(frame2, text="Auto Off", font=DEFAULT_FONT)
    auto_refresh_button.pack(side=tk.LEFT, padx=(0, 3))

    # Tare offset storage
    ctx.tare_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Rescale factors for forces and moments
    ctx.force_rescale = 1.0
    ctx.moment_rescale = 1.0

    # Frame display/save mode flag
    if not hasattr(ctx, "force_in_tcp_frame"):
        ctx.force_in_tcp_frame = DEFAULT_FORCE_FRAME_TCP

    copy_button = tk.Button(frame2, text="Copy", font=DEFAULT_FONT)
    copy_button.pack(side=tk.LEFT, padx=(10, 3))

    save_button = tk.Button(frame2, text="Save", font=DEFAULT_FONT)
    save_button.pack(side=tk.LEFT, padx=(0, 3))

    clear_button = tk.Button(frame2, text="Clear", font=DEFAULT_FONT)
    clear_button.pack(side=tk.LEFT, padx=(0, 3))

    tk.Label(frame2, text="Avg Samples: ", font=DEFAULT_FONT).pack(side=tk.LEFT, padx=(20, 0))
    avg_samples_entry = tk.Entry(frame2, width=3, font=DEFAULT_FONT)
    avg_samples_entry.pack(side=tk.LEFT, padx=(0, 3))
    avg_samples_entry.insert(0, "1")

    save_avg_button = tk.Button(frame2, text="Save_Avg", font=DEFAULT_FONT)
    save_avg_button.pack(side=tk.LEFT, padx=(0, 3))

    zero_ft_button = tk.Button(frame2, text="Zero_FT", font=DEFAULT_FONT)
    zero_ft_button.pack(side=tk.LEFT, padx=(0, 3))

    rescale_forces_button = tk.Button(frame2, text="Rescale_F", font=DEFAULT_FONT)
    rescale_forces_button.pack(side=tk.LEFT, padx=(5, 3))

    rescale_moments_button = tk.Button(frame2, text="Rescale_M", font=DEFAULT_FONT)
    rescale_moments_button.pack(side=tk.LEFT, padx=(0, 3))

    frame_mode_button = tk.Button(frame2, font=DEFAULT_FONT)
    frame_mode_button.pack(side=tk.LEFT, padx=(10, 3))

    def update_frame_button_text():
        if ctx.force_in_tcp_frame:
            frame_mode_button.config(text="Frame: TCP", fg="green")
        else:
            frame_mode_button.config(text="Frame: Base", fg="black")

    def toggle_frame_mode_press():
        ctx.force_in_tcp_frame = not ctx.force_in_tcp_frame
        update_frame_button_text()
        update_force_display()

    def update_force_display():
        """Update force labels using average over FORCE_AVG_SAMPLES readings"""
        if not hasattr(ctx, 'uri') or ctx.uri is None or not ctx.uri.is_connected():
            fx_label.config(text="N/C")
            fy_label.config(text="N/C")
            fz_label.config(text="N/C")
            mx_label.config(text="N/C")
            my_label.config(text="N/C")
            mz_label.config(text="N/C")
            return
        
        try:
            force_samples = []
            for i in range(FORCE_AVG_SAMPLES):
                if i > 0:
                    time.sleep(FORCE_SAMPLE_DELAY_S)
                forces_base = ctx.uri.recieve.getActualTCPForce()
                pose_base_tcp = ctx.uri.recieve.getActualTCPPose()

                if forces_base and pose_base_tcp and len(forces_base) >= 6 and len(pose_base_tcp) >= 6:
                    if ctx.force_in_tcp_frame:
                        wrench_tcp = gemini.wrench_trans(
                            forces_base[:6],
                            pose_base_tcp[:6],
                            base_to_tcp=True,
                            include_translation=False,
                        )
                        force_sample = list(wrench_tcp)
                    else:
                        force_sample = list(forces_base[:6])
                    force_samples.append(force_sample)

            if force_samples:
                avg_force = [sum(s[j] for s in force_samples) / len(force_samples) for j in range(6)]
                fx, fy, fz, mx, my, mz = avg_force
                # Apply tare offset
                fx -= ctx.tare_offset[0]
                fy -= ctx.tare_offset[1]
                fz -= ctx.tare_offset[2]
                mx -= ctx.tare_offset[3]
                my -= ctx.tare_offset[4]
                mz -= ctx.tare_offset[5]
                # Apply rescale factors
                fx /= ctx.force_rescale
                fy /= ctx.force_rescale
                fz /= ctx.force_rescale
                mx /= ctx.moment_rescale
                my /= ctx.moment_rescale
                mz /= ctx.moment_rescale
                fx_label.config(text=f"{fx:.3f}")
                fy_label.config(text=f"{fy:.3f}")
                fz_label.config(text=f"{fz:.3f}")
                mx_label.config(text=f"{mx:.3f}")
                my_label.config(text=f"{my:.3f}")
                mz_label.config(text=f"{mz:.3f}")

                print(
                    f"TCP force avg ({len(force_samples)} samples, frame={'TCP' if ctx.force_in_tcp_frame else 'Base'}): "
                    f"FX={fx:.3f}, FY={fy:.3f}, FZ={fz:.3f}, "
                    f"MX={mx:.3f}, MY={my:.3f}, MZ={mz:.3f}"
                )
                
                # Update unit labels with rescale factors
                if ctx.force_rescale != 1.0:
                    fx_unit_label.config(text=f"FX [N/{ctx.force_rescale}]:")
                    fy_unit_label.config(text=f"FY [N/{ctx.force_rescale}]:")
                    fz_unit_label.config(text=f"FZ [N/{ctx.force_rescale}]:")
                else:
                    fx_unit_label.config(text="FX [N]:")
                    fy_unit_label.config(text="FY [N]:")
                    fz_unit_label.config(text="FZ [N]:")
                    
                if ctx.moment_rescale != 1.0:
                    mx_unit_label.config(text=f"MX [Nm/{ctx.moment_rescale}]:")
                    my_unit_label.config(text=f"MY [Nm/{ctx.moment_rescale}]:")
                    mz_unit_label.config(text=f"MZ [Nm/{ctx.moment_rescale}]:")
                else:
                    mx_unit_label.config(text="MX [Nm]:")
                    my_unit_label.config(text="MY [Nm]:")
                    mz_unit_label.config(text="MZ [Nm]:")
        except Exception as e:
            fx_label.config(text="Error")
            print(f"Failed to get TCP force: {e}")

    def refresh_press():
        update_force_display()

    def auto_refresh_press():
        """Toggle auto-refresh"""
        ctx.tcp_force_auto_refresh = not ctx.tcp_force_auto_refresh
        if ctx.tcp_force_auto_refresh:
            auto_refresh_button.config(text="Auto On", fg="green")
            auto_update()
        else:
            auto_refresh_button.config(text="Auto Off", fg="black")

    def auto_update():
        """Auto-update force display if enabled"""
        if ctx.tcp_force_auto_refresh:
            update_force_display()
            # Schedule next update in 1000ms
            frame.after(AUTO_REFRESH_INTERVAL_MS, auto_update)

    def copy_press():
        """Copy current force values to clipboard"""
        try:
            values = {
                "FX": fx_label.cget("text"),
                "FY": fy_label.cget("text"),
                "FZ": fz_label.cget("text"),
                "MX": mx_label.cget("text"),
                "MY": my_label.cget("text"),
                "MZ": mz_label.cget("text")
            }
            copy2clip(str(values))
            print("Force values copied to clipboard")
        except Exception as e:
            print(f"Failed to copy: {e}")

    def save_press():
        """Save current force values and TCP position to JSON file"""
        try:
            # Get TCP force values
            force_data = {
                "timestamp": datetime.now().isoformat(),
                "frame": "tcp" if ctx.force_in_tcp_frame else "base",
                "FX": float(fx_label.cget("text")) if fx_label.cget("text") not in ["N/C", "Error"] else None,
                "FY": float(fy_label.cget("text")) if fy_label.cget("text") not in ["N/C", "Error"] else None,
                "FZ": float(fz_label.cget("text")) if fz_label.cget("text") not in ["N/C", "Error"] else None,
                "MX": float(mx_label.cget("text")) if mx_label.cget("text") not in ["N/C", "Error"] else None,
                "MY": float(my_label.cget("text")) if my_label.cget("text") not in ["N/C", "Error"] else None,
                "MZ": float(mz_label.cget("text")) if mz_label.cget("text") not in ["N/C", "Error"] else None
            }

            # Get TCP position
            try:
                tcp_pose = ctx.uri.recieve.getActualTCPPose()
                if tcp_pose and len(tcp_pose) >= 6:
                    force_data["X"] = tcp_pose[0]
                    force_data["Y"] = tcp_pose[1]
                    force_data["Z"] = tcp_pose[2]
                    force_data["Rx"] = tcp_pose[3]
                    force_data["Ry"] = tcp_pose[4]
                    force_data["Rz"] = tcp_pose[5]
            except Exception as e:
                print(f"Failed to get TCP position: {e}")

            # Load existing data
            data_list = []
            if TCP_FORCE_DATA_FILE.exists():
                with open(TCP_FORCE_DATA_FILE, 'r') as f:
                    data_list = json.load(f)
            
            # Append new data
            data_list.append(force_data)

            # Save to file
            with open(TCP_FORCE_DATA_FILE, 'w') as f:
                json.dump(data_list, f, indent=2)
            
            print(f"Force and position values saved to {TCP_FORCE_DATA_FILE}")
        except Exception as e:
            print(f"Failed to save: {e}")

    def clear_press():
        """Clear all saved force data"""
        try:
            if TCP_FORCE_DATA_FILE.exists():
                TCP_FORCE_DATA_FILE.unlink()
                print(f"Cleared {TCP_FORCE_DATA_FILE}")
            else:
                print("No data file to clear")
        except Exception as e:
            print(f"Failed to clear: {e}")

    def zero_ft_press():
        """Zero the robot's force/torque sensor"""
        if not hasattr(ctx, 'uri') or ctx.uri is None or not ctx.uri.is_connected():
            print("Robot not connected")
            return
        if not hasattr(ctx.uri, "control") or not hasattr(ctx.uri.control, "zeroFtSensor"):
            print("zeroFtSensor() not available")
            return
        try:
            ok = ctx.uri.control.zeroFtSensor()
            if ok:
                print("Force/torque sensor zeroed")
            else:
                print("zeroFtSensor() returned False — robot rejected the command "
                      "(check remote control mode, control script running, no protective stop)")
        except Exception as e:
            print(f"Failed to zero FT sensor: {e}")

    def rescale_forces_press():
        """Set rescale factor for FX, FY, FZ forces"""
        try:
            rescale = float(avg_samples_entry.get())
            if rescale <= 0:
                print("Rescale factor must be > 0")
                return
            ctx.force_rescale = rescale
            print(f"Force rescale factor set to {rescale}")
            update_force_display()
        except ValueError:
            print("Invalid rescale factor")

    def rescale_moments_press():
        """Set rescale factor for MX, MY, MZ moments"""
        try:
            rescale = float(avg_samples_entry.get())
            if rescale <= 0:
                print("Rescale factor must be > 0")
                return
            ctx.moment_rescale = rescale
            print(f"Moment rescale factor set to {rescale}")
            update_force_display()
        except ValueError:
            print("Invalid rescale factor")

    def save_avg_press():
        """Collect N samples and save their average"""
        try:
            num_samples = int(avg_samples_entry.get())
            if num_samples < 1:
                print("Number of samples must be >= 1")
                return
        except ValueError:
            print("Invalid number of samples")
            return

        if not hasattr(ctx, 'uri') or ctx.uri is None or not ctx.uri.is_connected():
            print("Robot not connected")
            return

        print(f"Collecting {num_samples} samples for averaging...")
        force_samples = []
        pose_samples = []

        # Collect samples
        for i in range(num_samples):
            try:
                forces = ctx.uri.recieve.getActualTCPForce()
                pose = ctx.uri.recieve.getActualTCPPose()
                
                if forces and pose and len(forces) >= 6 and len(pose) >= 6:
                    if ctx.force_in_tcp_frame:
                        wrench_tcp = gemini.wrench_trans(
                            forces[:6],
                            pose[:6],
                            base_to_tcp=True,
                            include_translation=False,
                        )
                        force_samples.append(list(transform_wrench_to_tcp(wrench_tcp, TCP_TIP_OFFSET)))
                    else:
                        force_samples.append(forces[:6])
                if pose and len(pose) >= 6:
                    pose_samples.append(pose[:6])
                    
                if num_samples > 1:
                    print(f"  Sample {i+1}/{num_samples}")
            except Exception as e:
                print(f"Failed to read sample {i+1}: {e}")
                return
            
            # Small delay between samples (10ms)
            frame.after(10)
            root.update()

        # Calculate averages
        if force_samples:
            avg_force = [sum(s[j] for s in force_samples) / len(force_samples) for j in range(6)]
            fx_avg, fy_avg, fz_avg, mx_avg, my_avg, mz_avg = avg_force

            # Create averaged data entry
            force_data = {
                "timestamp": datetime.now().isoformat(),
                "num_samples": num_samples,
                "frame": "tcp" if ctx.force_in_tcp_frame else "base",
                "FX": fx_avg,
                "FY": fy_avg,
                "FZ": fz_avg,
                "MX": mx_avg,
                "MY": my_avg,
                "MZ": mz_avg
            }

            # Add averaged pose if available
            if pose_samples:
                avg_pose = [sum(s[j] for s in pose_samples) / len(pose_samples) for j in range(6)]
                force_data["X"] = avg_pose[0]
                force_data["Y"] = avg_pose[1]
                force_data["Z"] = avg_pose[2]
                force_data["Rx"] = avg_pose[3]
                force_data["Ry"] = avg_pose[4]
                force_data["Rz"] = avg_pose[5]

            # Load existing data
            data_list = []
            if TCP_FORCE_DATA_FILE.exists():
                with open(TCP_FORCE_DATA_FILE, 'r') as f:
                    data_list = json.load(f)
            
            # Append averaged data
            data_list.append(force_data)

            # Save to file
            with open(TCP_FORCE_DATA_FILE, 'w') as f:
                json.dump(data_list, f, indent=2)
            
            print(f"Average force and position values saved (from {num_samples} samples)")
        else:
            print("No samples collected")

    refresh_button.config(command=refresh_press)
    auto_refresh_button.config(command=auto_refresh_press)
    copy_button.config(command=copy_press)
    save_button.config(command=save_press)
    clear_button.config(command=clear_press)
    save_avg_button.config(command=save_avg_press)
    zero_ft_button.config(command=zero_ft_press)
    rescale_forces_button.config(command=rescale_forces_press)
    rescale_moments_button.config(command=rescale_moments_press)
    frame_mode_button.config(command=toggle_frame_mode_press)
    update_frame_button_text()

    return container
