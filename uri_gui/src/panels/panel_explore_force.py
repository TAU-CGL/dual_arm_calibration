import tkinter as tk
import json
from pathlib import Path
from datetime import datetime
import math
import sys
import time

import uri_if

from uri_gui.src.config import *
from uri_gui.src.control import *

# Add the calibration directory to Python path for gemini
calibration_dir = Path(__file__).resolve().parents[2] / "calibration"
if calibration_dir not in sys.path:
    sys.path.insert(0, str(calibration_dir))
import gemini_calc_v2 as gemini

# TCP Force data storage file
TCP_FORCE_DATA_FILE = Path(__file__).resolve().parent.parent / "tcp_force_data.json"

# Explore Force results storage file
EXPLORE_FORCE_DATA_FILE = Path(__file__).resolve().parent.parent / "explore_force.json"

def add_explore_force_panel(root, parent=None, ctx=None):
    if parent is None:
        parent = root
    if ctx is None:
        ctx = root
    container = tk.Frame(parent)
    frame = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame.pack(anchor="w", pady=10)

    # Panel label
    tk.Label(frame, text="Explore Force:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    # Diff Forces button
    diff_button = tk.Button(frame, text="Diff_Forces", font=DEFAULT_FONT)
    diff_button.pack(side=tk.LEFT, padx=(0, 3))

    # Force difference display labels
    tk.Label(frame, text="ΔFX:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    dfx_label = tk.Label(frame, text="0.0", font=DEFAULT_FONT, width=8)
    dfx_label.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    tk.Label(frame, text="ΔFY:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    dfy_label = tk.Label(frame, text="0.0", font=DEFAULT_FONT, width=8)
    dfy_label.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    tk.Label(frame, text="ΔFZ:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    dfz_label = tk.Label(frame, text="0.0", font=DEFAULT_FONT, width=8)
    dfz_label.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    tk.Label(frame, text="ΔMX:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    dmx_label = tk.Label(frame, text="0.0", font=DEFAULT_FONT, width=8)
    dmx_label.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    tk.Label(frame, text="ΔMY:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    dmy_label = tk.Label(frame, text="0.0", font=DEFAULT_FONT, width=8)
    dmy_label.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    tk.Label(frame, text="ΔMZ:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    dmz_label = tk.Label(frame, text="0.0", font=DEFAULT_FONT, width=8)
    dmz_label.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

    # Second row for explore controls
    frame2 = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame2.pack(anchor="w", pady=10)

    tk.Label(frame2, text="Step Size [m/rad]:", font=DEFAULT_FONT).pack(side=tk.LEFT, padx=(0, 5))
    
    tk.Label(frame2, text="XYZ:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    xyz_step_entry = tk.Entry(frame2, width=8, font=DEFAULT_FONT)
    xyz_step_entry.insert(0, "0.001")
    xyz_step_entry.pack(side=tk.LEFT, padx=(0, 10))

    tk.Label(frame2, text="MXM YMZ:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    mxyz_step_entry = tk.Entry(frame2, width=8, font=DEFAULT_FONT)
    mxyz_step_entry.insert(0, "0.0")
    mxyz_step_entry.pack(side=tk.LEFT, padx=(0, 10))

    tk.Label(frame2, text="Sleep Time [s]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    sleep_time_entry = tk.Entry(frame2, width=8, font=DEFAULT_FONT)
    sleep_time_entry.insert(0, "0.5")
    sleep_time_entry.pack(side=tk.LEFT, padx=(0, 10))

    tk.Label(frame2, text="Speed [m/s]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    speed_entry = tk.Entry(frame2, width=8, font=DEFAULT_FONT)
    speed_entry.insert(0, "0.5")
    speed_entry.pack(side=tk.LEFT, padx=(0, 10))

    tk.Label(frame2, text="Accel [m/s²]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    accel_entry = tk.Entry(frame2, width=8, font=DEFAULT_FONT)
    accel_entry.insert(0, "0.5")
    accel_entry.pack(side=tk.LEFT, padx=(0, 10))

    explore_button = tk.Button(frame2, text="Explore", font=DEFAULT_FONT)
    explore_button.pack(side=tk.LEFT, padx=(10, 3))

    force_mode_button = tk.Button(frame2, text="ForceMode Off", font=DEFAULT_FONT)
    force_mode_button.pack(side=tk.LEFT, padx=(10, 3))

    # Force Mode Parameters frame
    frame_fm = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame_fm.pack(anchor="w", pady=10)

    tk.Label(frame_fm, text="Force Mode Params:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    tk.Label(frame_fm, text="Task Frame:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    task_frame_entry = tk.Entry(frame_fm, width=30, font=DEFAULT_FONT)
    task_frame_entry.insert(0, "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")
    task_frame_entry.pack(side=tk.LEFT, padx=(0, 10))

    tk.Label(frame_fm, text="Selection:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    selection_entry = tk.Entry(frame_fm, width=20, font=DEFAULT_FONT)
    selection_entry.insert(0, "[1, 1, 1, 1, 1, 1]")
    selection_entry.pack(side=tk.LEFT, padx=(0, 10))

    tk.Label(frame_fm, text="Wrench:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    wrench_entry = tk.Entry(frame_fm, width=25, font=DEFAULT_FONT)
    wrench_entry.insert(0, "[0.0, 0.0, 7.0, 0.0, 0.0, 0.0]")
    wrench_entry.pack(side=tk.LEFT, padx=(0, 10))

    tk.Label(frame_fm, text="Type:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    force_type_entry = tk.Entry(frame_fm, width=5, font=DEFAULT_FONT)
    force_type_entry.insert(0, "1")
    force_type_entry.pack(side=tk.LEFT, padx=(0, 10))

    tk.Label(frame_fm, text="Limits:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    limits_entry = tk.Entry(frame_fm, width=25, font=DEFAULT_FONT)
    limits_entry.insert(0, "[0.1, 0.1, 0.15, 0.1, 0.1, 0.1]")
    limits_entry.pack(side=tk.LEFT, padx=(0, 10))

    # Results frame for exploration
    frame3 = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame3.pack(anchor="w", pady=10)

    tk.Label(frame3, text="Exploration Results:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))
    results_label = tk.Label(frame3, text="Ready", font=DEFAULT_FONT, justify=tk.LEFT, wraplength=600)
    results_label.pack(side=tk.LEFT, anchor="w", padx=5)
    
    save_results_button = tk.Button(frame3, text="Save", font=DEFAULT_FONT)
    save_results_button.pack(side=tk.LEFT, padx=(10, 3))

    def diff_forces_press():
        """Compare last two force samples"""
        try:
            if not TCP_FORCE_DATA_FILE.exists():
                print("No force data file found")
                return
            
            with open(TCP_FORCE_DATA_FILE, 'r') as f:
                data = json.load(f)
            
            if len(data) < 2:
                print("Need at least 2 force samples to compare")
                dfx_label.config(text="N/A")
                dfy_label.config(text="N/A")
                dfz_label.config(text="N/A")
                dmx_label.config(text="N/A")
                dmy_label.config(text="N/A")
                dmz_label.config(text="N/A")
                return
            
            # Get last two samples
            sample1 = data[-2]
            sample2 = data[-1]
            
            # Compute differences
            dfx = sample2.get("FX", 0) - sample1.get("FX", 0)
            dfy = sample2.get("FY", 0) - sample1.get("FY", 0)
            dfz = sample2.get("FZ", 0) - sample1.get("FZ", 0)
            dmx = sample2.get("MX", 0) - sample1.get("MX", 0)
            dmy = sample2.get("MY", 0) - sample1.get("MY", 0)
            dmz = sample2.get("MZ", 0) - sample1.get("MZ", 0)
            
            # Update display
            dfx_label.config(text=f"{dfx:+.3f}")
            dfy_label.config(text=f"{dfy:+.3f}")
            dfz_label.config(text=f"{dfz:+.3f}")
            dmx_label.config(text=f"{dmx:+.3f}")
            dmy_label.config(text=f"{dmy:+.3f}")
            dmz_label.config(text=f"{dmz:+.3f}")
            
            print(f"Force difference: ΔFX={dfx:+.3f}, ΔFY={dfy:+.3f}, ΔFZ={dfz:+.3f}")
            
        except Exception as e:
            print(f"Failed to compute force difference: {e}")
            dfx_label.config(text="Error")

    def measure_force_magnitude():
        """Get current force magnitude"""
        try:
            forces = ctx.uri.recieve.getActualTCPForce()
            if forces and len(forces) >= 6:
                fx, fy, fz, mx, my, mz = forces[:6]
                # Total magnitude of all 6 components
                magnitude = math.sqrt(fx**2 + fy**2 + fz**2 + mx**2 + my**2 + mz**2)
                return magnitude, (fx, fy, fz, mx, my, mz)
        except Exception as e:
            print(f"Failed to measure force: {e}")
        return None, None

    def explore_press():
        """Explore each axis to find force-reducing directions"""
        try:
            xyz_step = float(xyz_step_entry.get())
            mxyz_step = float(mxyz_step_entry.get())
            sleep_time = float(sleep_time_entry.get())
            speed = float(speed_entry.get())
            accel = float(accel_entry.get())
            
            if xyz_step < 0 or mxyz_step < 0:
                print("Step sizes must be >= 0")
                return
            
            if xyz_step == 0 and mxyz_step == 0:
                print("At least one step size must be > 0")
                return
            
            if sleep_time < 0:
                print("Sleep time must be >= 0")
                return
            
            if speed <= 0 or accel <= 0:
                print("Speed and acceleration must be > 0")
                return
        except ValueError:
            print("Invalid step size, sleep time, speed, or acceleration")
            return

        print(f"Starting force exploration with step sizes: XYZ={xyz_step}, MXM YMZ={mxyz_step}")
        results_label.config(text="Exploring...")
        frame3.update()

        # Check if robot is connected
        if not hasattr(ctx, 'uri') or ctx.uri is None or not ctx.uri.is_connected():
            error_msg = "Error: Robot not connected. Please connect first."
            results_label.config(text=error_msg)
            print(error_msg)
            return

        # Get baseline force
        baseline_mag, baseline_forces = measure_force_magnitude()
        if baseline_mag is None:
            error_msg = "Error: Failed to measure baseline force. Check robot's RTDE script is running."
            results_label.config(text=error_msg)
            print(error_msg)
            return

        baseline_fx, baseline_fy, baseline_fz, baseline_mx, baseline_my, baseline_mz = baseline_forces
        print(f"Baseline force magnitude: {baseline_mag:.3f}")

        # Get current pose
        try:
            current_pose = ctx.uri.recieve.getActualTCPPose()
            if not current_pose or len(current_pose) < 6:
                print("Failed to get current TCP pose")
                return
        except Exception as e:
            print(f"Failed to get TCP pose: {e}")
            return

        results = {}
        axes = [
            ("X", xyz_step, 0),
            ("Y", xyz_step, 1),
            ("Z", xyz_step, 2),
            ("Rx", mxyz_step, 3),
            ("Ry", mxyz_step, 4),
            ("Rz", mxyz_step, 5),
        ]

        # Test each axis
        try:
            for axis_name, step_size, axis_idx in axes:
                # Skip this axis if step size is 0
                if step_size == 0:
                    results[axis_name] = "SKIPPED"
                    continue
                
                print(f"\nTesting axis {axis_name}...")
                
                # Compute pose with positive step (handle rotation properly)
                if axis_idx < 3:
                    # Position axes: simple addition
                    pose_plus = list(current_pose)
                    pose_plus[axis_idx] += step_size
                else:
                    # Rotation axes: compose rotations using transformation matrices
                    T_current = gemini.pose_to_T(current_pose)
                    step_rotvec = [0.0, 0.0, 0.0]
                    step_rotvec[axis_idx - 3] = step_size
                    R_step = gemini.rotvec_to_R(step_rotvec)
                    T_plus = T_current.copy()
                    T_plus[:3, :3] = T_plus[:3, :3] @ R_step
                    pose_plus = gemini.T_to_pose(T_plus)
                
                try:
                    ctx.uri.control.moveL(pose_plus, speed, accel, False)
                    time.sleep(sleep_time)  # Wait for robot to stabilize
                except Exception as e:
                    print(f"  Failed to move in +{axis_name}: {e}")
                    results[axis_name] = "ERROR"
                    continue
                
                mag_plus, _ = measure_force_magnitude()
                
                # Return to baseline
                try:
                    ctx.uri.control.moveL(current_pose, speed, accel, False)
                    time.sleep(sleep_time)  # Wait for robot to stabilize
                except Exception as e:
                    print(f"  Failed to return from +{axis_name}: {e}")

                # Compute pose with negative step
                if axis_idx < 3:
                    # Position axes: simple subtraction
                    pose_minus = list(current_pose)
                    pose_minus[axis_idx] -= step_size
                else:
                    # Rotation axes: compose rotations using transformation matrices
                    T_current = gemini.pose_to_T(current_pose)
                    step_rotvec = [0.0, 0.0, 0.0]
                    step_rotvec[axis_idx - 3] = -step_size
                    R_step = gemini.rotvec_to_R(step_rotvec)
                    T_minus = T_current.copy()
                    T_minus[:3, :3] = T_minus[:3, :3] @ R_step
                    pose_minus = gemini.T_to_pose(T_minus)
                
                try:
                    ctx.uri.control.moveL(pose_minus, speed, accel, False)
                    time.sleep(sleep_time)  # Wait for robot to stabilize
                except Exception as e:
                    print(f"  Failed to move in -{axis_name}: {e}")
                    results[axis_name] = "ERROR"
                    continue
                
                mag_minus, _ = measure_force_magnitude()
                
                # Return to baseline
                try:
                    ctx.uri.control.moveL(current_pose, speed, accel, False)
                    time.sleep(sleep_time)  # Wait for robot to stabilize
                except Exception as e:
                    print(f"  Failed to return from -{axis_name}: {e}")

                # Compare results
                if mag_plus is None or mag_minus is None:
                    results[axis_name] = "FAILED"
                    continue

                delta_plus = mag_plus - baseline_mag
                delta_minus = mag_minus - baseline_mag

                if delta_plus < delta_minus:
                    improvement = -delta_plus
                    direction = "+"
                    final_mag = mag_plus
                else:
                    improvement = -delta_minus
                    direction = "-"
                    final_mag = mag_minus

                if improvement > 0.01:  # Threshold for significant improvement
                    results[axis_name] = f"{axis_name}{direction}: {improvement:+.3f} ({final_mag:.3f})"
                    print(f"  {axis_name}{direction}: improved by {improvement:.3f} (mag: {final_mag:.3f})")
                else:
                    results[axis_name] = f"{axis_name}: no improvement"
                    print(f"  No significant improvement on {axis_name}")

            # Display results
            result_text = "\n".join([f"{k}: {v}" for k, v in results.items()])
            results_label.config(text=result_text)
            print(f"\nExploration complete. Results:\n{result_text}")
            
            # Store results for saving
            ctx.last_exploration_results = {
                "results": results,
                "baseline_mag": baseline_mag,
                "baseline_forces": list(baseline_forces),
                "current_pose": list(current_pose),
                "step_sizes": {"xyz": xyz_step, "mxyz": mxyz_step},
                "timestamp": datetime.now().isoformat()
        }
        
        except Exception as e:
            error_msg = f"Error during exploration: {str(e)}\nCheck robot's screen - RTDE control script may not be running."
            results_label.config(text=error_msg)
            print(error_msg)

    def force_mode_toggle_press():
        """Toggle force mode on/off with a fixed selection vector and wrench"""
        if not hasattr(ctx, 'uri') or ctx.uri is None or not ctx.uri.is_connected():
            print("Robot not connected")
            return

        if not hasattr(ctx, 'force_mode_active'):
            ctx.force_mode_active = False

        if ctx.force_mode_active:
            try:
                if hasattr(ctx.uri.control, "forceModeStop"):
                    ctx.uri.control.forceModeStop()
                else:
                    ctx.uri.control.stopScript()
                ctx.force_mode_active = False
                force_mode_button.config(text="ForceMode Off", fg="black")
                print("Force mode stopped")
            except Exception as e:
                print(f"Failed to stop force mode: {e}")
            return

        try:
            # Parse entries
            task_frame = eval(task_frame_entry.get())
            selection_vector = eval(selection_entry.get())
            wrench = eval(wrench_entry.get())
            force_type = int(force_type_entry.get())
            limits = eval(limits_entry.get())

            ctx.uri.control.forceMode(task_frame, selection_vector, wrench, force_type, limits)
            ctx.force_mode_active = True
            force_mode_button.config(text="ForceMode On", fg="green")
            print("Force mode started")
        except Exception as e:
            print(f"Failed to start force mode: {e}")

    def save_results_press():
        """Save exploration results and position to JSON file"""
        if not hasattr(ctx, 'last_exploration_results') or ctx.last_exploration_results is None:
            print("No exploration results to save")
            return
        
        try:
            explore_data = ctx.last_exploration_results.copy()
            
            # Load existing data
            data_list = []
            if EXPLORE_FORCE_DATA_FILE.exists():
                with open(EXPLORE_FORCE_DATA_FILE, 'r') as f:
                    data_list = json.load(f)
            
            # Append new exploration results
            data_list.append(explore_data)
            
            # Save to file
            with open(EXPLORE_FORCE_DATA_FILE, 'w') as f:
                json.dump(data_list, f, indent=2)
            
            print(f"Exploration results saved to {EXPLORE_FORCE_DATA_FILE}")
        except Exception as e:
            print(f"Failed to save results: {e}")

    diff_button.config(command=diff_forces_press)
    explore_button.config(command=explore_press)
    save_results_button.config(command=save_results_press)
    force_mode_button.config(command=force_mode_toggle_press)
    
    return container
