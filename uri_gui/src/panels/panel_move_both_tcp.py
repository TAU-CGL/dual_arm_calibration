import tkinter as tk
import tkinter.ttk as ttk
import sys
from pathlib import Path

import uri_if
import threading

from uri_gui.src.config import *
from uri_gui.src.control import *

# Add the calibration directory to Python path
calibration_dir = Path(__file__).resolve().parents[2] / "calibration"
if calibration_dir not in sys.path:
    sys.path.insert(0, str(calibration_dir))
import gemini_calc_v2 as gemini


def add_move_both_tcp_panel(root, parent=None):
    if parent is None:
        parent = root
    container = tk.Frame(parent)
    frame = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame.pack(anchor="w", pady=5)

    # Panel label
    tk.Label(frame, text="Move Both TCP:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

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

    # Second row for loop control
    frame2 = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame2.pack(anchor="w", pady=5)

    tk.Label(frame2, text="Loop MoveJ:", font=DEFAULT_FONT).pack(side=tk.LEFT, padx=(0, 5))
    
    tk.Label(frame2, text="Iterations:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    loop_count_entry = tk.Entry(frame2, width=5, font=DEFAULT_FONT)
    loop_count_entry.insert(0, "5")
    loop_count_entry.pack(side=tk.LEFT, padx=(0, 15))
    
    tk.Label(frame2, text="F Limit [N]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    f_limit_entry = tk.Entry(frame2, width=8, font=DEFAULT_FONT)
    f_limit_entry.insert(0, "30.0")
    f_limit_entry.pack(side=tk.LEFT, padx=(0, 10))
    
    tk.Label(frame2, text="M Limit [Nm]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
    m_limit_entry = tk.Entry(frame2, width=8, font=DEFAULT_FONT)
    m_limit_entry.insert(0, "10.0")
    m_limit_entry.pack(side=tk.LEFT, padx=(0, 10))
    
    loop_movej_button = tk.Button(frame2, text="Loop_MoveJ_IK", font=DEFAULT_FONT)
    loop_movej_button.pack(side=tk.LEFT, padx=(0, 3))

    # Status frame for loop results
    frame3 = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame3.pack(anchor="w", pady=5)

    tk.Label(frame3, text="Loop Status:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))
    loop_status_label = tk.Label(frame3, text="Ready", font=DEFAULT_FONT, justify=tk.LEFT, wraplength=600)
    loop_status_label.pack(side=tk.LEFT, anchor="w", padx=5)

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

    def connect_both():
        if not hasattr(root, 'uri1') or root.uri1 is None:
            root.uri1 = uri_if.RMPLAB_Uri(URI1_HOST)
            root.uri1.teachmode = False
        if not hasattr(root, 'uri2') or root.uri2 is None:
            root.uri2 = uri_if.RMPLAB_Uri(URI2_HOST)
            root.uri2.teachmode = False
        try:
            root.uri1.connect(False)
            root.uri2.connect(False)
            return True
        except Exception as e:
            print(f"Failed to connect robots: {e}")
            return False

    def disconnect_both():
        try:
            if hasattr(root, 'uri1') and root.uri1 is not None:
                root.uri1.disconnect()
                root.uri1 = None
            if hasattr(root, 'uri2') and root.uri2 is not None:
                root.uri2.disconnect()
                root.uri2 = None
        except Exception as e:
            print(f"Failed to disconnect robots: {e}")

    def perform_movement(move_func, x_add, y_add, z_add, rx_add, ry_add, rz_add):
        """Execute movement for both robots in parallel
        
        Args:
            move_func: Function to use for movement (tcp_movej or tcp_movel)
            x_add, y_add, z_add: Position offsets
            rx_add, ry_add, rz_add: Rotation offsets
        """
        x1, y1, z1, rx1, ry1, rz1 = root.uri1.recieve.getActualTCPPose()
        target1 = (x1 + x_add, y1 + y_add, z1 + z_add, rx1 + rx_add, ry1 + ry_add, rz1 + rz_add)
        target2 = gemini.calculate_mirror_position(target1, root.move_to_calibration_pose, flip_trans=True, translation=0.00)

        t1 = threading.Thread(target=lambda: move_func(root.uri1, *target1))
        t2 = threading.Thread(target=lambda: move_func(root.uri2, *target2))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

    def movej_press():
        if not connect_both():
            return
        if not hasattr(root, 'move_to_calibrate_ack') or not root.move_to_calibrate_ack:
            print("Calibration not available. Use Move To panel to set calibration.")
            disconnect_both()
            return

        tcp_add = get_tcp_addition_entry()
        x_add, y_add, z_add, rx_add, ry_add, rz_add = tcp_add

        try:
            perform_movement(tcp_movej, x_add, y_add, z_add, rx_add, ry_add, rz_add)
        except Exception as e:
            print(f"MoveJ both failed: {e}")
        finally:
            disconnect_both()

    def movel_press():
        if not connect_both():
            return
        if not hasattr(root, 'move_to_calibrate_ack') or not root.move_to_calibrate_ack:
            print("Calibration not available. Use Move To panel to set calibration.")
            disconnect_both()
            return

        tcp_add = get_tcp_addition_entry()
        x_add, y_add, z_add, rx_add, ry_add, rz_add = tcp_add

        try:
            perform_movement(tcp_movel, x_add, y_add, z_add, rx_add, ry_add, rz_add)
        except Exception as e:
            print(f"MoveL both failed: {e}")
        finally:
            disconnect_both()

    def loop_movej_press():
        """Loop MoveJ_IK with force limit checking"""
        try:
            loop_count = int(loop_count_entry.get())
            f_limit = float(f_limit_entry.get())
            m_limit = float(m_limit_entry.get())
            
            if loop_count < 1:
                print("Loop count must be >= 1")
                return
            if f_limit <= 0 or m_limit <= 0:
                print("Force limits must be > 0")
                return
        except ValueError:
            print("Invalid input parameters")
            return

        if not connect_both():
            return
        if not hasattr(root, 'move_to_calibrate_ack') or not root.move_to_calibrate_ack:
            print("Calibration not available. Use Move To panel to set calibration.")
            disconnect_both()
            return

        tcp_add = get_tcp_addition_entry()
        x_add, y_add, z_add, rx_add, ry_add, rz_add = tcp_add

        loop_status_label.config(text="Looping...")
        frame3.update()

        status_messages = []

        try:
            for iteration in range(loop_count):
                print(f"\n=== Iteration {iteration + 1}/{loop_count} ===")
                status_messages.append(f"Iteration {iteration + 1}/{loop_count}")
                
                try:
                    perform_movement(tcp_movej, x_add, y_add, z_add, rx_add, ry_add, rz_add)
                except Exception as e:
                    error_msg = f"Movement failed: {e}"
                    print(error_msg)
                    status_messages.append(error_msg)
                    break

                # Check forces for both robots
                try:
                    forces_uri1 = root.uri1.recieve.getActualTCPForce()
                    forces_uri2 = root.uri2.recieve.getActualTCPForce()
                    
                    if forces_uri1 and len(forces_uri1) >= 6:
                        fx1, fy1, fz1, mx1, my1, mz1 = forces_uri1[:6]
                        
                        print(f"URI1 - FX: {fx1:.3f}N, FY: {fy1:.3f}N, FZ: {fz1:.3f}N, MX: {mx1:.3f}Nm, MY: {my1:.3f}Nm, MZ: {mz1:.3f}Nm")
                        max_f1 = max(("FX", fx1), ("FY", fy1), ("FZ", fz1), key=lambda t: abs(t[1]))
                        max_m1 = max(("MX", mx1), ("MY", my1), ("MZ", mz1), key=lambda t: abs(t[1]))
                        status_messages.append(
                            f"URI1 maxF {max_f1[0]}={max_f1[1]:.3f}N, maxM {max_m1[0]}={max_m1[1]:.3f}Nm"
                        )
                        loop_status_label.config(text="\n".join(status_messages))
                        frame3.update()
                        
                        if abs(fx1) > f_limit:
                            msg = f"STOPPED: URI1 FX limit reached: {fx1:.3f}N > {f_limit}N"
                            print(msg)
                            status_messages.append(msg)
                            break
                        
                        if abs(fy1) > f_limit:
                            msg = f"STOPPED: URI1 FY limit reached: {fy1:.3f}N > {f_limit}N"
                            print(msg)
                            status_messages.append(msg)
                            break
                        
                        if abs(fz1) > f_limit:
                            msg = f"STOPPED: URI1 FZ limit reached: {fz1:.3f}N > {f_limit}N"
                            print(msg)
                            status_messages.append(msg)
                            break
                        
                        if abs(mx1) > m_limit:
                            msg = f"STOPPED: URI1 MX limit reached: {mx1:.3f}Nm > {m_limit}Nm"
                            print(msg)
                            status_messages.append(msg)
                            break
                        
                        if abs(my1) > m_limit:
                            msg = f"STOPPED: URI1 MY limit reached: {my1:.3f}Nm > {m_limit}Nm"
                            print(msg)
                            status_messages.append(msg)
                            break
                        
                        if abs(mz1) > m_limit:
                            msg = f"STOPPED: URI1 MZ limit reached: {mz1:.3f}Nm > {m_limit}Nm"
                            print(msg)
                            status_messages.append(msg)
                            break
                    
                    if forces_uri2 and len(forces_uri2) >= 6:
                        fx2, fy2, fz2, mx2, my2, mz2 = forces_uri2[:6]
                        
                        print(f"URI2 - FX: {fx2:.3f}N, FY: {fy2:.3f}N, FZ: {fz2:.3f}N, MX: {mx2:.3f}Nm, MY: {my2:.3f}Nm, MZ: {mz2:.3f}Nm")
                        max_f2 = max(("FX", fx2), ("FY", fy2), ("FZ", fz2), key=lambda t: abs(t[1]))
                        max_m2 = max(("MX", mx2), ("MY", my2), ("MZ", mz2), key=lambda t: abs(t[1]))
                        status_messages.append(
                            f"URI2 maxF {max_f2[0]}={max_f2[1]:.3f}N, maxM {max_m2[0]}={max_m2[1]:.3f}Nm"
                        )
                        loop_status_label.config(text="\n".join(status_messages))
                        frame3.update()
                        
                        if abs(fx2) > f_limit:
                            msg = f"STOPPED: URI2 FX limit reached: {fx2:.3f}N > {f_limit}N"
                            print(msg)
                            status_messages.append(msg)
                            break
                        
                        if abs(fy2) > f_limit:
                            msg = f"STOPPED: URI2 FY limit reached: {fy2:.3f}N > {f_limit}N"
                            print(msg)
                            status_messages.append(msg)
                            break
                        
                        if abs(fz2) > f_limit:
                            msg = f"STOPPED: URI2 FZ limit reached: {fz2:.3f}N > {f_limit}N"
                            print(msg)
                            status_messages.append(msg)
                            break
                        
                        if abs(mx2) > m_limit:
                            msg = f"STOPPED: URI2 MX limit reached: {mx2:.3f}Nm > {m_limit}Nm"
                            print(msg)
                            status_messages.append(msg)
                            break
                        
                        if abs(my2) > m_limit:
                            msg = f"STOPPED: URI2 MY limit reached: {my2:.3f}Nm > {m_limit}Nm"
                            print(msg)
                            status_messages.append(msg)
                            break
                        
                        if abs(mz2) > m_limit:
                            msg = f"STOPPED: URI2 MZ limit reached: {mz2:.3f}Nm > {m_limit}Nm"
                            print(msg)
                            status_messages.append(msg)
                            break
                
                except Exception as e:
                    error_msg = f"Force reading failed: {e}"
                    print(error_msg)
                    status_messages.append(error_msg)

            status_messages.append("Loop complete")
            
        except Exception as e:
            error_msg = f"Loop error: {e}"
            print(error_msg)
            status_messages.append(error_msg)
        
        finally:
            disconnect_both()
            
        # Display status
        status_text = "\n".join(status_messages)
        loop_status_label.config(text=status_text)
        print(f"\nLoop Results:\n{status_text}")

    movej_button.config(command=movej_press)
    movel_button.config(command=movel_press)
    loop_movej_button.config(command=loop_movej_press)
    
    return container
