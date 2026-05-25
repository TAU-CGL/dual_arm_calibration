import tkinter as tk

from uri_gui.src.config import *
from uri_gui.src.control import *

def add_loop_move_panel(root, parent=None):
    """Loop MoveJ panel for URI1 with force monitoring on both robots.
    
    Expects root.uri1 and root.uri2 to be connected.
    Moves URI1 by TCP offset repeatedly, checking force/moment limits on both.
    """
    if parent is None:
        parent = root

    container = tk.Frame(parent)

    # First row: TCP offset entries
    frame = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame.pack(anchor="w", pady=5)

    tk.Label(frame, text="Loop MoveJ:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

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

    # Second row: loop controls
    frame2 = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame2.pack(anchor="w", pady=5)

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

    loop_button = tk.Button(frame2, text="Loop_MoveJ_IK", font=DEFAULT_FONT)
    loop_button.pack(side=tk.LEFT, padx=(0, 3))

    # Third row: status
    frame3 = tk.Frame(container, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame3.pack(anchor="w", pady=5)

    tk.Label(frame3, text="Loop Status:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))
    status_label = tk.Label(frame3, text="Ready", font=DEFAULT_FONT, justify=tk.LEFT, wraplength=600)
    status_label.pack(side=tk.LEFT, anchor="w", padx=5)

    def get_tcp_offset():
        def safe_float(entry):
            try:
                return float(entry.get())
            except ValueError:
                return 0.0
        return (safe_float(x_entry), safe_float(y_entry), safe_float(z_entry),
                safe_float(rx_entry), safe_float(ry_entry), safe_float(rz_entry))

    def check_forces(uri, name, f_limit, m_limit):
        """Check force limits on a robot. Returns (ok, info_message)."""
        try:
            forces = uri.recieve.getActualTCPForce()
            if not forces or len(forces) < 6:
                return True, ""

            fx, fy, fz, mx, my, mz = forces[:6]
            force_labels = [("FX", fx), ("FY", fy), ("FZ", fz)]
            moment_labels = [("MX", mx), ("MY", my), ("MZ", mz)]

            max_f = max(force_labels, key=lambda t: abs(t[1]))
            max_m = max(moment_labels, key=lambda t: abs(t[1]))

            for lbl, val in force_labels:
                if abs(val) > f_limit:
                    return False, f"STOPPED: {name} {lbl}={val:.3f}N > {f_limit}N"
            for lbl, val in moment_labels:
                if abs(val) > m_limit:
                    return False, f"STOPPED: {name} {lbl}={val:.3f}Nm > {m_limit}Nm"

            info = f"{name} maxF {max_f[0]}={max_f[1]:.3f}N, maxM {max_m[0]}={max_m[1]:.3f}Nm"
            return True, info
        except Exception as e:
            return True, f"{name} force read error: {e}"

    def loop_press():
        try:
            loop_count = int(loop_count_entry.get())
            f_limit = float(f_limit_entry.get())
            m_limit = float(m_limit_entry.get())
            if loop_count < 1 or f_limit <= 0 or m_limit <= 0:
                print("Invalid loop parameters")
                return
        except ValueError:
            print("Invalid loop parameters")
            return

        uri1 = root.uri1
        uri2 = root.uri2
        if not uri1.is_connected() or not uri2.is_connected():
            status_label.config(text="Error: robots not connected")
            return

        offset = get_tcp_offset()
        x_add, y_add, z_add, rx_add, ry_add, rz_add = offset

        status_label.config(text="Looping...")
        frame3.update()

        messages = []
        stopped = False

        for i in range(loop_count):
            msg = f"Iteration {i+1}/{loop_count}"
            print(f"\n=== {msg} ===")
            messages.append(msg)

            # Move URI1 by offset
            try:
                x, y, z, rx, ry, rz = uri1.recieve.getActualTCPPose()
                tcp_movej(uri1, x + x_add, y + y_add, z + z_add,
                          rx + rx_add, ry + ry_add, rz + rz_add)
            except Exception as e:
                messages.append(f"Movement failed: {e}")
                break

            # Check forces on both robots
            ok1, info1 = check_forces(uri1, "URI1", f_limit, m_limit)
            if info1:
                messages.append(info1)
                print(info1)
            if not ok1:
                stopped = True
                break

            ok2, info2 = check_forces(uri2, "URI2", f_limit, m_limit)
            if info2:
                messages.append(info2)
                print(info2)
            if not ok2:
                stopped = True
                break

            status_label.config(text="\n".join(messages))
            frame3.update()

        if not stopped:
            messages.append("Loop complete")

        status_text = "\n".join(messages)
        status_label.config(text=status_text)
        print(f"\nLoop Results:\n{status_text}")

    loop_button.config(command=loop_press)

    return container
