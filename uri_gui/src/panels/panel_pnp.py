# import tkinter as tk 
# import tkinter.ttk as ttk
# import subprocess

# import pyperclip
# import sim_uri

# from config import *
# from control import *


# def copy2clip(txt):
#     """Copy text to clipboard using tkinter (works without external dependencies)."""
#     import tkinter as tk
    
#     # Create a temporary root window if one doesn't exist
#     try:
#         # Try to access the existing tkinter clipboard
#         temp_root = tk._default_root
#         if temp_root is None:
#             temp_root = tk.Tk()
#             temp_root.withdraw()  # Hide the window
            
#         temp_root.clipboard_clear()
#         temp_root.clipboard_append(txt.strip())
#         temp_root.update()  # Ensure clipboard is updated
#         return 0
#     except Exception as e:
#         # Fallback to pyperclip if available
#         try:
#             import pyperclip
#             pyperclip.copy(txt.strip())
#             return 0
#         except Exception:
#             raise Exception(f"Clipboard copy failed: {e}")


# def add_actual_tcp_panel(root):
#     frame = tk.Frame(root, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
#     frame.pack()

#     tk.Label(frame, text="X [m]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
#     x_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
#     x_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

#     tk.Label(frame, text="Y [m]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
#     y_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
#     y_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

#     tk.Label(frame, text="Z [m]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
#     z_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
#     z_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

#     tk.Label(frame, text="RX [rad]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
#     rx_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
#     rx_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

#     tk.Label(frame, text="RY [rad]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
#     ry_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
#     ry_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

#     tk.Label(frame, text="RZ [rad]:", font=DEFAULT_FONT).pack(side=tk.LEFT)
#     rz_entry = tk.Entry(frame, width=DEFAULT_ENTRY_WIDTH, font=DEFAULT_FONT)
#     rz_entry.pack(side=tk.LEFT, padx=DEFAULT_ENTRY_PAD)

#     get_button = tk.Button(frame, text="Get", font=DEFAULT_FONT)
#     get_button.pack(side=tk.LEFT, padx=(0, 3))

#     movej_button = tk.Button(frame, text="MoveJ_IK", font=DEFAULT_FONT)
#     movej_button.pack(side=tk.LEFT, padx=(0, 3))
    
#     movel_button = tk.Button(frame, text="MoveL", font=DEFAULT_FONT)
#     movel_button.pack(side=tk.LEFT, padx=(0, 3))

#     Copy_button = tk.Button(frame, text="Copy", font=DEFAULT_FONT)
#     Copy_button.pack(side=tk.LEFT, padx=(0, 3))

#     def get_tcp_pose_entry():
#         try:
#             x = float(x_entry.get())
#             y = float(y_entry.get())
#             z = float(z_entry.get())
#             rx = float(rx_entry.get())
#             ry = float(ry_entry.get())
#             rz = float(rz_entry.get())
#             return x, y, z, rx, ry, rz
#         except ValueError:
#             return None

#     def get_press():
#         tcp_pose = get_tcp_pose(root.uri)
#         if tcp_pose is None:
#             return
#         x, y, z, rx, ry, rz = tcp_pose
#         set_text(x_entry, "{:.6f}".format(x))
#         set_text(y_entry, "{:.6f}".format(y))
#         set_text(z_entry, "{:.6f}".format(z))
#         set_text(rx_entry, "{:.6f}".format(rx))
#         set_text(ry_entry, "{:.6f}".format(ry))
#         set_text(rz_entry, "{:.6f}".format(rz))
        
#     def movej_press():
#         tcp_pose = get_tcp_pose_entry()
#         if tcp_pose is None:
#             return
#         x, y, z, rx, ry, rz = tcp_pose
#         tcp_movej(root.uri, x, y, z, rx, ry, rz)
#         get_press()
    
#     def movel_press():
#         tcp_pose = get_tcp_pose_entry()
#         if tcp_pose is None:
#             return
#         x, y, z, rx, ry, rz = tcp_pose
#         tcp_movel(root.uri, x, y, z, rx, ry, rz)
#         get_press()

#     def copy_press():
#         try:
#             #Get the current rotvec parameters in the GUI:
#             tcp_copy = [float(x_entry.get()), float(y_entry.get()), float(z_entry.get()), float(rx_entry.get()), float(ry_entry.get()), float(rz_entry.get())]

#             #Convert the list of ROTVEC to string and copy to clipboard
#             copy2clip(str(tcp_copy))
#             get_press()
#         except ValueError:
#             # Handle case where entry fields contain invalid values
#             pass
#         except Exception as e:
#             print(f"Copy failed: {e}")
    
#     get_button.config(command=get_press)
#     movej_button.config(command=movej_press)
#     movel_button.config(command=movel_press)
#     Copy_button.config(command = copy_press)