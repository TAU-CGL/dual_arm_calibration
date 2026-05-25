import tkinter as tk 
import tkinter.ttk as ttk
import os, sys

_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _RMP_LAB_ROOT in sys.path:
    sys.path.remove(_RMP_LAB_ROOT)
sys.path.insert(0, _RMP_LAB_ROOT)
import uri_if

from uri_gui.src.config import *
from uri_gui.src.control import *
from uri_gui.src.panels import *


# Things we should do:
# TODO: Home button?
# TODO: Handle gracefully the case of protective stop
# TODO: Copy to clipboard


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Uri's GUI")
    root.geometry('1800x900')
    root.resizable(False, False)

    # Instead of passing Uri as an argument, 
    # just add it as the member of the root state of our software
    # We also add here all the auxilary member values for Uri
    root.uri = None
    
    # Initialize panels dictionary
    root.panels = {}
    root.current_panel = None

    # Add header and connection panel (always visible)
    add_header(root)
    add_connection_panel(root)
    
    # Add navigation panel with switching capability
    add_main_navigation_panel(root, None)
    
    # Create a content frame where panels will be displayed (one at a time)
    content_frame = tk.Frame(root)
    content_frame.pack(fill="both", expand=True)
    
    # Add all panels to content_frame (initially hidden)
    # Store the frame returned by each add_*_panel function
    root.panels["actual_tcp"] = add_actual_tcp_panel(root, content_frame)
    root.panels["actual_q"] = add_actual_q_panel(root, content_frame)
    root.panels["gripper"] = add_gripper_panel(root, content_frame)
    root.panels["calibrate"] = add_calibrate_panel(root, content_frame)
    root.panels["move_to"] = add_move_to_panel(root, content_frame)
    root.panels["tcp_force"] = add_tcp_force_panel(root, content_frame)
    root.panels["explore_force"] = add_explore_force_panel(root, content_frame)
    root.panels["move_tcp"] = add_move_tcp_panel(root, content_frame)
    root.panels["move_both_tcp"] = add_move_both_tcp_panel(root, content_frame)
    root.panels["alpha_puzzle"] = add_alpha_puzzle_panel(root, content_frame)
    # root.panels["pnp"] = add_pnp_panel(root, content_frame)
    
    # Hide all panels initially
    for panel_frame in root.panels.values():
        panel_frame.pack_forget()
    
    # Show the first panel by default
    root.panels["actual_tcp"].pack(anchor="w", pady=5)
    root.current_panel = "actual_tcp"

    # And run forever
    root.mainloop()
#TODO: home button, copy 6