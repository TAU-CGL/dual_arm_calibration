import tkinter as tk 
import tkinter.ttk as ttk

import os, sys
_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _RMP_LAB_ROOT in sys.path:
    sys.path.remove(_RMP_LAB_ROOT)
sys.path.insert(0, _RMP_LAB_ROOT)
import uri_if

_uris_gui_dir = os.path.dirname((__file__))
_repo_root = os.path.dirname(_uris_gui_dir)
sys.path.insert(0, os.path.join(_repo_root, "calibration"))
sys.path.insert(0, os.path.join(_repo_root, "alpha_puzzle"))
sys.path.insert(0, _uris_gui_dir)

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
    root.resizable(True, True)

    # Instead of passing Uri as an argument, 
    # just add it as the member of the root state of our software
    # We also add here all the auxilary member values for Uri
    root.uri = None
    root.in_teach_mode_both = False

    # Add, one by one, the widgets of the GUI
    add_header(root)
    add_connection_panel(root)

    # All panels use (root, parent=None) and return frame without packing.
    # Sub-frames (frame2, frame3, frame_fm) self-pack on root.
    # We just need to pack the returned main frame for each panel.
    add_actual_tcp_panel(root).pack(anchor="w", pady=5)
    add_actual_q_panel(root).pack(anchor="w", pady=5)
    add_gripper_panel(root).pack(anchor="w", pady=5)
    add_calibrate_panel(root).pack(anchor="w", pady=5)
    add_move_to_panel(root).pack(anchor="w", pady=5)
    add_tcp_force_panel(root).pack(anchor="w", pady=5)
    add_explore_force_panel(root).pack(anchor="w", pady=5)
    add_move_tcp_panel(root).pack(anchor="w", pady=5)
    add_move_both_tcp_panel(root).pack(anchor="w", pady=5)
    add_alpha_puzzle_panel(root).pack(anchor="w", pady=5)
    # add_pnp_panel(root).pack(anchor="w", pady=5)

    # And run forever
    root.mainloop()
#TODO: home button, copy 6
