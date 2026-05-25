import tkinter as tk 
import tkinter.ttk as ttk

from uri_gui.src.config import *


def add_main_navigation_panel(root, panel_container):
    """
    Creates a navigation panel with buttons to switch between different functional panels.
    
    Args:
        root: The root Tkinter window
        panel_container: The frame where panels will be shown/hidden
    """
    frame = tk.Frame(root, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame.pack(anchor="w", pady=10, fill=tk.X)

    tk.Label(frame, text="Navigation: ", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=10)

    # Dictionary to store all panel frames and their visibility state
    root.panels = {}
    root.current_panel = None

    def switch_panel(panel_name):
        """Hide current panel and show the selected panel"""
        # Hide current panel
        if root.current_panel and root.current_panel in root.panels:
            root.panels[root.current_panel].pack_forget()
        
        # Show selected panel
        if panel_name in root.panels:
            root.panels[panel_name].pack(anchor="w", pady=5, fill=tk.BOTH, expand=True)
            root.current_panel = panel_name

    # Create navigation buttons
    buttons_config = [
        ("Actual TCP", "actual_tcp"),
        ("Actual Joints", "actual_q"),
        ("Gripper", "gripper"),
        ("Calibrate", "calibrate"),
        ("Move To", "move_to"),
        ("TCP Force", "tcp_force"),
        ("Explore Force", "explore_force"),
        ("Move TCP", "move_tcp"),
        ("Move Both TCP", "move_both_tcp"),
        ("Alpha Puzzle", "alpha_puzzle"),
        ("Sim Visibility", "sim_visibility"),
        ("Sim Camera", "sim_camera"),
        ("Sim Settings", "sim_settings"),
    ]

    for button_text, panel_name in buttons_config:
        btn = tk.Button(
            frame, 
            text=button_text, 
            font=DEFAULT_FONT,
            command=lambda p=panel_name: switch_panel(p)
        )
        btn.pack(side=tk.LEFT, padx=5)

    return frame
