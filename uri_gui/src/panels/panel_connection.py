import tkinter as tk 
import tkinter.ttk as ttk

import uri_if

from uri_gui.src.config import *
from uri_gui.src.control import *

def add_connection_panel(root):
    frame = tk.Frame(root, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)
    frame.pack(anchor="w", pady=10)

    tk.Label(frame, text="Connection: ", font=LABEL_FONT, fg="white").pack(side=tk.LEFT)
    tk.Label(frame, text="status: ", font=DEFAULT_FONT).pack(side=tk.LEFT)
    
    initial_status = "Connected (uri1)" if (hasattr(root, 'uri') and root.uri and root.uri.is_connected()) else "Disconnected"
    initial_color = "green" if "Connected" in initial_status else "red"
    conn_label = tk.Label(frame, text=initial_status, fg=initial_color, font=DEFAULT_FONT)
    conn_label.pack(side=tk.LEFT)

    uri_choice = tk.StringVar()
    uri_choice.set("uri1")

    def _on_radio_switch(*_):
        selected_host = URI1_HOST if uri_choice.get() == "uri1" else URI2_HOST
        if hasattr(root, 'uri') and root.uri is not None and root.uri.host == selected_host and root.uri.is_connected():
            return
        root.uri = uri_if.RMPLAB_Uri(selected_host)
        root.uri.connect(False)
        if root.uri.is_connected():
            conn_label.config(text=f"Connected ({uri_choice.get()})", fg="green")
        else:
            conn_label.config(text="Failed", fg="red")

    uri1_radio = tk.Radiobutton(frame, text="uri1", variable=uri_choice, value="uri1",
                                font=DEFAULT_FONT, command=_on_radio_switch)
    uri1_radio.pack(side=tk.LEFT, padx=10)
    uri2_radio = tk.Radiobutton(frame, text="uri2", variable=uri_choice, value="uri2",
                                font=DEFAULT_FONT, command=_on_radio_switch)
    uri2_radio.pack(side=tk.LEFT, padx=10)

    teachmode_button = tk.Button(frame, text="Start Teach Mode", font=DEFAULT_FONT)
    teachmode_button.pack(side=tk.LEFT, padx=0)

    # homeposition_button = tk.Button(frame, text="Home Position", font=DEFAULT_FONT)
    # homeposition_button.pack(side=tk.LEFT, padx=0)

    should_calibrate = tk.BooleanVar(root)
    should_calibrate.set(False)  # Default is not to calibrate

    calibrate_checkbox = tk.Checkbutton(frame, text='Calibrate Gripper', font=DEFAULT_FONT, variable=should_calibrate)
    calibrate_checkbox.pack(side=tk.LEFT, padx=(30,0))

    def teachmode_press():
        if root.uri is None or not root.uri.is_connected():
            return
        toggle_teachmode(root.uri)
        if root.uri.teachmode:
            teachmode_button.config(text="End Teach Mode")
        else:
            teachmode_button.config(text="Start Teach Mode")

    teachmode_button.config(command=teachmode_press)