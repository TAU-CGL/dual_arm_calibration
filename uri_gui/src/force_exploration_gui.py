import tkinter as tk
import sys
from pathlib import Path

# Add the uris_gui directory to Python path
uris_gui_dir = Path(__file__).parent
if uris_gui_dir not in sys.path:
    sys.path.insert(0, str(uris_gui_dir))

from uri_gui.src.panels.panel_connection import add_connection_panel
from uri_gui.src.panels.panel_explore_force import add_explore_force_panel
from uri_gui.src.panels.panel_actual_tcp import add_actual_tcp_panel

def main():
    root = tk.Tk()
    root.title("Force Exploration GUI")
    root.geometry("2000x400")

    # Add panels
    add_connection_panel(root)
    add_actual_tcp_panel(root)
    add_explore_force_panel(root)

    root.mainloop()

if __name__ == "__main__":
    main()
