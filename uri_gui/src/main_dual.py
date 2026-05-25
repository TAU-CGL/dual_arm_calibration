import tkinter as tk
import sys
from types import SimpleNamespace

import os
_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _RMP_LAB_ROOT in sys.path:
    sys.path.remove(_RMP_LAB_ROOT)
sys.path.insert(0, _RMP_LAB_ROOT)
import uri_if

from uri_gui.src.config import *
from uri_gui.src.control import *
from uri_gui.src.panels.panel_move_tcp import add_move_tcp_panel
from uri_gui.src.panels.panel_actual_tcp import add_actual_tcp_panel
from uri_gui.src.panels.panel_tcp_force import add_tcp_force_panel
from uri_gui.src.panels.panel_explore_force import add_explore_force_panel
from uri_gui.src.panels.panel_loop_move import add_loop_move_panel


if __name__ == "__main__":
    # --- Connect to both robots before creating GUI ---
    uri1 = uri_if.RMPLAB_Uri(URI1_HOST)
    uri1.teachmode = False
    uri2 = uri_if.RMPLAB_Uri(URI2_HOST)
    uri2.teachmode = False

    errors = []
    try:
        uri1.connect(False)
    except Exception as e:
        errors.append(f"URI1 ({URI1_HOST}): {e}")
    try:
        uri2.connect(False)
    except Exception as e:
        errors.append(f"URI2 ({URI2_HOST}): {e}")

    if errors:
        for err in errors:
            print(f"ERROR: Failed to connect - {err}")
        # Clean up any successful connection
        try:
            if uri1.is_connected():
                uri1.disconnect()
        except Exception:
            pass
        try:
            if uri2.is_connected():
                uri2.disconnect()
        except Exception:
            pass
        sys.exit(1)

    print("Both robots connected successfully.")

    # --- Build GUI ---
    root = tk.Tk()
    root.title("Dual URI GUI")
    root.geometry("1920x900")

    # Store both URIs on root for panels that need both (loop_move)
    root.uri1 = uri1
    root.uri2 = uri2

    # Per-side contexts for single-robot panels
    ctx1 = SimpleNamespace(uri=uri1)
    ctx2 = SimpleNamespace(uri=uri2)

    # Resizable horizontal paned window
    paned = tk.PanedWindow(root, orient=tk.HORIZONTAL, sashwidth=6, sashrelief=tk.RAISED)
    paned.pack(fill=tk.BOTH, expand=True)

    # --- Left side: URI1 ---
    left_frame = tk.Frame(paned)
    paned.add(left_frame, stretch="always")
    tk.Label(left_frame, text="URI1", font=HEADER_FONT, fg="blue").pack(anchor="w", pady=(10, 5))

    add_move_tcp_panel(root, left_frame, ctx=ctx1).pack(anchor="w", pady=5)
    add_loop_move_panel(root, left_frame).pack(anchor="w", pady=5)
    add_tcp_force_panel(root, left_frame, ctx=ctx1).pack(anchor="w", pady=5)
    add_actual_tcp_panel(root, left_frame, ctx=ctx1).pack(anchor="w", pady=5)

    # --- Right side: URI2 ---
    right_frame = tk.Frame(paned)
    paned.add(right_frame, stretch="always")
    tk.Label(right_frame, text="URI2", font=HEADER_FONT, fg="red").pack(anchor="w", pady=(10, 5))

    add_move_tcp_panel(root, right_frame, ctx=ctx2).pack(anchor="w", pady=5)
    add_tcp_force_panel(root, right_frame, ctx=ctx2).pack(anchor="w", pady=5)
    add_actual_tcp_panel(root, right_frame, ctx=ctx2).pack(anchor="w", pady=5)
    add_explore_force_panel(root, right_frame, ctx=ctx2).pack(anchor="w", pady=5)

    # --- Graceful shutdown ---
    def on_close():
        try:
            if uri1.is_connected():
                uri1.disconnect()
        except Exception:
            pass
        try:
            if uri2.is_connected():
                uri2.disconnect()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
