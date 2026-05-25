import tkinter as tk

import numpy as np
from spatialmath import SO3

import uri_if

def requires_connection(f):
    def f_(uri: uri_if.RMPLAB_Uri, *args, **kwargs):
        if not uri.is_connected():
            print("Uri is not connected! Abort.")
            return
        return f(uri, *args, **kwargs)
    return f_
        

def rotvec_to_eul(rx, ry, rz):
    v = np.array([rx, ry, rz])
    theta = np.linalg.norm(v)
    v = v / theta
    rot = SO3.AngleAxis(theta, v)
    rz, ry, rx = rot.eul('deg')
    return rx, ry, rz

def eul_to_rotvec(rx, ry, rz):
    rot = SO3.Eul(rz, ry, rx, unit='deg')
    theta, v = rot.angvec()
    rx, ry, rz = theta * v
    return rx, ry, rz

def set_text(e: tk.Entry, text):
    e.delete(0, tk.END)
    e.insert(0, text)
    return

def copy2clip(txt):
    """Copy text to clipboard using tkinter (works without external dependencies)."""
    import tkinter as tk
    
    # Create a temporary root window if one doesn't exist
    try:
        # Try to access the existing tkinter clipboard
        temp_root = tk._default_root
        if temp_root is None:
            temp_root = tk.Tk()
            temp_root.withdraw()  # Hide the window
            
        temp_root.clipboard_clear()
        temp_root.clipboard_append(txt.strip())
        temp_root.update()  # Ensure clipboard is updated
        return 0
    except Exception as e:
        # Fallback to pyperclip if available
        try:
            import pyperclip
            pyperclip.copy(txt.strip())
            return 0
        except Exception:
            raise Exception(f"Clipboard copy failed: {e}")


if __name__ == "__main__":
    print(rotvec_to_eul(2.2, 2.2, 0))

    rx, ry, rz = 45, 90, -45
    print(eul_to_rotvec(rx, ry, rz))
