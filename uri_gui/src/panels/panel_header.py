import tkinter as tk 
import tkinter.ttk as ttk

import uri_if

from uri_gui.src.config import *
from uri_gui.src.control import *

def add_header(root):
    tk.Label(root, text="RMLAB Uri's GUI", font=HEADER_FONT).pack(anchor="w", pady=(10, 0))