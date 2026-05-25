import tkinter as tk
import json

from uri_gui.src.config import *


def add_sim_visibility_panel(root, parent=None, env=None, config=None, config_path=None):
    if parent is None:
        parent = root

    frame = tk.Frame(parent, borderwidth=DEFAULT_FRAME_BORDER, relief=tk.RIDGE)

    tk.Label(frame, text="Sim Visibility:", font=LABEL_FONT, fg="white").pack(side=tk.LEFT, padx=(0, 10))

    if env is None or config is None:
        tk.Label(frame, text="(not in sim mode)", font=DEFAULT_FONT).pack(side=tk.LEFT)
        return frame

    checkbuttons = {}

    for name, rcfg in config.get("robots", {}).items():
        var = tk.BooleanVar(value=rcfg.get("show", True))
        def _toggle(n=name, v=var):
            env.set_visible(n, v.get())
        cb = tk.Checkbutton(frame, text=name, font=DEFAULT_FONT,
                            variable=var, command=_toggle)
        cb.pack(side=tk.LEFT, padx=5)
        checkbuttons[name] = var

    for name, wcfg in config["walls"].items():
        var = tk.BooleanVar(value=wcfg.get("show", True))
        def _toggle(n=name, v=var):
            env.set_visible(n, v.get())
        cb = tk.Checkbutton(frame, text=name, font=DEFAULT_FONT,
                            variable=var, command=_toggle)
        cb.pack(side=tk.LEFT, padx=5)
        checkbuttons[name] = var

    for name, scfg in config["stands"].items():
        var = tk.BooleanVar(value=scfg.get("show", True))
        def _toggle(n=name, v=var):
            env.set_visible(n, v.get())
        cb = tk.Checkbutton(frame, text=name, font=DEFAULT_FONT,
                            variable=var, command=_toggle)
        cb.pack(side=tk.LEFT, padx=5)
        checkbuttons[name] = var

    # Overlay toggles (stored on env so render_frame can read them)
    tk.Label(frame, text="  |  ", font=DEFAULT_FONT).pack(side=tk.LEFT)

    env.show_axes = config.get("show_axes", True)
    axes_var = tk.BooleanVar(value=env.show_axes)
    def _toggle_axes():
        env.show_axes = axes_var.get()
    tk.Checkbutton(frame, text="XYZ Axes", font=DEFAULT_FONT,
                   variable=axes_var, command=_toggle_axes).pack(side=tk.LEFT, padx=5)
    checkbuttons["show_axes"] = axes_var

    env.show_joint_arcs = config.get("show_joint_arcs", True)
    arcs_var = tk.BooleanVar(value=env.show_joint_arcs)
    def _toggle_arcs():
        env.show_joint_arcs = arcs_var.get()
    tk.Checkbutton(frame, text="Joint Arcs", font=DEFAULT_FONT,
                   variable=arcs_var, command=_toggle_arcs).pack(side=tk.LEFT, padx=5)
    checkbuttons["show_joint_arcs"] = arcs_var

    def save_defaults():
        if config_path is None:
            return
        try:
            with open(config_path, "r") as f:
                cfg = json.load(f)
            for name, rcfg in cfg.get("robots", {}).items():
                if name in checkbuttons:
                    rcfg["show"] = checkbuttons[name].get()
            for name, wcfg in cfg.get("walls", []).items():
                wname = wcfg.get("name", "")
                if wname in checkbuttons:
                    wcfg["show"] = checkbuttons[wname].get()
            for name, scfg in cfg.get("stands", []).items():
                sname = scfg.get("name", "")
                if sname in checkbuttons:
                    scfg["show"] = checkbuttons[sname].get()
            if "show_axes" in checkbuttons:
                cfg["show_axes"] = checkbuttons["show_axes"].get()
            if "show_joint_arcs" in checkbuttons:
                cfg["show_joint_arcs"] = checkbuttons["show_joint_arcs"].get()
            with open(config_path, "w") as f:
                json.dump(cfg, f, indent=2)
                f.write("\n")
            print(f"[sim_visibility] Saved visibility defaults")
        except Exception as e:
            print(f"[sim_visibility] Failed to save: {e}")

    tk.Button(frame, text="Set Default", font=DEFAULT_FONT,
              command=save_defaults).pack(side=tk.LEFT, padx=10)

    return frame
