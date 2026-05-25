# region Readme
"""
This env has 3 modes: gui, script, headless.
1. gui      - BOTH simulation & gui
2. script   - ONLY simulation, NO gui. controlled via script
3. headless - NO visual simulation, NO gui. aimed for series of runs.
4. lab      - ONLY gui - used for controlling the robots in the lab
gui has 2 modes:
1. real - controls the real robots in the lab
2. sim  - controls the simulated robots in the sim
TODO - script and headless should also have real/sim modes.
TODO - panels that control the robot shoulf have two set of entries - current and target
TODO - should define how the robots act if they get a command before they finish the last one (should be similar to real)
TODO - add a signal back from the simulator/robots for when the action is done - that way i could respond to what is going on
"""
# endregion
# region imports & parameters (should be moved to json or something)
# General
import sys, os, argparse, importlib.util
# GUI modules
import uri_gui_pyqt5.src.robot_backends as robot_backends
import uri_gui_pyqt5.src.utils_1 as utils_1
# endregion

def _load_script_fn(script_path):
    """Load a script file and return its run(backend) function."""
    spec = importlib.util.spec_from_file_location("user_script", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not hasattr(module, "run"):
        raise AttributeError(f"Script {script_path} must define a run(backend) function")
    return module.run

def main():
    parser = argparse.ArgumentParser(description="RMP Lab")
    parser.add_argument("mode", nargs="?", choices=["gui", "script", "headless"],
                        help="Override mode (default: from config)")
    parser.add_argument("script", nargs="?",
                        help="Path to script file with run(backend) (for script/headless modes)")
    args = parser.parse_args()

    config = utils_1.load_config()
    mode = args.mode or config["mode"]
    backend_name = config["backend"]

    def _send_reach_grid(backend):
        rg = config["config_data"].get("reach_grid", {})
        if not rg.get("enabled", False):
            return
        if backend_name != "sim":
            print("[main] reach_grid ignored: only available in sim backend")
            return
        npz_path = rg.get("npz_path", "")
        if not npz_path:
            print("[main] reach_grid.enabled=true but npz_path is empty")
            return
        if not os.path.isabs(npz_path):
            npz_path = os.path.abspath(os.path.join(
                os.path.dirname(__file__), "..", npz_path))
        if not os.path.isfile(npz_path):
            print(f"[main] reach_grid: file not found: {npz_path}")
            return
        print(f"[main] sending show_reach_grid: {npz_path}")
        backend.send_command({
            "action": "show_reach_grid",
            "npz_path": npz_path,
            "point_size": rg.get("point_size", 4.0),
            "show_unreachable": rg.get("show_unreachable", False),
        })

    match mode:
        case "gui":
            from PyQt5.QtWidgets import QApplication
            import uri_gui_pyqt5.src.gui as g

            print(f"[main] {backend_name.upper()} + GUI")
            if backend_name == "real":
                backend = robot_backends.RealBackend()
            else:
                backend = robot_backends.SimBackend(config)
                _send_reach_grid(backend)

            app = QApplication(sys.argv)
            window = g.MainWindow(backend=backend)
            window.show()
            sys.exit(app.exec_())

        case "script" | "headless":
            config["mode"] = mode  # pass to sim so it knows GUI vs headless
            print(f"[main] {mode.upper()} - no GUI")
            backend = robot_backends.InProcessSimBackend(config)
            _send_reach_grid(backend)
            try:
                if args.script:
                    script_fn = _load_script_fn(args.script)
                    script_fn(backend)
                else:
                    print("[main] No script provided, running idle sim. Ctrl+C to stop.")
            finally:
                backend.shutdown()

if __name__ == "__main__":
    main()

