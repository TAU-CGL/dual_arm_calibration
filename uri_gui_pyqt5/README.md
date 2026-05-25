# uri_gui_pyqt5

Entry point for driving the lab's UR5e robots — real or simulated — from one codebase.

## Modes

`src/main.py` has three modes (set in `sim_config.json` or via CLI arg):

- **gui** — PyQt5 GUI + PyBullet sim (or real backend). Sim runs in a child process so Qt and PyBullet don't fight over event loops. GUI panels talk to it via a command queue.
- **script** — no GUI. Loads a user script (`path/to/script.py` with a top-level `run(backend)` function) and runs it against an in-process PyBullet sim. The same script body works against real robots — see below.
- **headless** — same as script, but PyBullet runs in `DIRECT` mode (no window).

```bash
python uri_gui_pyqt5/src/main.py gui
python uri_gui_pyqt5/src/main.py script   uri_calibration/src/calibration.py
python uri_gui_pyqt5/src/main.py headless uri_calibration/src/calibration.py
```

## How "one script, two backends" works

`uri_if.RMPLAB_Uri` is a unified handle for a UR5e. On `connect()` it checks `uri_if`'s in-process robot registry: if the host is registered (because a sim booted in the same process and called `uri_if.register_robot(...)`), it dispatches to the PyBullet backend; otherwise to the real RTDE backend. So a script that does:

```python
uri = uri_if.RMPLAB_Uri(uri_if.HOST_URI); uri.connect(False)
ayal = uri_if.RMPLAB_Uri(uri_if.HOST_AYAL); ayal.connect(False)
full_calibration(uri, ayal)
```

works in **script mode** (sim, in-process) and against **real robots** (no sim booted) with zero changes.

This dispatch only works when the registry-writer (sim) and registry-reader (script) live in the **same** Python process. That's why `script`/`headless` mode uses `InProcessSimBackend` instead of the subprocess `SimBackend` that GUI mode uses.

## Sim modes at a glance

| Mode | PyBullet runs in | Script talks to robots via |
|---|---|---|
| GUI sim | child process | `backend.send_command({...})` (queue) |
| Script sim | main process | `RMPLAB_Uri(...).connect()` — same as real |
| Real | main process | `RMPLAB_Uri(...).connect()` — same as sim |

"Same script for sim and real" holds only for the **second and third** rows — they share a process. GUI-mode scripts use a different (queue-based) API and don't share code with real-robot scripts.

## Files

- `src/main.py` — entry point; picks mode and backend, loads user scripts.
- `src/robot_backends.py` — `SimBackend` (subprocess, for GUI), `InProcessSimBackend` (for script/headless), `RealBackend` (RTDE).
- `src/sim.py` — `multiRobotsSim`: boots PyBullet, loads two UR5e + Robotiq85 robots, registers them with `uri_if`, and exposes a `mapCommand(cmd)` dispatcher for queue-based scripts.
- `src/gui.py` — Qt panels (only used in `gui` mode).
- `sim_config.json` — mode, backend (`sim`/`real`), robot base poses, default joint configs, render rate.
