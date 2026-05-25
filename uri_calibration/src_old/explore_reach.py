#!/usr/bin/env python3
"""
Explore reachable workspace by stepping along each axis (+x, +y, +z, -x, -y, -z).

For each candidate step (0.01 m), checks:
  1. URI can reach the new pose (IK + safety).
  2. AYAL can reach its mirror pose (IK + safety).
If both pass, URI moves there (AYAL follows via its own mirroring logic).
Keeps going until a direction is blocked, then moves to the next axis.
"""
# region imports
import os
import sys
import json
import time
from pathlib import Path

import numpy as np
_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _RMP_LAB_ROOT in sys.path:
    sys.path.remove(_RMP_LAB_ROOT)
sys.path.insert(0, _RMP_LAB_ROOT)
import uri_if

# ── Tee stdout/stderr to a log file (overwritten each run) ────────────────────
_LOG_FILE = Path(__file__).resolve().parent.parent / "logs" / "explore_reach.log"

class _Tee:
    """Write to both the original stream and a file."""
    def __init__(self, stream, fileobj):
        self._stream = stream
        self._file = fileobj
    def write(self, data):
        self._stream.write(data)
        self._file.write(data)
        self._file.flush()
    def flush(self):
        self._stream.flush()
        self._file.flush()
    def __getattr__(self, name):
        return getattr(self._stream, name)

_log_fh = open(_LOG_FILE, "w")          # truncate on every run
sys.stdout = _Tee(sys.__stdout__, _log_fh)
sys.stderr = _Tee(sys.__stderr__, _log_fh)

# Add sibling dirs to path so we can import helpers
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "calibration"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "uris_gui"))

from uri_calibration.src.is_valid_pose import check_pose, _manipulability, MIN_MANIPULABILITY_AYAL
from find_center import run_find_center
from find_center_3 import run_find_center_direct as run_find_center_v3
from uri_calibration.src.dashboard_handler import DashboardHandler
import gemini_calc_v2 as gemini
from config import DEFAULT_SPEED, DEFAULT_ACCELERATION
# endregion

#region config
# ── Configuration ──────────────────────────────────────────────────────────────
URI_HOST  = "192.168.56.101"
AYAL_HOST = "192.168.57.101"

STEP_SIZE = 0.005   # metres per step (XYZ)
ROT_STEP  = 0.05   # radians per step (RX/RY/RZ)

CALIBRATION_FILE = Path(__file__).resolve().parent.parent.parent / "uris_gui" / "calibration.json"

OUTPUT_JSON = Path(__file__).resolve().parent.parent / "logs" / "explore_reach_log.json"

# Force-mode (compliance) parameters
FORCE_TASK_FRAME     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
FORCE_SELECTION      = [1, 1, 1, 1, 1, 1]
FORCE_WRENCH         = [0.0, 0.0, 4.0, 0.0, 0.0, 0.0]
FORCE_TYPE           = 1
FORCE_LIMITS         = [0.08, 0.08, 0.12, 0.08, 0.08, 0.08]

# Which axes to explore — toggle True/False as needed
EXPLORE_X  = True
EXPLORE_Y  = True
EXPLORE_Z  = True
EXPLORE_RX = True
EXPLORE_RY = True
EXPLORE_RZ = True

DIRECTIONS = [
    ("+X",  [STEP_SIZE, 0, 0, 0, 0, 0]),
    ("+Y",  [0, STEP_SIZE, 0, 0, 0, 0]),
    ("+Z",  [0, 0, STEP_SIZE, 0, 0, 0]),
    ("-X",  [-STEP_SIZE, 0, 0, 0, 0, 0]),
    ("-Y",  [0, -STEP_SIZE, 0, 0, 0, 0]),
    ("-Z",  [0, 0, -STEP_SIZE, 0, 0, 0]),
    ("+RX", [0, 0, 0, ROT_STEP, 0, 0]),
    ("+RY", [0, 0, 0, 0, ROT_STEP, 0]),
    ("+RZ", [0, 0, 0, 0, 0, ROT_STEP]),
    ("-RX", [0, 0, 0, -ROT_STEP, 0, 0]),
    ("-RY", [0, 0, 0, 0, -ROT_STEP, 0]),
    ("-RZ", [0, 0, 0, 0, 0, -ROT_STEP]),
]

_AXIS_FILTER = {
    "X":  EXPLORE_X,
    "Y":  EXPLORE_Y,
    "Z":  EXPLORE_Z,
    "RX": EXPLORE_RX,
    "RY": EXPLORE_RY,
    "RZ": EXPLORE_RZ,
}
# Extract axis name: strip leading +/- to get "X", "Y", "RX", etc.
DIRECTIONS = [(n, d) for n, d in DIRECTIONS if _AXIS_FILTER.get(n.lstrip("+-"), False)]

MY_SPEED = DEFAULT_SPEED/4
MY_ACCELERATION = DEFAULT_ACCELERATION/5

# Number of times to repeat the full exploration over all directions
N_LOOPS = 1

# Run find_center every this many exploration steps (across all directions)
CENTER_EVERY_N_STEPS = 15
# endregion

# ── Helpers ────────────────────────────────────────────────────────────────────
def load_calibration():
    """Load the [x,y,z,rx,ry,rz] calibration pose from calibration.json."""
    with open(CALIBRATION_FILE, "r") as f:
        return json.load(f)

def fmt(pose):
    return "[" + ", ".join(f"{v:.6f}" for v in pose) + "]"

def add_poses(a, b):
    return [ai + bi for ai, bi in zip(a, b)]

def check_robots_ok(uri, ayal):
    """Return True if both robots are fine. Print & return False on protective/emergency stop."""
    for name, robot in [("URI", uri), ("AYAL", ayal)]:
        if robot.recieve.isProtectiveStopped():
            print(f"❌  {name} is in PROTECTIVE STOP — aborting.")
            return False
        if robot.recieve.isEmergencyStopped():
            print(f"❌  {name} is in EMERGENCY STOP — aborting.")
            return False
    return True


def _is_mode_running(mode_text: str) -> bool:
    return "RUNNING" in str(mode_text).strip().upper()


def ensure_both_running_or_confirm(uri_dashboard, ayal_dashboard, context: str) -> bool:
    """Check both robot modes before motion and ask user if they want to continue."""
    uri_mode = uri_dashboard.get_robot_mode()
    ayal_mode = ayal_dashboard.get_robot_mode()

    if _is_mode_running(uri_mode) and _is_mode_running(ayal_mode):
        return True

    print("\n⚠  Robot mode check failed before move")
    print(f"   Context: {context}")
    print(f"   URI  mode: {uri_mode}")
    print(f"   AYAL mode: {ayal_mode}")
    answer = input("   Continue anyway? [y/N]: ").strip().lower()
    return answer in ("y", "yes")

# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    # Load calibration (Ayal-base expressed in Uri-base frame)
    calib_pose = load_calibration()
    print(f"Calibration pose: {fmt(calib_pose)}")

    # Connect both robots
    uri  = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)

    try:
        uri.connect(False)
    except Exception as e:
        print(f"ERROR connecting URI ({URI_HOST}): {e}")
        sys.exit(1)

    try:
        ayal.connect(False)
    except Exception as e:
        print(f"ERROR connecting AYAL ({AYAL_HOST}): {e}")
        uri.disconnect()
        sys.exit(1)

    if not check_robots_ok(uri, ayal):
        uri.disconnect()
        ayal.disconnect()
        sys.exit(1)

    uri_dashboard = DashboardHandler(URI_HOST)
    ayal_dashboard = DashboardHandler(AYAL_HOST)

    current_pose = list(uri.recieve.getActualTCPPose())
    print(f"Starting URI TCP: {fmt(current_pose)}\n")

    # Enable force mode on AYAL so it stays compliant while URI explores
    print("Enabling force mode (compliance) on AYAL…")
    ayal.control.forceMode(
        FORCE_TASK_FRAME, FORCE_SELECTION, FORCE_WRENCH, FORCE_TYPE, FORCE_LIMITS
    )
    print("Force mode active on AYAL.\n")

    exploration_log = []  # only populated after find_center runs
    global_step_counter = 0  # steps since last centering

    try:
        for loop_idx in range(1, N_LOOPS + 1):
            print(f"\n{'#' * 70}")
            print(f"  EXPLORATION LOOP {loop_idx} / {N_LOOPS}")
            print(f"{'#' * 70}")

            results = {}  # direction_name → steps taken (this loop)

            # Group directions into axis pairs: (+X,-X), (+Y,-Y), …
            axis_pairs = {}
            for dir_name, delta in DIRECTIONS:
                axis = dir_name.lstrip("+-")
                axis_pairs.setdefault(axis, []).append((dir_name, delta))

            for axis, dirs_in_pair in axis_pairs.items():
                best_m_ayal = _manipulability(list(ayal.recieve.getActualQ()))
                best_uri_pose = list(current_pose)
                print(f"\n  ▶  Starting {axis}-axis — baseline AYAL m = {best_m_ayal:.6f}")

                for dir_name, delta in dirs_in_pair:
                    print(f"\n{'='*60}")
                    print(f"Direction: {dir_name}  (loop {loop_idx})")
                    print(f"{'='*60}")

                    time.sleep(1)  # brief pause before starting new direction - so ayal start being loose again

                    steps = 0
                    pose = list(current_pose)

                    while True:
                        if not check_robots_ok(uri, ayal):
                            print("Aborting exploration due to robot stop.")
                            sys.exit(1)

                        candidate = add_poses(pose, delta)
                        print(f"\n  Step {steps + 1}: candidate URI pose {fmt(candidate)}")

                        # 1. Check URI
                        uri_reachable, uri_straight = check_pose(uri, candidate)
                        if not uri_reachable:
                            print(f"  ❌  URI cannot reach this pose. Stopping {dir_name}.")
                            break
                        if not uri_straight:
                            print(f"  ⚠️   URI has no straight-line path. Stopping {dir_name}.")
                            break

                        # 2. Compute mirror pose for AYAL
                        ayal_candidate = gemini.calculate_mirror_position(
                            candidate, calib_pose, flip_trans=True
                        )
                        print(f"       Mirror AYAL pose  {fmt(ayal_candidate)}")

                        # 3. Check AYAL
                        ayal_reachable, ayal_straight = check_pose(
                            ayal, list(ayal_candidate), min_m=MIN_MANIPULABILITY_AYAL
                        )
                        if not ayal_reachable:
                            print(f"  ❌  AYAL cannot reach mirror pose. Stopping {dir_name}.")
                            break
                        if not ayal_straight:
                            print(f"  ⚠️   AYAL has no straight-line path. Stopping {dir_name}.")
                            break

                        # 4. Both OK → move URI
                        print(f"  ✅  Both robots OK — moving URI…")
                        if not ensure_both_running_or_confirm(
                            uri_dashboard,
                            ayal_dashboard,
                            context=f"{dir_name} step {steps + 1} candidate move",
                        ):
                            print("User chose to stop exploration.")
                            sys.exit(0)
                        uri.control.moveL(candidate, MY_SPEED, MY_ACCELERATION, False)

                        pose = list(uri.recieve.getActualTCPPose())
                        steps += 1
                        global_step_counter += 1

                        # 5. Track manipulability
                        try:
                            q_uri_now = list(uri.recieve.getActualQ())
                            m_uri = _manipulability(q_uri_now)

                            qnear_ayal = list(ayal.recieve.getActualQ())
                            q_ayal = ayal.control.getInverseKinematics(
                                list(ayal_candidate), qnear_ayal
                            )
                            m_ayal = _manipulability(q_ayal)

                            print(f"       URI  m = {m_uri:.6f}")
                            print(f"       AYAL m = {m_ayal:.6f}")

                            if m_ayal > best_m_ayal:
                                best_m_ayal = m_ayal
                                best_uri_pose = list(candidate)
                        except RuntimeError:
                            pass

                        # 6. Every N steps → stop force mode, run find_center, re-enable, log
                        if global_step_counter >= CENTER_EVERY_N_STEPS:
                            global_step_counter = 0
                            print(f"\n  🔍  Running find_center on AYAL (every {CENTER_EVERY_N_STEPS} steps)…")

                            # Stop force mode so find_center can move AYAL freely
                            if hasattr(ayal.control, "forceModeStop"):
                                ayal.control.forceModeStop()
                            else:
                                ayal.control.stopScript()

                            # First pass: fast direct-mode centering (find_center_3)
                            print("  [v3 direct] Running find_center_3 direct mode…")
                            run_find_center_v3(ayal)

                            # Second pass: original probe-based centering (find_center)
                            center_result = run_find_center(ayal)

                            # Calibrate: compute Ayal-base in Uri-base frame
                            uri_tcp  = list(uri.recieve.getActualTCPPose())
                            ayal_tcp = list(ayal.recieve.getActualTCPPose())
                            calibration_pose = list(gemini.calculate_ayal_in_uri(uri_tcp, ayal_tcp))
                            print(f"  📐  Calibration (Ayal in Uri): {fmt(calibration_pose)}")

                            # Log the centering result together with current URI state
                            exploration_log.append({
                                "loop":              loop_idx,
                                "direction":         dir_name,
                                "exploration_step":  steps,
                                "uri_pose":          uri_tcp,
                                "ayal_tcp":          ayal_tcp,
                                "calibration":       calibration_pose,
                                "ayal_center":       center_result["overall_center"],
                                "ayal_final_pose":   center_result["final_pose"],
                                "center_iterations": center_result["iterations"],
                                "center_details":    {
                                    ax: {
                                        "center": r["center"],
                                        "range":  r["range"],
                                        "steps_plus":  r["steps_plus"],
                                        "steps_minus": r["steps_minus"],
                                    }
                                    for ax, r in center_result["results"].items()
                                },
                            })

                            # Re-enable force mode on AYAL
                            print("  Re-enabling force mode on AYAL…")
                            ayal.control.forceMode(
                                FORCE_TASK_FRAME, FORCE_SELECTION,
                                FORCE_WRENCH, FORCE_TYPE, FORCE_LIMITS
                            )

                    results[dir_name] = steps

                    # Return to original position before next direction
                    print(f"\n  ↩  Returning to start position…")
                    if not ensure_both_running_or_confirm(
                        uri_dashboard,
                        ayal_dashboard,
                        context=f"return to start after {dir_name}",
                    ):
                        print("User chose to stop exploration.")
                        sys.exit(0)
                    uri.control.moveL(current_pose, MY_SPEED / 3, MY_ACCELERATION / 3, False)
                    time.sleep(0.01)

                # After both directions of this axis, move to best pose
                if best_uri_pose != list(current_pose):
                    print(f"\n  ★  Best AYAL m on {axis}-axis = {best_m_ayal:.6f} "
                          f"at URI pose {fmt(best_uri_pose)}")
                    print(f"     Moving URI to best pose…")
                    if not ensure_both_running_or_confirm(
                        uri_dashboard,
                        ayal_dashboard,
                        context=f"move to best pose for axis {axis}",
                    ):
                        print("User chose to stop exploration.")
                        sys.exit(0)
                    uri.control.moveL(best_uri_pose, MY_SPEED / 3, MY_ACCELERATION / 3, False)
                    time.sleep(0.01)
                    current_pose = list(uri.recieve.getActualTCPPose())
                    print(f"     New starting pose: {fmt(current_pose)}")
                else:
                    print(f"\n  ★  {axis}-axis: start position was already best "
                          f"(m = {best_m_ayal:.6f})")

            # ── Per-loop summary ──────────────────────────────────────────────
            print(f"\n{'=' * 60}")
            print(f"LOOP {loop_idx} SUMMARY")
            print(f"{'=' * 60}")
            for name, count in results.items():
                axis_name = name.lstrip("+-")
                if axis_name in ("RX", "RY", "RZ"):
                    total = count * ROT_STEP
                    print(f"  {name}: {count} steps  ({total:.3f} rad / {np.degrees(total):.1f}°)")
                else:
                    dist_mm = count * STEP_SIZE * 1000
                    print(f"  {name}: {count} steps  ({dist_mm:.0f} mm)")

        # ── Final summary ─────────────────────────────────────────────────────
        print(f"\nFinal URI pose: {fmt(current_pose)}")
        print(f"Total centering samples collected: {len(exploration_log)}")

    finally:
        # Stop force mode on AYAL before disconnecting
        print("\nStopping force mode on AYAL…")
        try:
            if hasattr(ayal.control, "forceModeStop"):
                ayal.control.forceModeStop()
            else:
                ayal.control.stopScript()
        except Exception:
            pass

        # Save exploration log
        if exploration_log:
            with open(OUTPUT_JSON, "w") as f:
                json.dump(exploration_log, f, indent=2)
            print(f"Exploration log saved to {OUTPUT_JSON}")
        else:
            print("No centering samples were collected.")

        uri.disconnect()
        ayal.disconnect()
        try:
            uri_dashboard.disconnect()
        except Exception:
            pass
        try:
            ayal_dashboard.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
