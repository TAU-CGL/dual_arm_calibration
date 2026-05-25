#!/usr/bin/env python3
"""
Find the center point for Ayal's TCP by probing each axis (±X, ±Y, ±Z, ±RX, ±RY, ±RZ).

For each axis the robot steps incrementally in the + direction until a force/moment
threshold is exceeded (contact detected), records that limit pose, returns to the
start, then probes the − direction the same way.  The midpoint of the two limit
poses is reported as the "center" for that axis.

After all axes are done the robot moves to the final computed center and a summary
is printed and saved to find_center_log.json.
"""

import sys
import json
import math
import time
from pathlib import Path
from datetime import datetime

import numpy as np
import uri_if

# ── Tee stdout/stderr to a log file (overwritten each run) ────────────────────
_LOG_FILE = Path(__file__).resolve().parent / "find_center.log"

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

if __name__ == "__main__":
    _log_fh = open(_LOG_FILE, "w")          # truncate on every run
    sys.stdout = _Tee(sys.__stdout__, _log_fh)
    sys.stderr = _Tee(sys.__stderr__, _log_fh)

# ── Path setup so we can import helpers ────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "calibration"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "uris_gui"))

import gemini_calc_v2 as gemini
from config import DEFAULT_SPEED, DEFAULT_ACCELERATION
from uri_calibration.src.is_valid_pose import _manipulability, MIN_MANIPULABILITY_AYAL

# ── Configuration ──────────────────────────────────────────────────────────────
AYAL_HOST = "192.168.57.101"

# Step sizes
STEP_SIZE = 0.0001       # metres per step for X / Y / Z
ROT_STEP  = 0.01        # radians per step for RX / RY / RZ

# Force / moment thresholds – probing stops when the *relevant* component exceeds
# this value (absolute).
FORCE_THRESHOLD  = 10.0   # Newtons  (for X, Y, Z)
MOMENT_THRESHOLD = 1.0    # Newton-metres (for RX, RY, RZ)

# Cross-axis safety: if any *other* axis exceeds this multiple of its
# normal threshold, probing aborts with an error.
CROSS_AXIS_MULTIPLIER = 3.0

# Motion parameters (conservative)
SPEED = DEFAULT_SPEED / 2
ACCEL = DEFAULT_ACCELERATION / 2

# Time to wait after each incremental move so the F/T sensor stabilises
SETTLE_TIME = 0.05   # seconds

# Safety cap – maximum steps in one direction before giving up
MAX_STEPS = 50

# Which axes to probe (set False to skip an axis)
PROBE_X  = True
PROBE_Y  = True
PROBE_Z  = True
PROBE_RX = True
PROBE_RY = True
PROBE_RZ = True

# Z-axis: only probe the + direction (tool pushes deeper); the start
# pose is used as the − limit so the "center" becomes halfway between
# start and the +Z contact.
Z_PLUS_ONLY = True

# Convergence: repeat the full axis sweep until the center moves less
# than these thresholds between iterations.
CONVERGENCE_POS = 0.0002    # metres  — position axes (X, Y, Z)
CONVERGENCE_ROT = 0.005     # radians — rotation axes (RX, RY, RZ)
MAX_ITERATIONS  = 10        # safety cap on centering iterations

# Output file
OUTPUT_JSON = Path(__file__).resolve().parent / "find_center_log.json"

FORCE_LABELS = ["FX", "FY", "FZ", "MX", "MY", "MZ"]
MIN_MANIPULABILITY = MIN_MANIPULABILITY_AYAL

# ── Axis descriptors ──────────────────────────────────────────────────────────
# (name, move_index, step_size, is_rotation,
#  force_index_to_check, force_threshold,
#  moment_index_to_check, moment_threshold)
#
# When probing axis i (or Ri) we check BOTH F_i and M_i against their
# respective thresholds; probing stops when *either* is exceeded.
AXES = [
    ("RX", 3, ROT_STEP,  True,  0, FORCE_THRESHOLD, 3, MOMENT_THRESHOLD),
    ("RY", 4, ROT_STEP,  True,  1, FORCE_THRESHOLD, 4, MOMENT_THRESHOLD),
    ("RZ", 5, ROT_STEP,  True,  2, FORCE_THRESHOLD, 5, MOMENT_THRESHOLD),
    ("X",  0, STEP_SIZE, False, 0, FORCE_THRESHOLD, 3, MOMENT_THRESHOLD),
    ("Y",  1, STEP_SIZE, False, 1, FORCE_THRESHOLD, 4, MOMENT_THRESHOLD),
    ("Z",  2, STEP_SIZE, False, 2, FORCE_THRESHOLD, 5, MOMENT_THRESHOLD),
]
_AXIS_ENABLED = {
    "X":  PROBE_X,
    "Y":  PROBE_Y,
    "Z":  PROBE_Z,
    "RX": PROBE_RX,
    "RY": PROBE_RY,
    "RZ": PROBE_RZ,
}
AXES = [a for a in AXES if _AXIS_ENABLED.get(a[0], False)]

# ── Helpers ────────────────────────────────────────────────────────────────────
def fmt_pose(pose):
    """Pretty-print a 6-element pose."""
    return "[" + ", ".join(f"{v: .6f}" for v in pose) + "]"

def fmt_forces(forces):
    """Pretty-print all 6 force/moment components."""
    labels = FORCE_LABELS
    return "  ".join(f"{labels[i]}={forces[i]:+.3f}" for i in range(6))

def forces_in_tcp_frame(ayal):
    """
    Read the F/T sensor and transform the wrench into the TCP frame.

    getActualTCPForce() returns [Fx, Fy, Fz, Mx, My, Mz] in the **base**
    frame. We use the full wrench transform (including translation term)
    into the tool frame.
    """
    forces_base = ayal.recieve.getActualTCPForce()
    pose = ayal.recieve.getActualTCPPose()
    return gemini.wrench_trans(
        forces_base[:6],
        pose[:6],
        base_to_tcp=True,
        include_translation=True,
    )

def step_pose(pose, axis_idx, delta, is_rotation):
    """
    Return a new pose that is *pose* shifted by *delta* along *axis_idx*,
    expressed **in the TCP frame**.

    The step is applied by composing  T_current @ T_step  so that the
    movement is always relative to the tool's own axes, not the base frame.
    """
    T_current = gemini.pose_to_T(pose)

    # Build the incremental transform in TCP frame
    T_step = np.eye(4)
    if not is_rotation:
        # Pure translation along TCP-x (0), TCP-y (1), or TCP-z (2)
        T_step[axis_idx, 3] = delta
    else:
        # Pure rotation about TCP-x (3→0), TCP-y (4→1), or TCP-z (5→2)
        step_rv = [0.0, 0.0, 0.0]
        step_rv[axis_idx - 3] = delta
        T_step[:3, :3] = gemini.rotvec_to_R(step_rv)

    T_new = T_current @ T_step
    return list(gemini.T_to_pose(T_new))

def _assert_manipulability_before_move(robot, target_pose, min_m=MIN_MANIPULABILITY):
    """Raise RuntimeError if target pose is near-singular by manipulability metric."""
    try:
        qnear = list(robot.recieve.getActualQ())
        q_target = robot.control.getInverseKinematics(target_pose, qnear)
        m_target = _manipulability(q_target)
    except Exception as e:
        raise RuntimeError(f"Manipulability pre-check failed (IK error): {e}")

    if m_target < min_m:
        raise RuntimeError(
            f"Manipulability too low before move: m={m_target:.6f} < {min_m:.6f}"
        )

def _guarded_moveL(robot, target_pose, speed, accel, async_flag=False):
    _assert_manipulability_before_move(robot, target_pose)
    robot.control.moveL(target_pose, speed, accel, async_flag)

def midpoint_pose(pose_a, pose_b):
    """
    Compute the element-wise midpoint of two 6-DOF poses.

    For position components this is exact.  For rotation-vector components
    it is a reasonable approximation when the two orientations are close.
    """
    return [(a + b) / 2.0 for a, b in zip(pose_a, pose_b)]

# ── Probing logic ─────────────────────────────────────────────────────────────
def probe_direction(ayal, start_pose, axis_idx, sign, step, is_rotation,
                    f_idx, f_thresh, m_idx, m_thresh):
    """
    Incrementally move Ayal in the given direction until EITHER the paired
    force component (f_idx) exceeds f_thresh OR the paired moment component
    (m_idx) exceeds m_thresh, or MAX_STEPS is reached.

    Returns
    -------
    limit_pose : list[float]
        The TCP pose at which a threshold was exceeded (or last pose if capped).
    steps_taken : int
    force_log : list[dict]
        Per-step record with pose, all 6 force components, and the step number.
    """
    pose = list(start_pose)
    delta = sign * step
    force_log = []

    for step_num in range(1, MAX_STEPS + 1):
        candidate = step_pose(pose, axis_idx, delta, is_rotation)

        # Move there (let exceptions propagate → script stops cleanly)
        _guarded_moveL(ayal, candidate, SPEED, ACCEL, False)

        time.sleep(SETTLE_TIME)

        # Read all forces — rotated into the TCP frame
        forces = forces_in_tcp_frame(ayal)
        if len(forces) < 6:
            raise RuntimeError(f"forces_in_tcp_frame() returned invalid data: {forces}")
        actual_pose = list(ayal.recieve.getActualTCPPose())

        # Log this step
        force_log.append({
            "step": step_num,
            "pose": actual_pose,
            "forces": forces,
        })

        # Print all forces + highlight the checked pair
        f_val = abs(forces[f_idx])
        m_val = abs(forces[m_idx])
        print(f"    step {step_num:3d}  {fmt_forces(forces)}")
        print(f"             |{FORCE_LABELS[f_idx]}|={f_val:.3f}/{f_thresh:.1f}  "
              f"|{FORCE_LABELS[m_idx]}|={m_val:.3f}/{m_thresh:.1f}")

        # ── Cross-axis safety check ───────────────────────────────────────
        # If any force/moment on an axis we are NOT probing exceeds 2×
        # its normal threshold, something unexpected is happening → abort.
        cross_f_limit = FORCE_THRESHOLD * CROSS_AXIS_MULTIPLIER
        cross_m_limit = MOMENT_THRESHOLD * CROSS_AXIS_MULTIPLIER
        for ci in range(6):
            if ci == f_idx or ci == m_idx:
                continue  # skip the pair we're already checking
            limit = cross_f_limit if ci < 3 else cross_m_limit
            if abs(forces[ci]) >= limit:
                print(f"    ⚠  CROSS-AXIS ALERT: |{FORCE_LABELS[ci]}| = "
                      f"{abs(forces[ci]):.3f} exceeds {limit:.1f} "
                      f"(2× threshold) at step {step_num} — stopping this direction.")
                return actual_pose, step_num, force_log

        if f_val >= f_thresh:
            print(f"    ✅  Force threshold ({FORCE_LABELS[f_idx]}) reached at step {step_num}")
            return actual_pose, step_num, force_log
        if m_val >= m_thresh:
            print(f"    ✅  Moment threshold ({FORCE_LABELS[m_idx]}) reached at step {step_num}")
            return actual_pose, step_num, force_log

        pose = candidate  # advance

    print(f"    ⚠  MAX_STEPS ({MAX_STEPS}) reached without hitting threshold")
    return list(ayal.recieve.getActualTCPPose()), MAX_STEPS, force_log

def return_to(ayal, target_pose, label="start"):
    """Move back to *target_pose* safely."""
    print(f"    ↩  Returning to {label} pose …")
    _guarded_moveL(ayal, target_pose, SPEED, ACCEL, False)
    time.sleep(SETTLE_TIME)

# ── Reusable centering routine ─────────────────────────────────────────────────
def _has_converged(prev_center, new_center):
    """Return True if the center has not moved beyond the convergence thresholds."""
    for axis_name, axis_idx, _step, is_rot, *_rest in AXES:
        tol = CONVERGENCE_ROT if is_rot else CONVERGENCE_POS
        if abs(new_center[axis_idx] - prev_center[axis_idx]) > tol:
            return False
    return True

def _run_single_pass(ayal, current_pose):
    """
    One full sweep over all enabled axes.  After each axis is probed ±,
    the robot moves to that axis's center before proceeding to the next.

    Returns (overall_center, results_dict).
    """
    start_pose = list(current_pose)
    results = {}

    for axis_name, axis_idx, step, is_rot, f_idx, f_thresh, m_idx, m_thresh in AXES:
        print(f"\n  {'─' * 50}")
        print(f"    Probing axis: {axis_name}  "
              f"(|{FORCE_LABELS[f_idx]}|≥{f_thresh} or |{FORCE_LABELS[m_idx]}|≥{m_thresh})")

        # ── Probe + direction ──────────────────────────────────────────────
        print(f"\n    ▶ +{axis_name}")
        limit_plus, steps_plus, flog_plus = probe_direction(
            ayal, current_pose, axis_idx, +1, step, is_rot,
            f_idx, f_thresh, m_idx, m_thresh,
        )

        # ── Probe − direction ──────────────────────────────────────────────
        if axis_name == "Z" and Z_PLUS_ONLY:
            print(f"\n    ▶ −{axis_name}  [SKIPPED — Z_PLUS_ONLY]")
            limit_minus = list(current_pose)
            steps_minus = 0
            flog_minus = []
        else:
            print(f"\n    ▶ −{axis_name}")
            limit_minus, steps_minus, flog_minus = probe_direction(
                ayal, limit_plus, axis_idx, -1, step, is_rot,
                f_idx, f_thresh, m_idx, m_thresh,
            )

        # ── Compute per-axis center ────────────────────────────────────────
        if limit_plus is not None and limit_minus is not None:
            # Symmetric clamping: use the shorter reach on both sides
            # so the center isn't biased toward the longer side.
            reach_plus  = abs(limit_plus[axis_idx]  - current_pose[axis_idx])
            reach_minus = abs(limit_minus[axis_idx] - current_pose[axis_idx])
            min_reach = min(reach_plus, reach_minus)
            if reach_plus != reach_minus:
                print(f"\n    Asymmetric reach on {axis_name}: "
                      f"+{reach_plus:.6f} / −{reach_minus:.6f} → "
                      f"clamping both to {min_reach:.6f}")
            # Build symmetric limit poses for midpoint calculation
            sym_plus  = list(current_pose)
            sym_minus = list(current_pose)
            sign_plus  = +1 if limit_plus[axis_idx]  >= current_pose[axis_idx] else -1
            sign_minus = -1 if limit_minus[axis_idx] <= current_pose[axis_idx] else +1
            sym_plus[axis_idx]  = current_pose[axis_idx] + sign_plus  * min_reach
            sym_minus[axis_idx] = current_pose[axis_idx] + sign_minus * min_reach
            center = midpoint_pose(sym_plus, sym_minus)
            range_val = 2 * min_reach
            unit = "rad" if is_rot else "m"
            print(f"\n    Center {axis_name}: {fmt_pose(center)}")
            print(f"    Range : {range_val:.6f} {unit}  "
                  f"(+{steps_plus} / −{steps_minus} steps)")
        else:
            center = list(current_pose)
            range_val = None

        results[axis_name] = {
            "limit_plus":    limit_plus,
            "limit_minus":   limit_minus,
            "steps_plus":    steps_plus,
            "steps_minus":   steps_minus,
            "center":        center,
            "range":         range_val,
            "force_log_plus":  flog_plus,
            "force_log_minus": flog_minus,
        }

        # ── Move to this axis's center before probing the next axis ────────
        # Update only the component(s) for this axis so we progressively
        # refine the pose.
        if center is not None:
            current_pose[axis_idx] = center[axis_idx]
            print(f"    → Moving to {axis_name} center …")
            _guarded_moveL(ayal, current_pose, SPEED, ACCEL, False)
            time.sleep(SETTLE_TIME)

    # overall_center is simply the progressively updated current_pose
    overall_center = list(current_pose)
    return overall_center, results, start_pose

def run_find_center(ayal, zero_ft=False):
    """
    Run the iterative center-finding procedure on an already-connected robot.

    Repeats the full axis sweep until the computed center stops moving beyond
    the convergence thresholds, or MAX_ITERATIONS is reached.

    Parameters
    ----------
    ayal : sim_uri.RMPLAB_Uri
        An already-connected robot object.
    zero_ft : bool
        If True, zero the F/T sensor before the first iteration.

    Returns
    -------
    dict with keys:
        "start_pose", "overall_center", "final_pose",
        "results" (per-axis details from the last iteration),
        "iterations" (number of sweeps performed)
    """
    if zero_ft:
        print("Zeroing F/T sensor …")
        ayal.control.zeroFtSensor()
        time.sleep(SETTLE_TIME)

    initial_pose = list(ayal.recieve.getActualTCPPose())
    print(f"  find_center initial pose: {fmt_pose(initial_pose)}")

    prev_center = list(initial_pose)
    all_results = None

    for iteration in range(1, MAX_ITERATIONS + 1):
        print(f"\n{'═' * 60}")
        print(f"  CENTERING ITERATION {iteration} / {MAX_ITERATIONS}")
        print(f"{'═' * 60}")

        current_pose = list(ayal.recieve.getActualTCPPose())
        overall_center, results, start_pose = _run_single_pass(ayal, current_pose)
        all_results = results

        print(f"\n  Iteration {iteration} center: {fmt_pose(overall_center)}")

        # Move to the new overall center
        _guarded_moveL(ayal, overall_center, SPEED, ACCEL, False)
        time.sleep(SETTLE_TIME)

        # Check convergence
        if iteration > 1 and _has_converged(prev_center, overall_center):
            print(f"  ✅  Converged after {iteration} iterations "
                  f"(pos < {CONVERGENCE_POS} m, rot < {CONVERGENCE_ROT} rad)")
            break
        else:
            # Show per-axis deltas
            for axis_name, axis_idx, _step, is_rot, *_rest in AXES:
                delta = abs(overall_center[axis_idx] - prev_center[axis_idx])
                tol = CONVERGENCE_ROT if is_rot else CONVERGENCE_POS
                status = "✓" if delta <= tol else "✗"
                unit = "rad" if is_rot else "m"
                print(f"    {status} Δ{axis_name} = {delta:.6f} {unit}  (tol {tol})")

        prev_center = list(overall_center)
    else:
        print(f"  ⚠  MAX_ITERATIONS ({MAX_ITERATIONS}) reached without full convergence")

    final_pose = list(ayal.recieve.getActualTCPPose())
    print(f"\n  Final pose: {fmt_pose(final_pose)}")

    return {
        "start_pose":     initial_pose,
        "overall_center": overall_center,
        "final_pose":     final_pose,
        "results":        all_results,
        "iterations":     iteration,
    }

# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    print("=" * 70)
    print("  Ayal TCP Center Finder — force-threshold probing")
    print("=" * 70)
    print(f"  Host           : {AYAL_HOST}")
    print(f"  Step (XYZ)     : {STEP_SIZE} m")
    print(f"  Step (RXRYRZ)  : {ROT_STEP} rad")
    print(f"  Force threshold: {FORCE_THRESHOLD} N")
    print(f"  Moment threshold: {MOMENT_THRESHOLD} Nm")
    print(f"  Max steps/dir  : {MAX_STEPS}")
    print(f"  Speed / Accel  : {SPEED} / {ACCEL}")
    print(f"  Settle time    : {SETTLE_TIME} s")
    print()

    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    try:
        ayal.connect(False)
    except Exception as e:
        print(f"ERROR connecting to Ayal ({AYAL_HOST}): {e}")
        sys.exit(1)

    print(f"Connected to Ayal at {AYAL_HOST}")

    try:
        result = run_find_center(ayal)

        # ── Summary table ─────────────────────────────────────────────────
        results = result["results"]
        print(f"\n{'=' * 70}")
        print(f"{'Axis':>4}  {'−Limit':>12}  {'+Limit':>12}  {'Center':>12}  {'Range':>12}")
        print(f"{'─' * 70}")
        for axis_name, axis_idx, step, is_rot, *_thresholds in AXES:
            r = results[axis_name]
            unit = "rad" if is_rot else "m"
            lm = f"{r['limit_minus'][axis_idx]:.6f}" if r["limit_minus"] else "N/A"
            lp = f"{r['limit_plus'][axis_idx]:.6f}"  if r["limit_plus"]  else "N/A"
            cv = f"{r['center'][axis_idx]:.6f}"       if r["center"]      else "N/A"
            rv = f"{r['range']:.6f} {unit}"           if r["range"] is not None else "N/A"
            print(f"{axis_name:>4}  {lm:>12}  {lp:>12}  {cv:>12}  {rv:>12}")
        print(f"{'=' * 70}")

        # ── Save to JSON ──────────────────────────────────────────────────
        log_entry = {
            "timestamp":      datetime.now().isoformat(),
            "start_pose":     result["start_pose"],
            "overall_center": result["overall_center"],
            "final_pose":     result["final_pose"],
            "config": {
                "step_size":        STEP_SIZE,
                "rot_step":         ROT_STEP,
                "force_threshold":  FORCE_THRESHOLD,
                "moment_threshold": MOMENT_THRESHOLD,
                "max_steps":        MAX_STEPS,
                "speed":            SPEED,
                "accel":            ACCEL,
                "settle_time":      SETTLE_TIME,
            },
            "axes": {},
        }
        for axis_name, axis_idx, *_rest in AXES:
            r = results[axis_name]
            log_entry["axes"][axis_name] = {
                "limit_plus":      r["limit_plus"],
                "limit_minus":     r["limit_minus"],
                "steps_plus":      r["steps_plus"],
                "steps_minus":     r["steps_minus"],
                "center":          r["center"],
                "range":           r["range"],
                "force_log_plus":  r["force_log_plus"],
                "force_log_minus": r["force_log_minus"],
            }

        log_data = []
        if OUTPUT_JSON.exists():
            with open(OUTPUT_JSON, "r") as f:
                log_data = json.load(f)
        log_data.append(log_entry)
        with open(OUTPUT_JSON, "w") as f:
            json.dump(log_data, f, indent=2)
        print(f"\nResults saved to {OUTPUT_JSON}")

    finally:
        ayal.disconnect()
        print("\nDisconnected from Ayal.")

if __name__ == "__main__":
    main()