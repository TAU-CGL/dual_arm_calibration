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
import uri_if
import numpy as np
from pathlib import Path
from datetime import datetime

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

from uri_calibration.src import utils
from uri_gui.src.config import DEFAULT_SPEED, DEFAULT_ACCELERATION
from uri_calibration.src.is_valid_pose import _manipulability, MIN_MANIPULABILITY_AYAL

# ── Configuration ──────────────────────────────────────────────────────────────
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
    return utils.wrench_trans(
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
    T_current = utils.pose_to_T(pose)

    # Build the incremental transform in TCP frame
    T_step = np.eye(4)
    if not is_rotation:
        # Pure translation along TCP-x (0), TCP-y (1), or TCP-z (2)
        T_step[axis_idx, 3] = delta
    else:
        # Pure rotation about TCP-x (3→0), TCP-y (4→1), or TCP-z (5→2)
        step_rv = [0.0, 0.0, 0.0]
        step_rv[axis_idx - 3] = delta
        T_step[:3, :3] = utils.rotvec_to_R(step_rv)

    T_new = T_current @ T_step
    return list(utils.T_to_pose(T_new))

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

def wrench_magnitude(forces):
    """||F|| + ||M|| — scalar measure of how far from zero the wrench is."""
    f_mag = math.sqrt(forces[0]**2 + forces[1]**2 + forces[2]**2)
    m_mag = math.sqrt(forces[3]**2 + forces[4]**2 + forces[5]**2)
    return f_mag + m_mag

def check_safety(forces, label=""):
    """Raise RuntimeError if any component exceeds the safety cap."""
    for i in range(3):
        if abs(forces[i]) >= FORCE_SAFETY:
            msg = (f"  ❌  SAFETY: |{FORCE_LABELS[i]}| = {abs(forces[i]):.3f} "
                   f"≥ {FORCE_SAFETY} N  {label}")
            print(msg)
            raise RuntimeError(msg)
    for i in range(3, 6):
        if abs(forces[i]) >= MOMENT_SAFETY:
            msg = (f"  ❌  SAFETY: |{FORCE_LABELS[i]}| = {abs(forces[i]):.3f} "
                   f"≥ {MOMENT_SAFETY} Nm  {label}")
            print(msg)
            raise RuntimeError(msg)

# ── direct ─────────────────────────────────────────────────────────────
def run_find_center_direct(robot, zero_ft=False):
    """
    Iteratively move toward zero wrench using direct mode.

    At each step: read the current F/T, normalise forces and moments to a
    common scale, pick the dominant component, and step *against* it.
    Stop when ||W|| can no longer be reduced (or all components are within
    noise).
    """
    if zero_ft:
        print("Zeroing F/T sensor …")
        robot.control.zeroFtSensor()
        time.sleep(SETTLE_TIME)

    # Measure sensor noise while stationary
    noise_std, noise_floor = measure_noise(robot)

    initial_pose = list(robot.recieve.getActualTCPPose())
    forces_start = forces_in_tcp_frame(robot)
    check_safety(forces_start, label="at start")
    wmag_start = wrench_magnitude(forces_start)

    print(f"  Start pose:   {fmt_pose(initial_pose)}")
    print(f"  Start forces: {fmt_forces(forces_start)}")
    print(f"  Start ||W|| = {wmag_start:.4f}")
    print(f"  Force/moment scale: {FORCE_SCALE} N / {MOMENT_SCALE} Nm  "
          f"(ratio {FORCE_SCALE / MOMENT_SCALE:.1f}:1)")

    current_pose = list(initial_pose)
    wmag_current = wmag_start
    step_log = []
    total_steps = 0
    step_scale = 1.0          # halved when improvement ≤ σ (fine phase)

    for step_num in range(1, MAX_STEPS + 1):
        print(f"\n{'─' * 60}")
        print(f"  STEP {step_num} / {MAX_STEPS}   ||W|| = {wmag_current:.4f}  "
              f"(step_scale={step_scale})")
        print(f"{'─' * 60}")

        # Read current forces
        forces_here = forces_in_tcp_frame(robot)
        check_safety(forces_here, label=f"at step {step_num}")
        print(f"  Forces: {fmt_forces(forces_here)}")

        # Print normalised scores for all components
        print("  Normalised scores:")
        for fi in range(6):
            if fi not in _AXIS_BY_FORCE_IDX:
                continue
            val = forces_here[fi]
            sig = abs(val) - abs(noise_std[fi])
            sc = FORCE_SCALE if fi < 3 else MOMENT_SCALE
            norm = max(0.0, sig) / sc
            marker = "" if sig <= 0 else f"  ← signal {sig:.4f}"
            print(f"    {FORCE_LABELS[fi]:>2} = {val:+.4f}  "
                  f"σ={noise_std[fi]:.4f}  norm={norm:.4f}{marker}")

        # Pick the dominant component
        dominant = pick_dominant_axis(forces_here, noise_std)
        if dominant is None:
            print(f"\n  ✅  All components within noise — centered.")
            break

        force_idx, axis_entry, sign, score = dominant
        axis_name, axis_idx, _probe_step, move_step, is_rot = axis_entry
        dir_label = f"{'+'if sign > 0 else '-'}{axis_name}"

        print(f"\n  ★  Dominant: {FORCE_LABELS[force_idx]}={forces_here[force_idx]:+.4f}  "
              f"(norm={score:.4f}) → step {dir_label}")

        # Remember the targeted component's value before stepping
        comp_before = abs(forces_here[force_idx])
        comp_noise  = abs(noise_std[force_idx])

        # Take a step (scaled) against the dominant force/moment
        actual_step = move_step * step_scale
        new_pose = step_pose(current_pose, axis_idx, sign * actual_step, is_rot)
        _guarded_moveL(robot, new_pose, SPEED, ACCEL, False)
        time.sleep(SETTLE_TIME)

        forces_after = forces_in_tcp_frame(robot)
        check_safety(forces_after, label=f"after step {step_num}")
        wmag_after = wrench_magnitude(forces_after)
        actual_pose = list(robot.recieve.getActualTCPPose())

        comp_after = abs(forces_after[force_idx])
        comp_improvement = comp_before - comp_after

        print(f"  After step: ||W|| = {wmag_after:.4f}  "
              f"(Δ = {wmag_after - wmag_current:+.4f})")
        print(f"  {FORCE_LABELS[force_idx]}: {comp_before:.4f} → {comp_after:.4f}  "
              f"(Δ = {-comp_improvement:+.4f}, σ = {comp_noise:.4f})")
        print(f"  Forces: {fmt_forces(forces_after)}")

        # # Check improvement on the targeted component only
        # if comp_improvement < 0.1:
        #     # Step made the targeted component worse — revert
        #     print(f"\n  ↩  |{FORCE_LABELS[force_idx]}| worsened "
        #           f"({comp_improvement:+.4f}) — reverting.")
        #     robot.control.moveL(current_pose, SPEED, ACCEL, False)
        #     time.sleep(SETTLE_TIME)
        #     step_log.append({
        #         "step": step_num, "direction": dir_label,
        #         "dominant": FORCE_LABELS[force_idx],
        #         "comp_before": round(comp_before, 4),
        #         "comp_after": round(comp_after, 4),
        #         "wmag_before": wmag_current, "wmag_after": wmag_after,
        #         "forces_after": list(forces_after),
        #         "pose_before": current_pose, "pose_after": actual_pose,
        #         "reverted": True,
        #     })
        #     total_steps = step_num
        #     continue

        if comp_improvement <= comp_noise:
            if step_scale > 0.5:
                # First time hitting noise floor — halve steps for fine phase
                step_scale *= 0.5
                print(f"\n  🔬  |{FORCE_LABELS[force_idx]}| improvement within noise "
                      f"({comp_improvement:.4f} ≤ σ {comp_noise:.4f}) "
                      f"— halving step size → step_scale={step_scale}")
            elif comp_improvement <= comp_noise * 0.5:
                # Already at half-step and improvement ≤ 0.5σ — centered
                print(f"\n  ✅  |{FORCE_LABELS[force_idx]}| improvement within 0.5σ "
                      f"({comp_improvement:.4f} ≤ 0.5·σ {comp_noise * 0.5:.4f}) "
                      f"at step_scale={step_scale} — centered.")
                total_steps = step_num
                step_log.append({
                    "step": step_num, "direction": dir_label,
                    "dominant": FORCE_LABELS[force_idx],
                    "comp_before": round(comp_before, 4),
                    "comp_after": round(comp_after, 4),
                    "wmag_before": wmag_current, "wmag_after": wmag_after,
                    "forces_after": list(forces_after),
                    "pose_before": current_pose, "pose_after": actual_pose,
                    "step_scale": step_scale,
                })
                break
            else:
                # At half-step but improvement still > 0.5σ — keep going
                print(f"\n  🔬  |{FORCE_LABELS[force_idx]}| improvement "
                      f"({comp_improvement:.4f}) still > 0.5·σ ({comp_noise * 0.5:.4f}) "
                      f"at step_scale={step_scale} — continuing.")

        step_log.append({
            "step":          step_num,
            "direction":     dir_label,
            "dominant":      FORCE_LABELS[force_idx],
            "norm_score":    round(score, 4),
            "comp_before":   round(comp_before, 4),
            "comp_after":    round(comp_after, 4),
            "wmag_before":   wmag_current,
            "wmag_after":    wmag_after,
            "forces_after":  list(forces_after),
            "pose_before":   current_pose,
            "pose_after":    actual_pose,
            "step_scale":    step_scale,
        })

        current_pose = actual_pose
        wmag_current = wmag_after
        total_steps = step_num
    else:
        print(f"\n  ⚠  MAX_STEPS ({MAX_STEPS}) reached")
        total_steps = MAX_STEPS

    final_pose = list(robot.recieve.getActualTCPPose())
    forces_final = forces_in_tcp_frame(robot)
    wmag_final = wrench_magnitude(forces_final)

    print(f"\n  Final pose:   {fmt_pose(final_pose)}")
    print(f"  Final forces: {fmt_forces(forces_final)}")
    print(f"  Final ||W|| = {wmag_final:.4f}  (started at {wmag_start:.4f})")

    return {
        "mode":            "direct",
        "start_pose":      initial_pose,
        "final_pose":      final_pose,
        "total_steps":     total_steps,
        "wmag_start":      wmag_start,
        "wmag_final":      wmag_final,
        "forces_start":    list(forces_start),
        "forces_final":    list(forces_final),
        "noise_std":       noise_std,
        "noise_floor":     noise_floor,
        "step_log":        step_log,
    }

# ── by value ─────────────────────────────────────────────────────────────
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

def run_find_center_by_value(ayal, zero_ft=False):
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

# ── by grad ─────────────────────────────────────────────────────────
# Probe step sizes (test-step to measure wrench in each direction)
PROBE_STEP_SIZE = 0.0002    # metres  — X / Y / Z
PROBE_ROT_STEP  = 0.01      # radians — RX / RY / RZ

# Move step sizes (actual step taken in the chosen direction)
MOVE_STEP_SIZE = 0.0001     # metres
MOVE_ROT_STEP  = 0.005      # radians

# Absolute safety caps — abort if any component exceeds these
FORCE_SAFETY  = 20.0   # N
MOMENT_SAFETY = 2.0    # Nm

# Motion parameters (conservative)
SPEED = DEFAULT_SPEED / 2
ACCEL = DEFAULT_ACCELERATION / 2

# Time to wait after each move for F/T sensor to stabilise
SETTLE_TIME = 0.05   # seconds

# Maximum total descent steps
MAX_STEPS = 200

# Number of static samples to measure sensor noise at startup
NOISE_SAMPLES = 100

# Which axes to include
PROBE_X  = True
PROBE_Y  = True
PROBE_Z  = True
PROBE_RX = True
PROBE_RY = True
PROBE_RZ = True

# Z-axis: only allow + direction
Z_PLUS_ONLY = True

# ── Mode ──────────────────────────────────────────────────────────────────────
# "probe"  — test-step every ± direction, pick the one with lowest ||W||
# "direct" — read current F/T, move against the dominant component (faster)
MODE = "direct"

# Normalisation scales for comparing forces (N) vs moments (Nm).
# |F_i| / FORCE_SCALE  vs  |M_i| / MOMENT_SCALE  → unitless, comparable.
# Default uses the safety caps giving a 10:1 ratio (20 N ≈ 2 Nm).
FORCE_SCALE  = FORCE_SAFETY    # 20 N
MOMENT_SCALE = MOMENT_SAFETY   # 2 Nm

# Output file
OUTPUT_JSON = Path(__file__).resolve().parent / "find_center_3_log.json"

FORCE_LABELS = ["FX", "FY", "FZ", "MX", "MY", "MZ"]
MIN_MANIPULABILITY = MIN_MANIPULABILITY_AYAL

# ── Axis descriptors ──────────────────────────────────────────────────────────
# (name, move_index, probe_step, move_step, is_rotation)
AXES = [
    ("RX", 3, PROBE_ROT_STEP,  MOVE_ROT_STEP,  True),
    ("RY", 4, PROBE_ROT_STEP,  MOVE_ROT_STEP,  True),
    ("RZ", 5, PROBE_ROT_STEP,  MOVE_ROT_STEP,  True),
    ("X",  0, PROBE_STEP_SIZE, MOVE_STEP_SIZE, False),
    ("Y",  1, PROBE_STEP_SIZE, MOVE_STEP_SIZE, False),
    ("Z",  2, PROBE_STEP_SIZE, MOVE_STEP_SIZE, False),
]

_AXIS_ENABLED = {
    "X": PROBE_X, "Y": PROBE_Y, "Z": PROBE_Z,
    "RX": PROBE_RX, "RY": PROBE_RY, "RZ": PROBE_RZ,
}
AXES = [a for a in AXES if _AXIS_ENABLED.get(a[0], False)]

# Lookup: force/moment index → AXES entry  (axis_idx coincides with force idx)
_AXIS_BY_FORCE_IDX = {}
for _ae in AXES:
    _AXIS_BY_FORCE_IDX[_ae[1]] = _ae

def measure_noise(robot, n_samples=NOISE_SAMPLES):
    """
    Collect n_samples F/T readings while the robot is stationary.

    Returns
    -------
    noise_std : list[float]
        Per-component standard deviation [FX, FY, FZ, MX, MY, MZ].
    noise_floor : float
        ||std_F|| + ||std_M|| — the wrench magnitude of the noise.
        Changes in ||W|| smaller than this are indistinguishable from noise.
    """
    print(f"  Measuring sensor noise ({n_samples} samples) …")
    samples = []
    for i in range(n_samples):
        samples.append(forces_in_tcp_frame(robot))
        time.sleep(0.01)  # 10 ms
    arr = np.array(samples)  # (n_samples, 6)
    noise_std = list(arr.std(axis=0))
    noise_floor = (math.sqrt(noise_std[0]**2 + noise_std[1]**2 + noise_std[2]**2)
                   + math.sqrt(noise_std[3]**2 + noise_std[4]**2 + noise_std[5]**2))
    print(f"  Noise σ: {' '.join(f'{FORCE_LABELS[i]}={noise_std[i]:.4f}' for i in range(6))}")
    print(f"  Noise floor (||σ_F||+||σ_M||) = {noise_floor:.4f}")
    return noise_std, noise_floor

def probe_all_directions(robot, current_pose, wmag_here, noise_floor=0.0, axes_subset=None):
    """
    Test-step every enabled ± direction, measure ||W|| at each, return back.

    Candidates whose ||W|| improvement over wmag_here is smaller than
    noise_floor are marked as no real improvement.

    Returns a list of candidates sorted by wrench magnitude at the probed
    position (lowest first — i.e. closest to zero).
    """
    candidates = []

    active_axes = AXES if axes_subset is None else axes_subset

    for axis_name, axis_idx, probe_step, move_step, is_rot in active_axes:
        signs = [+1] if (axis_name == "Z" and Z_PLUS_ONLY) else [+1, -1]

        for sign in signs:
            dir_label = f"{'+'if sign > 0 else '-'}{axis_name}"
            test_pose = step_pose(current_pose, axis_idx, sign * probe_step, is_rot)

            _guarded_moveL(robot, test_pose, SPEED, ACCEL, False)
            time.sleep(SETTLE_TIME)

            forces_there = forces_in_tcp_frame(robot)
            check_safety(forces_there, label=f"probing {dir_label}")
            wmag_there = wrench_magnitude(forces_there)

            candidates.append({
                "axis_name":  axis_name,
                "axis_idx":   axis_idx,
                "is_rot":     is_rot,
                "sign":       sign,
                "move_step":  move_step,
                "test_pose":  test_pose,
                "dir_label":  dir_label,
                "wmag_there": wmag_there,
                "forces":     forces_there,
            })

            print(f"    {dir_label:>4}  ||W||={wmag_there:.4f}  ({fmt_forces(forces_there)})")

            # Return to current pose
            _guarded_moveL(robot, current_pose, SPEED, ACCEL, False)
            time.sleep(SETTLE_TIME)

    # Sort by wrench magnitude — lowest (closest to 0) first
    candidates.sort(key=lambda c: c["wmag_there"])
    return candidates

def pick_dominant_axis(forces, noise_std):
    """
    Find the F/T component with the highest normalised magnitude above noise.

    Returns (force_idx, axis_entry, sign, score) or None if everything is
    within the noise floor.  *sign* is the direction to step: opposite to
    the measured force/moment.
    """
    def _best_from_indices(indices):
        best_score = 0.0
        best_local = None

        for force_idx in indices:
            if force_idx not in _AXIS_BY_FORCE_IDX:
                continue  # axis disabled

            value = forces[force_idx]
            # Only consider signal above the per-component noise σ
            signal = abs(value) - abs(noise_std[force_idx])
            if signal <= 0:
                continue

            scale = FORCE_SCALE if force_idx < 3 else MOMENT_SCALE
            score = signal / scale

            axis_entry = _AXIS_BY_FORCE_IDX[force_idx]
            axis_name = axis_entry[0]

            # Move with the force: step in +sign(value)
            sign = +1 if value > 0 else -1

            # Respect Z_PLUS_ONLY
            if axis_name == "Z" and Z_PLUS_ONLY and sign < 0:
                continue

            if score > best_score:
                best_score = score
                best_local = (force_idx, axis_entry, sign, score)

        return best_local

    # Rotation-first policy: reduce moments before translations.
    best_rot = _best_from_indices([3, 4, 5])
    if best_rot is not None:
        return best_rot
    return _best_from_indices([0, 1, 2])

def run_find_center_by_grad(robot, zero_ft=False):
    """
    Iteratively move toward zero wrench.

    At each step: probe all ± directions, pick the one with the smallest
    ||W||, move there.  Stop when the best probed ||W|| is not lower than
    the current ||W|| by more than the noise floor (no real improvement).
    """
    if zero_ft:
        print("Zeroing F/T sensor …")
        robot.control.zeroFtSensor()
        time.sleep(SETTLE_TIME)

    # Measure sensor noise while stationary
    noise_std, noise_floor = measure_noise(robot)

    initial_pose = list(robot.recieve.getActualTCPPose())
    forces_start = forces_in_tcp_frame(robot)
    check_safety(forces_start, label="at start")
    wmag_start = wrench_magnitude(forces_start)

    print(f"  Start pose:   {fmt_pose(initial_pose)}")
    print(f"  Start forces: {fmt_forces(forces_start)}")
    print(f"  Start ||W|| = {wmag_start:.4f}")

    current_pose = list(initial_pose)
    wmag_current = wmag_start
    step_log = []
    total_steps = 0
    rotation_phase = True

    for step_num in range(1, MAX_STEPS + 1):
        active_axes = [a for a in AXES if a[4]] if rotation_phase else [a for a in AXES if not a[4]]
        phase_name = "rotation" if rotation_phase else "translation"

        print(f"\n{'─' * 60}")
        print(f"  STEP {step_num} / {MAX_STEPS}   ||W|| = {wmag_current:.4f}   phase={phase_name}")
        print(f"{'─' * 60}")

        # Probe every direction
        candidates = probe_all_directions(robot, current_pose, wmag_current, noise_floor, active_axes)

        if not candidates:
            if rotation_phase:
                print("  ℹ  No rotational candidate directions. Switching to translation phase.")
                rotation_phase = False
                continue
            print("  ⚠  No candidate directions.")
            break

        best = candidates[0]

        # Can we improve beyond noise?
        improvement = wmag_current - best["wmag_there"]
        if improvement <= noise_floor:
            if rotation_phase:
                print(f"\n  ✅  Rotational improvement is within noise floor ({noise_floor:.4f}).")
                print(f"      Best rotational probe: {best['dir_label']} → ||W||={best['wmag_there']:.4f}  "
                    f"(improvement {improvement:.4f} ≤ noise {noise_floor:.4f})")
                print("      Switching to translation phase.")
                rotation_phase = False
                continue
            print(f"\n  ✅  No translation direction reduces ||W|| beyond noise floor ({noise_floor:.4f})")
            print(f"      Current ||W|| = {wmag_current:.4f}")
            print(f"      Best probe: {best['dir_label']} → ||W||={best['wmag_there']:.4f}  "
                f"(improvement {improvement:.4f} ≤ noise {noise_floor:.4f})")
            print(f"      Centered.")
            break

            # Move to the best probed pose (optimal candidate)
            print(f"\n  ★  Best: {best['dir_label']}  ||W||={best['wmag_there']:.4f}  "
                  f"(Δ = {best['wmag_there'] - wmag_current:+.4f})")

            _guarded_moveL(robot, best["test_pose"], SPEED, ACCEL, False)
            time.sleep(SETTLE_TIME)

        forces_after = forces_in_tcp_frame(robot)
        check_safety(forces_after, label=f"after step {step_num}")
        wmag_after = wrench_magnitude(forces_after)
        actual_pose = list(robot.recieve.getActualTCPPose())

        print(f"  After step: ||W|| = {wmag_after:.4f}  "
              f"(Δ = {wmag_after - wmag_current:+.4f})")
        print(f"  Forces: {fmt_forces(forces_after)}")
        print(f"  Pose:   {fmt_pose(actual_pose)}")

        step_log.append({
            "step":           step_num,
            "direction":      best["dir_label"],
            "wmag_before":    wmag_current,
            "wmag_after":     wmag_after,
            "forces_before":  forces_in_tcp_frame(robot) if False else None,  # skip re-read
            "forces_after":   list(forces_after),
            "pose_before":    current_pose,
            "pose_after":     actual_pose,
        })

        current_pose = actual_pose
        wmag_current = wmag_after
        total_steps = step_num
    else:
        print(f"\n  ⚠  MAX_STEPS ({MAX_STEPS}) reached")
        total_steps = MAX_STEPS

    final_pose = list(robot.recieve.getActualTCPPose())
    forces_final = forces_in_tcp_frame(robot)
    wmag_final = wrench_magnitude(forces_final)

    print(f"\n  Final pose:   {fmt_pose(final_pose)}")
    print(f"  Final forces: {fmt_forces(forces_final)}")
    print(f"  Final ||W|| = {wmag_final:.4f}  (started at {wmag_start:.4f})")

    return {
        "start_pose":      initial_pose,
        "final_pose":      final_pose,
        "total_steps":     total_steps,
        "wmag_start":      wmag_start,
        "wmag_final":      wmag_final,
        "forces_start":    list(forces_start),
        "forces_final":    list(forces_final),
        "noise_std":       noise_std,
        "noise_floor":     noise_floor,
        "step_log":        step_log,
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
        result = run_find_center_by_value(ayal)

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