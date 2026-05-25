#!/usr/bin/env python3
"""
Find the center point by minimising all forces/moments toward zero.

Assumes that zero wrench (F=0, M=0) corresponds to the centered pose.
At each step the script probes every enabled ± direction with a small
test-step, measures the total wrench magnitude ||W||, and moves in the
direction that brings ||W|| closest to zero.  Stops when no direction
can reduce ||W|| any further.
"""

import sys, os
import json
import math
import time
from pathlib import Path
from datetime import datetime

import numpy as np
_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _RMP_LAB_ROOT in sys.path:
    sys.path.remove(_RMP_LAB_ROOT)
sys.path.insert(0, _RMP_LAB_ROOT)
import uri_if

# ── Tee stdout/stderr to a log file ───────────────────────────────────────────
_LOG_FILE = Path(__file__).resolve().parent / "find_center_3.log"

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
    _log_fh = open(_LOG_FILE, "w")
    sys.stdout = _Tee(sys.__stdout__, _log_fh)
    sys.stderr = _Tee(sys.__stderr__, _log_fh)

# ── Path setup ─────────────────────────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "calibration"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "uris_gui"))

import gemini_calc_v2 as gemini
from config import DEFAULT_SPEED, DEFAULT_ACCELERATION
from uri_calibration.src.check_pose import _manipulability, MIN_MANIPULABILITY_AYAL

# ── Configuration ──────────────────────────────────────────────────────────────
AYAL_HOST = "192.168.57.101"

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

# ── Helpers ────────────────────────────────────────────────────────────────────
def fmt_pose(pose):
    return "[" + ", ".join(f"{v: .6f}" for v in pose) + "]"

def fmt_forces(forces):
    return "  ".join(f"{FORCE_LABELS[i]}={forces[i]:+.3f}" for i in range(6))

def forces_in_tcp_frame(robot):
    """Read F/T sensor and rotate wrench into the TCP frame."""
    forces_base = np.array(robot.recieve.getActualTCPForce(), dtype=float)
    pose = robot.recieve.getActualTCPPose()
    R = gemini.pose_to_T(pose)[:3, :3]
    R_inv = R.T
    f_tcp = R_inv @ forces_base[:3]
    m_tcp = R_inv @ forces_base[3:]
    return list(np.concatenate([f_tcp, m_tcp]))

def wrench_magnitude(forces):
    """||F|| + ||M|| — scalar measure of how far from zero the wrench is."""
    f_mag = math.sqrt(forces[0]**2 + forces[1]**2 + forces[2]**2)
    m_mag = math.sqrt(forces[3]**2 + forces[4]**2 + forces[5]**2)
    return f_mag + m_mag

def step_pose(pose, axis_idx, delta, is_rotation):
    """Shift pose by delta along axis_idx in the TCP frame."""
    T_current = gemini.pose_to_T(pose)
    T_step = np.eye(4)
    if not is_rotation:
        T_step[axis_idx, 3] = delta
    else:
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

# ── Noise measurement ─────────────────────────────────────────────────────────
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

# ── Core: probe all directions, pick the one closest to zero ──────────────────
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

# ── Direct mode: oppose the dominant F/T component ────────────────────────────
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

# ── Probe mode descent loop ──────────────────────────────────────────────────
def run_find_center(robot, zero_ft=False):
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
    print("  Find Center v3 — minimise wrench toward zero")
    print("=" * 70)
    print(f"  Host              : {AYAL_HOST}")
    print(f"  Probe step (XYZ)  : {PROBE_STEP_SIZE} m")
    print(f"  Probe step (rot)  : {PROBE_ROT_STEP} rad")
    print(f"  Move step  (XYZ)  : {MOVE_STEP_SIZE} m")
    print(f"  Move step  (rot)  : {MOVE_ROT_STEP} rad")
    print(f"  Force safety      : {FORCE_SAFETY} N")
    print(f"  Moment safety     : {MOMENT_SAFETY} Nm")
    print(f"  Max steps         : {MAX_STEPS}")
    print(f"  Speed / Accel     : {SPEED} / {ACCEL}")
    print(f"  Settle time       : {SETTLE_TIME} s")
    print(f"  Z plus only       : {Z_PLUS_ONLY}")
    print(f"  Mode              : {MODE}")
    if MODE == "direct":
        print(f"  Force scale       : {FORCE_SCALE} N")
        print(f"  Moment scale      : {MOMENT_SCALE} Nm")
        print(f"  Ratio             : {FORCE_SCALE / MOMENT_SCALE:.1f}:1  "
              f"(1 Nm ≡ {FORCE_SCALE / MOMENT_SCALE:.1f} N)")
    print()

    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    try:
        ayal.connect(False)
    except Exception as e:
        print(f"ERROR connecting to Ayal ({AYAL_HOST}): {e}")
        sys.exit(1)

    print(f"Connected to Ayal at {AYAL_HOST}")

    try:
        if MODE == "direct":
            result = run_find_center_direct(ayal)
        else:
            result = run_find_center(ayal)

        # ── Summary ───────────────────────────────────────────────────────
        print(f"\n{'=' * 70}")
        print(f"  SUMMARY")
        print(f"{'=' * 70}")
        print(f"  Total steps:  {result['total_steps']}")
        print(f"  ||W|| start:  {result['wmag_start']:.4f}")
        print(f"  ||W|| final:  {result['wmag_final']:.4f}")
        print(f"  Noise floor:  {result['noise_floor']:.4f}")
        # print(f"  Noise σ:      {' '.join(f'{FORCE_LABELS[i]}={result["noise_std"][i]:.4f}' for i in range(6))}")
        print(f"  Start pose:   {fmt_pose(result['start_pose'])}")
        print(f"  Final pose:   {fmt_pose(result['final_pose'])}")

        if result["step_log"]:
            print(f"\n  {'Step':>4}  {'Dir':>4}  {'||W|| before':>12}  {'||W|| after':>12}  {'Δ':>8}")
            print(f"  {'─' * 50}")
            for entry in result["step_log"]:
                delta = entry["wmag_after"] - entry["wmag_before"]
                print(f"  {entry['step']:>4}  {entry['direction']:>4}  "
                      f"{entry['wmag_before']:>12.4f}  "
                      f"{entry['wmag_after']:>12.4f}  "
                      f"{delta:>+8.4f}")
        print(f"{'=' * 70}")

        # ── Save to JSON ──────────────────────────────────────────────────
        log_entry = {
            "timestamp":    datetime.now().isoformat(),
            "mode":         MODE,
            "start_pose":   result["start_pose"],
            "final_pose":   result["final_pose"],
            "total_steps":  result["total_steps"],
            "wmag_start":   result["wmag_start"],
            "wmag_final":   result["wmag_final"],
            "forces_start": result["forces_start"],
            "forces_final": result["forces_final"],
            "noise_std":    result["noise_std"],
            "noise_floor":  result["noise_floor"],
            "config": {
                "probe_step_size": PROBE_STEP_SIZE,
                "probe_rot_step":  PROBE_ROT_STEP,
                "move_step_size":  MOVE_STEP_SIZE,
                "move_rot_step":   MOVE_ROT_STEP,
                "force_safety":    FORCE_SAFETY,
                "moment_safety":   MOMENT_SAFETY,
                "max_steps":       MAX_STEPS,
                "speed":           SPEED,
                "accel":           ACCEL,
                "settle_time":     SETTLE_TIME,
                "z_plus_only":     Z_PLUS_ONLY,
            },
            "step_log": result["step_log"],
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
