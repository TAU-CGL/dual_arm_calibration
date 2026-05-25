"""
Peg-in controller / calibration refiner for Ayal.

Approach phase (LQR-driven):
    Drives TCP in its own Z+ direction while using the learned 3x15 LQR gain
    to correct rotation (rx, ry, rz) from dual-arm force feedback. Breaks out
    when ayal is within `--approach-stop` (default 2 mm) of the mirror-mate
    pose computed from URI's actual pose + the current base_to_base.

    Input vector x (15 dims):
        [ayal_wrench_tcp(6), uri_wrench_tcp(6), rel_distance(1),
         ayal_manipulability(1), uri_manipulability(1)]
    Output (3 dims): rotation delta of ayal pose toward the mate.
    Translation step is the constant Z+ stride; the model only steers orientation.

Slow-seek phase (no LQR):
    After approach + user confirmation, steps 0.1 mm at a time along ayal's
    TCP +Z, monitoring change in TCP-frame F_z on BOTH robots. Stops on the
    first reading where max(|dF_z_ayal|, |dF_z_uri|) >= `--slow-force-thresh`
    (default 0.5 N). User then chooses [f]inish or [r]etry.
    Finish recomputes base_to_base via utils.calculate_ayal_in_uri and
    APPENDS a new sample to uris_gui/calibration_samples.json (the same file
    panel_calibrate writes to, consumed by utils.calculate_optimal_calibration).
    Existing calibration.json is left untouched. Retry puts both arms in
    freedrive and starts again from the mate computation.

Usage:
    python peg_in.py [--steps N] [--step-size M] [--force-limit F]
                     [--approach-stop M] [--slow-step M]
                     [--slow-force-thresh N] [--slow-max-steps K]
"""

import os
import sys
import time
import math
import json
import argparse
import threading
import torch
from uri_calibration.src import jitter
from uri_calibration.src import utils
import uri_if
import numpy as np
import roboticstoolbox as rtb

# --- config ---
K_PATH          = os.path.join(os.path.dirname(__file__), "../logs/lqr_gain_K.pt")
CALIBRATION_SAMPLES_FILE  = os.path.join(uri_if._RMP_LAB_ROOT, "shared", "calibration_samples.json")

# Fallback if calibration.json is missing — matches auto_peg_cycle.CALIBRATION.
DEFAULT_BASE_TO_BASE = [0.509576, -0.952662, 0.270758, -0.001881, -0.006325, 0.016716]

Z_STEP          = 0.0005     # metres per iteration in TCP Z+
SPEED           = 0.01      # moveL speed  [m/s]
ACCEL           = 0.05      # moveL accel  [m/s²]
MAX_STEPS       = 50
FORCE_LIMIT     = 40.0      # N  — stop if any force component exceeds this
LQR_SCALE       = 0.2       # scales LQR correction relative to forward step (0=off, 1=full)
LQR_FORCE_THRESHOLD = 5.0   # N  — below this total force magnitude, skip LQR correction
N_SAMPLES       = 30        # readings averaged per force sample
SAMPLE_DELAY_S  = 0.002
SETTLE_S        = 0.3       # wait after each moveL

# When True, apply the full 6-dim LQR output (translation + rotation) to the
# pose. When False, only apply the rotation half (indices 3:6 of K's output)
# to the rotation portion of the pose.
APPLY_FULL_POSE_CORR = True

# safety clamps on the LQR correction (per axis)
MAX_ROT_CORR    = 0.05      # rad
MAX_TRANS_CORR  = 0.002     # m

# Calibration-flow defaults
APPROACH_STOP_DIST  = 0.005   # m  — break out of approach when |ayal_xyz - mate_xyz| <= this
SLOW_STEP_SIZE      = 0.0001  # m  — 0.1 mm per slow-seek step
SLOW_FORCE_THRESH   = 5.0     # N  — slight-but-not-too-slight Z-force change for contact detect
SLOW_MAX_STEPS      = 100     # safety cap on slow seek (~ 10 mm at 0.1 mm/step)
SLOW_SPEED          = 0.005   # m/s — moveL speed during slow seek
SLOW_ACCEL          = 0.02    # m/s² — moveL accel during slow seek
# --------------

_UR5_MODEL = rtb.models.UR5()

def manipulability(q: list[float]) -> float:
    J = _UR5_MODEL.jacob0(q)
    return float(math.sqrt(max(0.0, float(np.linalg.det(J @ J.T)))))

def avg_wrench(robot) -> list[float]:
    samples = []
    for i in range(N_SAMPLES):
        if i:
            time.sleep(SAMPLE_DELAY_S)
        samples.append(list(robot.recieve.getActualTCPForce())[:6])
    return [sum(s[j] for s in samples) / N_SAMPLES for j in range(6)]

def wrench_to_tcp(wrench_base: list[float], pose: list[float]) -> list[float]:
    return list(utils.wrench_trans(wrench_base, pose, base_to_tcp=True, include_translation=False))

def tcp_z_in_base(pose: list[float]) -> np.ndarray:
    R = utils.rotvec_to_R(pose[3:6])
    return np.array([R[0, 2], R[1, 2], R[2, 2]])

def ayal_pose_in_uri_base(ayal_pose: list[float], base_to_base: list[float]) -> list[float]:
    T_uri_ayalbase = utils.pose_to_T(base_to_base)
    T_ayalbase_ayaltcp = utils.pose_to_T(ayal_pose)
    return list(utils.T_to_pose(T_uri_ayalbase @ T_ayalbase_ayaltcp))

def clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))
\
def append_calibration_sample(uri_pose: list[float], ayal_pose: list[float], derived_b2b: list[float]) -> int:
    """Append a new sample to calibration_samples.json. Returns total sample count.

    Schema matches what panel_calibrate writes / utils.calculate_optimal_calibration
    consumes (only `P_uri_when_calib` / `P_ayal_when_calib` are read by the optimiser);
    extra fields are for human inspection and are ignored by it."""
    samples: list[dict] = []
    if os.path.exists(CALIBRATION_SAMPLES_FILE):
        try:
            with open(CALIBRATION_SAMPLES_FILE) as f:
                loaded = json.load(f)
            if isinstance(loaded, list):
                samples = loaded
            else:
                print(f"calibration_samples.json has unexpected shape ({type(loaded).__name__}); starting fresh")
        except json.JSONDecodeError as e:
            print(f"calibration_samples.json failed to parse ({e}); starting fresh")

    sample = {
        "P_uri_when_calib": [float(v) for v in uri_pose],
        "P_ayal_when_calib": [float(v) for v in ayal_pose],
        "derived_base_to_base": [float(v) for v in derived_b2b],
        "source": "peg_in.py",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    samples.append(sample)

    with open(CALIBRATION_SAMPLES_FILE, "w") as f:
        json.dump(samples, f, indent=2)
    print(f"appended sample #{len(samples)} to {CALIBRATION_SAMPLES_FILE}")
    return len(samples)

def build_input(
    ayal_wrench_tcp: list[float],
    uri_wrench_tcp: list[float],
    ayal_pose: list[float],
    uri_pose: list[float],
    ayal_q: list[float],
    uri_q: list[float],
    base_to_base: list[float],
) -> torch.Tensor:
    ayal_in_uri = ayal_pose_in_uri_base(ayal_pose, base_to_base)
    rel_distance = float(np.linalg.norm([ayal_in_uri[j] - uri_pose[j] for j in range(3)]))
    ayal_m = manipulability(ayal_q)
    uri_m = manipulability(uri_q)
    return torch.tensor(
        list(ayal_wrench_tcp) + list(uri_wrench_tcp) + [rel_distance, ayal_m, uri_m],
        dtype=torch.float32,
    )

def apply_lqr_correction(pose: list[float], K_insert: torch.Tensor, x: torch.Tensor) -> list[float]:
    corr = (LQR_SCALE * K_insert @ x).tolist()  # 6 values: [trans(3), rot(3)]
    out = list(pose)
    if APPLY_FULL_POSE_CORR:
        for i in range(3):
            out[i]     = pose[i]     - clamp(corr[i],     MAX_TRANS_CORR)
            out[3 + i] = pose[3 + i] - clamp(corr[3 + i], MAX_ROT_CORR)
    else:
        for i in range(3):
            out[3 + i] = pose[3 + i] - clamp(corr[3 + i], MAX_ROT_CORR)
    return out

def freedrive_both(ayal, uri, prompt: str) -> None:
    ayal.control.teachMode()
    uri.control.teachMode()
    input(prompt)
    ayal.control.endTeachMode()
    uri.control.endTeachMode()
    time.sleep(SETTLE_S)

def read_ft_in_tcp(robot) -> tuple[list[float], list[float]]:
    pose = list(robot.recieve.getActualTCPPose())
    wrench_tcp = wrench_to_tcp(avg_wrench(robot), pose)
    return pose, wrench_tcp

def _start_jitter_thread(uri, uri_base_pose: list[float]) -> tuple[threading.Thread, threading.Event]:
    stop_event = threading.Event()
    thread = threading.Thread(
        target=jitter.jitter,
        args=(uri, uri_base_pose, 0.01, stop_event),
        daemon=True,
    )
    thread.start()
    return thread, stop_event

def _stop_jitter_thread(thread: threading.Thread, stop_event: threading.Event) -> None:
    stop_event.set()
    thread.join(timeout=2.0)

def approach_to_near_mate(
    ayal, uri,
    K_insert: torch.Tensor,
    base_to_base: list[float],
    args,
) -> bool:
    """Existing LQR-driven Z+ stepping with an early-exit when |ayal_xyz - mate_xyz|
    drops below args.approach_stop. mate_pose is recomputed each iteration from
    URI's actual pose so it tracks any URI drift. Returns True if early-exit hit."""

    uri_base_pose = list(uri.recieve.getActualTCPPose())
    jitter_thread, stop_jitter = _start_jitter_thread(uri, uri_base_pose)

    try:
        for step in range(args.steps):
            ayal_pose = list(ayal.recieve.getActualTCPPose())
            uri_pose  = list(uri.recieve.getActualTCPPose())
            ayal_q    = list(ayal.recieve.getActualQ())
            uri_q     = list(uri.recieve.getActualQ())

            mate_pose = list(utils.calculate_mirror_position(uri_pose, base_to_base, flip_trans=True))
            dist_to_mate = math.sqrt(sum((mate_pose[j] - ayal_pose[j]) ** 2 for j in range(3)))
            if dist_to_mate <= args.approach_stop:
                print(f"approach: within {dist_to_mate*1000:.2f} mm of mate — stopping.")
                return True

            ayal_wrench_base = avg_wrench(ayal)
            ayal_wrench_tcp  = wrench_to_tcp(ayal_wrench_base, ayal_pose)
            uri_wrench_base  = avg_wrench(uri)
            uri_wrench_tcp   = wrench_to_tcp(uri_wrench_base, uri_pose)

            if max(abs(f) for f in ayal_wrench_tcp[:3]) > args.force_limit:
                print(f"step {step}: force limit reached — pausing uri jitter, trying spiral search.")
                _stop_jitter_thread(jitter_thread, stop_jitter)
                time.sleep(SETTLE_S)
                try:
                    spiral_ok = spiral_orientation_search(ayal, args)
                finally:
                    jitter_thread, stop_jitter = _start_jitter_thread(uri, uri_base_pose)
                if spiral_ok:
                    print(f"step {step}: spiral found low-force pose — resuming approach.")
                else:
                    print(f"step {step}: spiral failed — stepping back.")
                    z_base = tcp_z_in_base(ayal_pose)
                    back = list(ayal_pose)
                    back[0] -= args.step_size * z_base[0]
                    back[1] -= args.step_size * z_base[1]
                    back[2] -= args.step_size * z_base[2]
                    ayal.control.moveL(back, SPEED, ACCEL, False)
                    time.sleep(SETTLE_S)
                continue

            z_base = tcp_z_in_base(ayal_pose)
            target = list(ayal_pose)
            target[0] += args.step_size * z_base[0]
            target[1] += args.step_size * z_base[1]
            target[2] += args.step_size * z_base[2]

            force_magnitude = (sum(f ** 2 for f in ayal_wrench_tcp[:3])) ** 0.5
            if force_magnitude >= LQR_FORCE_THRESHOLD:
                # x = build_input(
                #     ayal_wrench_tcp, uri_wrench_tcp,
                #     ayal_pose, uri_pose,
                #     ayal_q, uri_q,
                #     base_to_base,
                # )
                x = torch.tensor(ayal_wrench_tcp, dtype=torch.float32)
                target = apply_lqr_correction(target, K_insert, x)

            print(
                f"step {step:3d} | dist={dist_to_mate*1000:6.2f} mm | "
                f"ayal F/T: [{', '.join(f'{v:7.3f}' for v in ayal_wrench_tcp)}]"
            )
            ayal.control.moveL(target, SPEED, ACCEL, False)
            time.sleep(SETTLE_S)

        print("approach: step cap hit without reaching the 2 mm threshold.")
        return False
    # try:
    #     task_frame = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     selection_vector = [1, 1, 1, 1, 1, 1]
    #     wrench = [0.0, 0.0, 7.0, 0.0, 0.0, 0.0]
    #     force_type = 1
    #     limits = [0.1, 0.1, 0.15, 0.1, 0.1, 0.1]     
    #     ayal.control.forceMode(task_frame, selection_vector, wrench, force_type, limits)
    #     input(f"press to end")
    finally:
        ayal.control.forceModeStop()
        _stop_jitter_thread(jitter_thread, stop_jitter)


def slow_force_seek(ayal, uri, args) -> bool:
    """Step ayal 0.1 mm at a time along its TCP +Z. Stop on first reading where
    max(|dF_z_ayal|, |dF_z_uri|) >= args.slow_force_thresh, using the wrench at
    entry as the baseline. Returns True if contact was detected."""
    _, ayal_w0 = read_ft_in_tcp(ayal)
    _, uri_w0 = read_ft_in_tcp(uri)
    F_ayal0 = ayal_w0[2]
    F_uri0 = uri_w0[2]
    print(f"slow-seek baseline F_z: ayal={F_ayal0:+.3f} N | uri={F_uri0:+.3f} N "
          f"(threshold dF >= {args.slow_force_thresh:.3f} N)")

    for k in range(args.slow_max_steps):
        ayal_pose = list(ayal.recieve.getActualTCPPose())
        z_base = tcp_z_in_base(ayal_pose)
        target = list(ayal_pose)
        for axis in range(3):
            target[axis] += args.slow_step * z_base[axis]
        ayal.control.moveL(target, SLOW_SPEED, SLOW_ACCEL, False)
        time.sleep(SETTLE_S)

        _, ayal_w = read_ft_in_tcp(ayal)
        _, uri_w = read_ft_in_tcp(uri)
        dF_ayal = ayal_w[2] - F_ayal0
        dF_uri = uri_w[2] - F_uri0
        print(
            f"slow {k:3d} | ayal F_z={ayal_w[2]:+.3f} (dF={dF_ayal:+.3f}) | "
            f"uri F_z={uri_w[2]:+.3f} (dF={dF_uri:+.3f})"
        )
        if max(abs(dF_ayal), abs(dF_uri)) >= args.slow_force_thresh:
            print(f"  contact detected after {(k + 1) * args.slow_step * 1000:.2f} mm of slow approach.")
            return True

    print("slow-seek: cap hit without detecting contact.")
    return False

def retract_to_start(ayal, uri, ayal_start_pose: list[float]) -> None:
    """Put uri in 6-DOF compliance (zero wrench) so it yields if grippers are
    still engaged, moveJ_IK ayal back to its starting pose, then drop uri out
    of compliance and re-zero both FT sensors."""
    task_frame       = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    selection_vector = [1, 1, 1, 1, 1, 1]
    wrench           = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    force_type       = 2
    limits           = [0.05, 0.05, 0.05, 0.2, 0.2, 0.2]
    print("retract: uri -> compliance mode")
    uri.control.forceMode(task_frame, selection_vector, wrench, force_type, limits)
    try:
        print(f"retract: moveJ_IK ayal back to start pose")
        ayal.control.moveJ_IK(ayal_start_pose, SPEED, ACCEL, False)
        time.sleep(SETTLE_S)
    finally:
        if hasattr(uri.control, "forceModeStop"):
            uri.control.forceModeStop()
    ayal.control.zeroFtSensor()
    uri.control.zeroFtSensor()
    time.sleep(SETTLE_S)
    print("retract: done — ready to retry approach")

def spiral_orientation_search(
    ayal,
    args,
    max_amp_rad: float = 0.05,
    max_steps: int = 24,
    low_force_thresh: float = 10.0,
) -> bool:
    """Hold ayal TCP xyz fixed and spiral the tool orientation about the
    current TCP-Z, with amplitude growing linearly from 0 to `max_amp_rad`.

    After each perturbation step, read ayal F/T:
      - if any |F_xyz| exceeds args.force_limit  → retract to start pose and
        try the opposite spiral direction.
      - if |F_xyz| drops below low_force_thresh → return True (likely aligned).

    Returns False if all directions are exhausted without finding a low-force
    pose. On exit, ayal is always returned to the start pose."""
    base_pose = list(ayal.recieve.getActualTCPPose())
    R_base = utils.rotvec_to_R(base_pose[3:6])

    _, ayal_w = read_ft_in_tcp(ayal)
    f_mag_init = math.sqrt(sum(f * f for f in ayal_w[:3]))

    for direction in (+1, -1):
        print(f"spiral: direction {direction:+d}")
        hit_limit = False
        for k in range(1, max_steps + 1):
            t = k / max_steps
            theta = direction * 2.0 * math.pi * t   # one full turn over the sweep
            amp = max_amp_rad * t
            pert_rotvec = [amp * math.cos(theta), amp * math.sin(theta), 0.0]
            R_target = R_base @ utils.rotvec_to_R(pert_rotvec)
            target = list(base_pose[:3]) + list(utils.R_to_rotvec(R_target))

            ayal.control.moveL(target, SLOW_SPEED, SLOW_ACCEL, False)
            time.sleep(SETTLE_S)

            _, ayal_w = read_ft_in_tcp(ayal)
            f_mag = math.sqrt(sum(f * f for f in ayal_w[:3]))
            print(
                f"  spiral {direction:+d} {k:2d}/{max_steps} | amp={amp:.3f} θ={theta:+.2f} | "
                f"|F|={f_mag:5.2f} | F=[{', '.join(f'{v:+5.2f}' for v in ayal_w[:3])}]"
            )

            if max(abs(f) for f in ayal_w[:3]) > args.force_limit:
                print("  force limit exceeded — retracting and trying other direction")
                hit_limit = True
                break
            if f_mag_init-f_mag < low_force_thresh:
                print(f"  low-force pose found at step {k} — staying here")
                return True

        ayal.control.moveL(base_pose, SLOW_SPEED, SLOW_ACCEL, False)
        time.sleep(SETTLE_S)
        if not hit_limit:
            print("  spiral sweep finished without finding a low-force pose")

    return False

def print_paired_forces(ayal, uri) -> None:
    _, ayal_w = read_ft_in_tcp(ayal)
    _, uri_w = read_ft_in_tcp(uri)
    print(f"  ayal F/T (tcp): [{', '.join(f'{v:7.3f}' for v in ayal_w)}]")
    print(f"  uri  F/T (tcp): [{', '.join(f'{v:7.3f}' for v in uri_w)}]")

def connect(uri, ayal, args):
    K = torch.load(K_PATH, weights_only=True)
    if tuple(K.shape) != (6,6): #(3, 15):
        raise ValueError(f"expected K shape (3,15), got {tuple(K.shape)}")
    K_insert = -K
    print(f"Loaded K from {K_PATH} (shape {tuple(K.shape)})")

    base_to_base = utils.load_calibration()
    utils.connect_robots()

    try:
        prompt = "Both robots in freedrive — position them, then press Enter to start..."
        skip_freedrive = False
        while True:
            if not skip_freedrive:
                freedrive_both(ayal, uri, prompt)
            skip_freedrive = False

            ayal.control.zeroFtSensor()
            uri.control.zeroFtSensor()
            time.sleep(SETTLE_S)

            ayal_start_pose = list(ayal.recieve.getActualTCPPose())

            approached = approach_to_near_mate(ayal, uri, K_insert, base_to_base, args)
            if not approached:
                ans = input("approach didn't reach the 2 mm target. [c]ontinue / [r]etry / re[t]ract+retry / [s]piral / [q]uit: ").strip().lower()
                if ans in ("q", "quit"):
                    return
                if ans in ("r", "retry"):
                    prompt = "freedrive both arms — reposition, then press Enter..."
                    continue
                if ans in ("t", "retract"):
                    retract_to_start(ayal, uri, ayal_start_pose)
                    skip_freedrive = True
                    continue
                if ans in ("s", "spiral"):
                    spiral_orientation_search(ayal, args)
                    skip_freedrive = True
                    continue

            print(f"forces at ~{args.approach_stop*1000:.1f} mm before mate:")
            print_paired_forces(ayal, uri)
            input("press Enter to begin slow contact seek...")

            slow_force_seek(ayal, uri, args)

            print("forces at end of slow seek:")
            print_paired_forces(ayal, uri)

            while True:
                choice = input("[f]inish (record calibration) / [s]piral / [r]etry: ").strip().lower()
                if choice in ("f", "finish"):
                    uri_pose = list(uri.recieve.getActualTCPPose())
                    ayal_pose = list(ayal.recieve.getActualTCPPose())
                    new_b2b = list(utils.calculate_ayal_in_uri(uri_pose, ayal_pose))
                    append_calibration_sample(uri_pose, ayal_pose, new_b2b)
                    print(f"derived base_to_base: {new_b2b}")
                    return
                if choice in ("s", "spiral"):
                    spiral_orientation_search(ayal, args)
                    print("forces after spiral:")
                    print_paired_forces(ayal, uri)
                    continue
                if choice in ("r", "retry"):
                    prompt = "freedrive both arms — reposition, then press Enter..."
                    break
                print("unknown choice")

    finally:
        ayal.disconnect()
        uri.disconnect()

def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps",         type=int,   default=MAX_STEPS)
    parser.add_argument("--step-size",     type=float, default=Z_STEP)
    parser.add_argument("--force-limit",   type=float, default=FORCE_LIMIT)
    parser.add_argument("--approach-stop", type=float, default=APPROACH_STOP_DIST,
                        help="break out of approach when |ayal_xyz - mate_xyz| <= this (m)")
    parser.add_argument("--slow-step",     type=float, default=SLOW_STEP_SIZE,
                        help="step size during slow contact seek (m)")
    parser.add_argument("--slow-force-thresh", type=float, default=SLOW_FORCE_THRESH,
                        help="|dF_z| change (N) on either robot that ends the slow seek")
    parser.add_argument("--slow-max-steps", type=int, default=SLOW_MAX_STEPS)
    return parser.parse_args()

def main():
    args = parse_arguments()
    uri, ayal =utils.connect_robots()
    connect(uri, ayal, args)

if __name__ == "__main__":
    main()
