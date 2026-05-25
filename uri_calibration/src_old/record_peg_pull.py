"""
this script would record the force and pose of ayal while connected to uri.
Uri would stay in place.
Start position is when the two are connected with the rectangular connector, and aligned such that the connector is free to move.
The connector used has 0.2 'Shpill' in each direction.
End position is when they are unconnected, facing each other.
Throughout the process, I will move Ayal to step by step, and record the pose and force at each step.
The puprose is to create a table of Delta_pose vs force, to be used for learning the pegging policy.
"""
#region imports
import os
import sys
import json
import math
import random
import time
from pathlib import Path
from types import SimpleNamespace
import uri_if
from uri_calibration.src import utils as gemini
from uri_calibration.src.is_valid_pose import check_pose, _manipulability, MIN_MANIPULABILITY_URI, MIN_MANIPULABILITY_AYAL
import math as _math

# --- config ---
AYAL_HOST        = "192.168.57.101"
URI_HOST         = "192.168.56.101"

CALIBRATION_FILE = Path(__file__).resolve().parent / "calibration.json"

# Ayal safe/home joint config (degrees → radians) — far from Uri's workspace
# endregion

AYAL_BASE_JOINT = [-0.3936, -83.6455, 145.7909, -222.5215, 0.4769, 0.0297]

# Uri workspace randomization bounds
WS_RADIUS_MIN = 0.20    # m  — avoid base singularity
WS_RADIUS_MAX = 1.10    # m
WS_X_MIN      = -0.45   # m
WS_Y_MAX      = 0.28    # m
WS_Z_MIN      = 0.05    # m  — small margin above floor

# Rotation randomization bounds (radians)
ROT_MAX = math.pi / 2

# Peg-in: Ayal starts this far back from the mating pose along its own -Z
PEG_OFFSET_Z = 0.026    # m

MAX_ROT_FAILS = 15      # rotation retries before re-randomizing position
MAX_POS_ATTEMPTS = 200  # position attempts before giving up

SPEED = 0.05
ACCEL = 0.10
SUPER_SPEED = 0.2

N_SAMPLES      = 30
SAMPLE_DELAY_S = 0.002
SETTLE_S       = 0.5
# --------------

# region Reused helpers (from previous files. i think they are copied) ────────────────────────────────────────────────────────────
def fmt(v):
    return "[" + ", ".join(f"{x:.4f}" for x in v) + "]"

def avg_wrench(robot):
    samples = []
    for i in range(N_SAMPLES):
        if i:
            time.sleep(SAMPLE_DELAY_S)
        samples.append(list(robot.recieve.getActualTCPForce())[:6])
    return [sum(s[j] for s in samples) / N_SAMPLES for j in range(6)]

def wrench_to_tcp(wrench_base, pose_base):
    return list(gemini.wrench_trans(wrench_base, pose_base, base_to_tcp=True, include_translation=False))

def tcp_z_in_base(pose):
    R = gemini.rotvec_to_R(pose[3:6])
    return [R[0, 2], R[1, 2], R[2, 2]]

def load_calibration():
    with open(CALIBRATION_FILE, "r") as f:
        return json.load(f)
# endregion

# ── Geometry helpers ─────────────────────────────────────────────────────────
def radial_rotation(x, y, z):
    """
    Returns [rx, ry, rz] rotation vector such that TCP Z+ points
    radially from the origin toward (x, y, z).
    """
    import numpy as np

    z_axis = np.array([x, y, z], dtype=float)
    z_axis /= np.linalg.norm(z_axis)

    up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(z_axis, up))) > 0.9:
        up = np.array([0.0, 1.0, 0.0])

    x_axis = np.cross(up, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)

    R = np.column_stack([x_axis, y_axis, z_axis])

    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    theta = math.acos(max(-1.0, min(1.0, 0.5 * (trace - 1.0))))

    if theta < 1e-9:
        return [0.0, 0.0, 0.0]

    s = 2.0 * math.sin(theta)
    return [
        (R[2, 1] - R[1, 2]) / s * theta,
        (R[0, 2] - R[2, 0]) / s * theta,
        (R[1, 0] - R[0, 1]) / s * theta,
    ]

# ── Step 1: clear_ayal ────────────────────────────────────────────────────────
def clear_ayal(ayal):
    """Move Ayal to its base joint config, which is unreachable by Uri."""
    print("Moving Ayal to base (clear) position...")
    ayal.control.moveJ_IK(AYAL_BASE_JOINT, SUPER_SPEED, ACCEL, False)
    time.sleep(SETTLE_S)
    print("Ayal cleared.")

# ── Step 3: randomize_next ────────────────────────────────────────────────────
def suggest_pose_uri():
    """Rejection-sample a random [x,y,z] within Uri's allowed workspace."""
    for _ in range(10_000):
        x = random.uniform(WS_X_MIN, WS_RADIUS_MAX)
        y = random.uniform(-WS_RADIUS_MAX, WS_Y_MAX)
        z = random.uniform(WS_Z_MIN, WS_RADIUS_MAX)
        r = math.sqrt(x**2 + y**2 + z**2)
        if WS_RADIUS_MIN <= r <= WS_RADIUS_MAX:
            return [x, y, z]
    raise RuntimeError("Could not sample a valid Uri position after 10 000 tries.")

def check_reach_half_ayal(ayal, xyz, calib_pose):
    """
    Coarse position-only check: can Ayal reach the mirror of this xyz?
    Uses neutral orientation (0,0,0) as placeholder for Uri's rotation.
    """
    placeholder_uri_pose = xyz + [0.0, 0.0, 0.0]
    ayal_approx = list(gemini.calculate_mirror_position(placeholder_uri_pose, calib_pose, flip_trans=True))
    ayal_approx[3:6] = radial_rotation(*ayal_approx[:3])
    try:
        return bool(ayal.control.getInverseKinematicsHasSolution(ayal_approx))
    except RuntimeError:
        return False

def suggest_rotation_uri():
    """Return a random [rx, ry, rz] rotation vector."""
    return [random.uniform(-ROT_MAX, ROT_MAX) for _ in range(3)]

def _uri_full_ok(uri, pose):
    """True if Uri can reach full pose with acceptable manipulability."""
    reachable, _ = check_pose(uri, pose, min_m=MIN_MANIPULABILITY_URI)
    return reachable

def _ayal_full_ok(ayal, uri_pose, calib_pose):
    """True if Ayal can reach the mirror of uri_pose with acceptable manipulability."""
    ayal_pose = list(gemini.calculate_mirror_position(uri_pose, calib_pose, flip_trans=True))
    reachable, _ = check_pose(ayal, ayal_pose, min_m=MIN_MANIPULABILITY_AYAL)
    return reachable

def randomize_next(uri, ayal, calib_pose):
    """
    Steps 3.1 – 3.4: sample a pose for Uri that both robots can reach.
    Returns the accepted Uri pose, or raises RuntimeError after too many failures.
    """
    for pos_attempt in range(MAX_POS_ATTEMPTS):
        # 3.1 suggest xyz
        xyz = suggest_pose_uri()
        print(f"  [pos {pos_attempt+1}] trying xyz={fmt(xyz)}")

        # 3.2 coarse Ayal position check
        if not check_reach_half_ayal(ayal, xyz, calib_pose):
            print("    Ayal half-reach FAIL — re-randomizing position.")
            continue

        rot_fails = 0
        while rot_fails < MAX_ROT_FAILS:
            # 3.3 suggest rotation, check Uri full pose
            rot = suggest_rotation_uri()
            uri_pose = xyz + rot
            print(f"    [rot {rot_fails+1}/{MAX_ROT_FAILS}] trying rot={fmt(rot)}")

            if not _uri_full_ok(uri, uri_pose):
                print("    Uri full-reach FAIL.")
                rot_fails += 1
                continue

            # 3.4 check Ayal full pose
            if not _ayal_full_ok(ayal, uri_pose, calib_pose):
                print("    Ayal full-reach FAIL.")
                rot_fails += 1
                continue

            print(f"  Accepted Uri pose: {fmt(uri_pose)}")
            return uri_pose

        print(f"  Exhausted {MAX_ROT_FAILS} rotation attempts — re-randomizing position.")

    raise RuntimeError(f"Failed to find a valid pose after {MAX_POS_ATTEMPTS} position attempts.")

# ── Step 4: move_both_to_new ──────────────────────────────────────────────────
def compute_ayal_start_pose(ayal_final_pose):
    """26 mm back along Ayal's own -Z axis from its mating pose."""
    z = tcp_z_in_base(ayal_final_pose)
    start = list(ayal_final_pose)
    start[0] -= PEG_OFFSET_Z * z[0]
    start[1] -= PEG_OFFSET_Z * z[1]
    start[2] -= PEG_OFFSET_Z * z[2]
    return start

def move_both_to_new(uri, ayal, uri_pose, calib_pose):
    # 4.1 move Uri
    print(f"Moving Uri to {fmt(uri_pose)}")
    uri.control.moveJ_IK(uri_pose, SPEED, ACCEL, False)
    time.sleep(SETTLE_S)

    # 4.2 compute Ayal's final (mating) pose
    ayal_final = list(gemini.calculate_mirror_position(uri_pose, calib_pose, flip_trans=True))
    print(f"Ayal final (mating) pose: {fmt(ayal_final)}")

    # 4.3 compute Ayal's start pose (26 mm back in -Z)
    ayal_start = compute_ayal_start_pose(ayal_final)
    print(f"Ayal start pose (-26 mm Z): {fmt(ayal_start)}")

    # 4.4 move Ayal
    ayal.control.moveJ_IK(ayal_start, SPEED, ACCEL, False)
    time.sleep(SETTLE_S)

    return ayal_final, ayal_start

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    calib_pose = load_calibration()
    print(f"Calibration pose: {fmt(calib_pose)}")

    uri  = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    uri.connect(False)
    ayal.connect(False)
    
    try:
        # 1. Clear Ayal
        clear_ayal(ayal)

        # 2. Ask user to place connector on Uri
        input("Place the connector on Uri, then press Enter to continue...")

        # 3. Find a valid randomized pose
        uri_pose = randomize_next(uri, ayal, calib_pose)

        # 4. Move both robots to the new pose
        ayal_final, ayal_start = move_both_to_new(uri, ayal, uri_pose, calib_pose)

        print("\nReady.")
        print(f"  Uri pose:        {fmt(uri_pose)}")
        print(f"  Ayal final pose: {fmt(ayal_final)}")
        print(f"  Ayal start pose: {fmt(ayal_start)}")

    finally:
        uri.disconnect()
        ayal.disconnect()

if __name__ == "__main__":
    main()
