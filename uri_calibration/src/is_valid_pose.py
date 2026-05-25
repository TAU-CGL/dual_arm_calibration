#!/usr/bin/env python3
"""Check whether a target TCP pose is reachable and if a straight-line path exists."""

import os
import sys
import math
import numpy as np
import roboticstoolbox as rtb
import uri_if

N_WAYPOINTS = 50  # number of intermediate samples along the linear path

# Jacobian-based singularity thresholds
# Manipulability (Yoshikawa index) = sqrt(det(J @ J.T)); 0 = singular
MIN_MANIPULABILITY_URI  = 0.0035
MIN_MANIPULABILITY_AYAL = 0.035
MIN_Z_FOR_URI = 0.2  # minimum Z height for UR5e TCP to avoid hitting the table

_UR5E_MODEL = rtb.models.UR5()

def _lerp_pose(start, end, t):
    """Linearly interpolate between two TCP poses (position + rotation vector)."""
    return [s + t * (e - s) for s, e in zip(start, end)]

def _manipulability(q):
    """
    Compute the Yoshikawa manipulability index for the UR5e at joint config q.
    m = sqrt(det(J @ J.T))  —  0 at a singularity.
    """
    J = _UR5E_MODEL.jacob0(q)
    return float(np.sqrt(max(0.0, np.linalg.det(J @ J.T))))

def _ik_ok(uri, pose, min_m=MIN_MANIPULABILITY_URI):
    """Return True if the pose has an IK solution, is within safety limits, and is not near a singularity."""
    try:
        if not uri.control.getInverseKinematicsHasSolution(pose):
            return False
    except RuntimeError:
        return False
    try:
        if not uri.control.isPoseWithinSafetyLimits(pose):
            return False
    except RuntimeError:
        return False
    # Singularity check
    try:
        qnear = list(uri.recieve.getActualQ())
        q = uri.control.getInverseKinematics(pose, qnear)
        m_candidate = _manipulability(q)
        if m_candidate < min_m:
            m_current = _manipulability(qnear)
            print(f"  ⚠  Waypoint near singularity:  current m = {m_current:.6f},  candidate m = {m_candidate:.6f}  (threshold = {min_m})")
            return False
    except RuntimeError:
        return False
    return True

def is_valid_pose(uri, target_pose, min_m=MIN_MANIPULABILITY_URI):
    """Return (is_reachable, has_straight_line) for the given [x,y,z,rx,ry,rz]."""

    # --- 1. Is the target reachable at all? ---
    try:
        is_reachable = uri.control.getInverseKinematicsHasSolution(target_pose)
    except RuntimeError:
        is_reachable = False
    if not is_reachable:
        return False, False

    # --- 2. Is it within safety limits? ---
    try:
        is_safe = uri.control.isPoseWithinSafetyLimits(target_pose)
    except RuntimeError:
        is_safe = False
    if not is_safe:
        print("  ⚠  Pose has an IK solution but violates safety limits.")
        return True, False

    # --- 3. Is the target near a singularity? ---
    try:
        qnear = list(uri.recieve.getActualQ())
        m_current = _manipulability(qnear)
        q_target = uri.control.getInverseKinematics(target_pose, qnear)
        m_target = _manipulability(q_target)
        if m_target < min_m:
            print(f"  ⚠  Pose is near a singularity.")
            print(f"       current m = {m_current:.6f}")
            print(f"       target  m = {m_target:.6f}  (threshold = {min_m})")
            return True, False
    except RuntimeError:
        return True, False

    # --- Additional check: for UR5e, ensure target Z is above minimum to avoid table collision ---
    if target_pose[2] < MIN_Z_FOR_URI:
        print(f"  ⚠  Pose is below minimum Z height of {MIN_Z_FOR_URI} m for UR5e; may collide with table.")
        return True, False

    # --- 4. Check straight-line path (sample intermediate waypoints) ---
    current_pose = list(uri.recieve.getActualTCPPose())

    for i in range(1, N_WAYPOINTS + 1):
        t = i / N_WAYPOINTS
        waypoint = _lerp_pose(current_pose, target_pose, t)

        if not _ik_ok(uri, waypoint, min_m):
            return True, False

    return True, True

def parse_pose(raw):
    """Parse a pose from '[x, y, z, rx, ry, rz]' bracket notation (pass in quotes)."""
    text = " ".join(raw).strip("[] ")
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) != 6:
        print(f'Usage: python {sys.argv[0]} "[x, y, z, rx, ry, rz]"')
        print(f"  (all values in meters / radians)")
        sys.exit(1)
    return [float(v) for v in parts]

def main():
    target = parse_pose(sys.argv[1:])

    print(f"Target pose: [{', '.join(f'{v:.4f}' for v in target)}]")
    print()

    uri = uri_if.RMPLAB_Uri(uri_if.HOST_URI)
    try:
        uri.connect(False)
    except Exception as e:
        print(f"ERROR: could not connect to {uri_if.HOST_URI}: {e}")
        sys.exit(1)

    current = list(uri.recieve.getActualTCPPose())
    print(f"Current TCP: [{', '.join(f'{v:.4f}' for v in current)}]")
    print()

    reachable, straight_line = is_valid_pose(uri, target)

    if not reachable:
        print("❌  Pose is NOT reachable (no IK solution).")
    elif straight_line:
        print("✅  Pose is reachable.")
        print("✅  Straight-line (moveL) path exists from current pose.")
    else:
        print("✅  Pose is reachable.")
        print("⚠️   No straight-line path — use moveJ_IK instead of moveL.")

    uri.disconnect()

def test__ayal_manipulability():
    ayal = uri_if.RMPLAB_Uri(uri_if.HOST_AYAL)
    ayal.connect(False)
    q = list(ayal.recieve.getActualQ())
    m = _manipulability(q)
    print(f"Ayal current q: {q}")
    print(f"Ayal current manipulability: {m:.6f}")
    ayal.disconnect()

if __name__ == "__main__":
    test__ayal_manipulability()
    # main()
