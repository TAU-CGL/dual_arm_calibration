#!/usr/bin/env python3
"""
Grip-stage sequence for URI-AYAL dual-arm setup.

Manual mode (default):
  1  — sample URI and AYAL TCP poses
  2  — calculate AYAL TCP in URI TCP frame
  3  — open AYAL gripper half way (pos=120)
  4  — AYAL teach mode, wait for user
  5  — sample URI and AYAL TCP poses
  7  — close AYAL gripper
  8  — AYAL teach mode, wait for user
  9  — sample URI and AYAL TCP poses
  10 — AYAL teach mode, wait for user
  11 — open AYAL gripper half way (pos=120)

Auto mode (--auto):
  0  — move AYAL TCP to REL_POS_0 in URI TCP frame
  1  — sample URI and AYAL TCP poses
  2  — calculate AYAL TCP in URI TCP frame
  3  — open AYAL gripper half way (pos=120)
  4  — move AYAL TCP to REL_POS_4 in URI TCP frame
  5  — sample URI and AYAL TCP poses
  7  — close AYAL gripper
  8  — move AYAL TCP to REL_POS_8 in URI TCP frame
  (stops here)

Usage:
  python peg_grip_stages.py              # manual, all stages
  python peg_grip_stages.py 3            # manual, stage 3 only
  python peg_grip_stages.py --auto       # auto, all stages
  python peg_grip_stages.py --auto 4     # auto, stage 4 only
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _RMP_LAB_ROOT not in sys.path:
    sys.path.insert(0, _RMP_LAB_ROOT)

_CALIB_DIR = os.path.join(_RMP_LAB_ROOT, "calibration")
if _CALIB_DIR not in sys.path:
    sys.path.insert(0, _CALIB_DIR)

import uri_if
import gemini_calc_v2 as gemini

URI_HOST  = "192.168.56.101"
AYAL_HOST = "192.168.57.101"
CALIB_FILE = Path(_RMP_LAB_ROOT) / "uris_gui" / "calibration.json"

GRIPPER_HALF  = 120
GRIPPER_SPEED = 170
GRIPPER_FORCE = 200
SETTLE_S  = 0.5
AUTO_SPEED = 0.1
AUTO_ACCEL = 0.05

# AYAL TCP expressed in URI TCP frame, recorded from real runs.
REL_POS_IN = [0.0,  0.0,  0.0,  0.0,  3.1416,  0.0]
REL_POS_OUT = [0.0,  0.0,  0.03,  0.0,  3.1416,  0.0]

# REL_POS_0: initial contact pose (from stage 2 output)
REL_POS_0 = [-0.0012,  0.0021,  0.0371,  0.0728,  2.9893,  0.1085]
# REL_POS_4: pose after teach in stage 4 (gripper half-open, pre-grasp)
REL_POS_4 = [ 0.0013,  0.0053, -0.0204, -0.0299, -3.1228,  0.0838]
# REL_POS_8: pose after teach in stage 8 (gripper closed, post-grasp move)
REL_POS_8  = [ 0.0002,  0.0045,  0.0066, -0.0345, -3.1380,  0.1277]
# REL_POS_9: AYAL_POS_AWAY in URI_POS_STATIC TCP frame
REL_POS_9  = [-0.0123, -0.0160,  0.2001,  0.0457,  3.1175, -0.2164]
# REL_POS_10: AYAL_POS_FAR_AWAY in URI_POS_STATIC TCP frame
REL_POS_10 = [ 0.2154, -0.6870,  0.1385, -1.2072,  2.0441, -1.3963]

URI_POS_STATIC = [0.3945, -0.4743, 0.5918, 0.9645, 0.5197, -1.0038]
AYAL_POS_AWAY = [-0.1367, 0.3151, 0.4239, -1.2304, -2.048, -1.0349]
AYAL_POS_FAR_AWAY = [-0.6168, 0.1359, -0.0694, 0.0396, -3.0058, 0.0929]
AYAL_POS_FINAL = [-0.1612, 0.3302, -0.234, 1.3443, 2.8289, 0.0119]

# AYAL_POSE_INSIDE = 

def fmt(v):
    return "[" + ", ".join(f"{x:.4f}" for x in v) + "]"


def load_calibration():
    with open(CALIB_FILE) as f:
        return json.load(f)


def connect_robots():
    uri  = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    uri.connect(False)
    ayal.connect(False)
    return uri, ayal


def sample_tcps(uri, ayal):
    uri_tcp  = list(uri.recieve.getActualTCPPose())
    ayal_tcp = list(ayal.recieve.getActualTCPPose())
    print(f"  URI  TCP: {fmt(uri_tcp)}")
    print(f"  AYAL TCP: {fmt(ayal_tcp)}")
    return uri_tcp, ayal_tcp


def ayal_tcp_in_uri_tcp_frame(uri_tcp, ayal_tcp, calib_pose):
    """Express AYAL TCP as a pose in URI TCP's coordinate frame."""
    T_U_TCP = gemini.pose_to_T(uri_tcp)     # URI base  → URI TCP
    T_A_TCP = gemini.pose_to_T(ayal_tcp)    # AYAL base → AYAL TCP
    T_U_A   = gemini.pose_to_T(calib_pose)  # URI base  → AYAL base
    T_rel = np.linalg.inv(T_U_TCP) @ T_U_A @ T_A_TCP
    return list(gemini.T_to_pose(T_rel))


def rel_to_ayal_tcp(rel_pose, uri_tcp, calib_pose):
    """Convert a relative pose (AYAL TCP in URI TCP frame) to AYAL TCP in AYAL base frame.

    Inverse of ayal_tcp_in_uri_tcp_frame:
      T_A_TCP = inv(T_U_A) @ T_U_TCP @ T_UTCP_ATCP
    """
    T_U_TCP      = gemini.pose_to_T(uri_tcp)    # URI base → URI TCP
    T_U_A        = gemini.pose_to_T(calib_pose) # URI base → AYAL base
    T_UTCP_ATCP  = gemini.pose_to_T(rel_pose)   # URI TCP  → AYAL TCP
    T_A_TCP = np.linalg.inv(T_U_A) @ T_U_TCP @ T_UTCP_ATCP
    return list(gemini.T_to_pose(T_A_TCP))


def move_ayal_to_rel(uri, ayal, calib, rel_pose, label, axis=None, new_val=0):
    """Move AYAL TCP to rel_pose expressed in current URI TCP frame."""
    uri_tcp = list(uri.recieve.getActualTCPPose())
    target  = rel_to_ayal_tcp(rel_pose, uri_tcp, calib)
    if axis is not None:
        idx = "xyz".index(axis)
        target[idx] = new_val
    print(f"  URI  TCP (current): {fmt(uri_tcp)}")
    print(f"  AYAL target TCP:    {fmt(target)}")
    ayal.control.moveL(target, AUTO_SPEED, AUTO_ACCEL, False)
    print(f"  Moved to {label}.")


# ── stages (shared) ───────────────────────────────────────────────────────────

def stage_1(uri, ayal, calib, state):
    print("\n[Stage 1] Sampling URI and AYAL TCP poses")
    uri_tcp, ayal_tcp = sample_tcps(uri, ayal)
    state["uri_tcp_1"]  = uri_tcp
    state["ayal_tcp_1"] = ayal_tcp


def stage_2(uri, ayal, calib, state):
    print("\n[Stage 2] AYAL TCP expressed in URI TCP frame")
    uri_tcp  = state.get("uri_tcp_1")  or list(uri.recieve.getActualTCPPose())
    ayal_tcp = state.get("ayal_tcp_1") or list(ayal.recieve.getActualTCPPose())
    rel = ayal_tcp_in_uri_tcp_frame(uri_tcp, ayal_tcp, calib)
    print(f"  AYAL TCP in URI TCP frame: {fmt(rel)}")
    state["ayal_in_uri_tcp_1"] = rel


def stage_3(uri, ayal, calib, state):
    print(f"\n[Stage 3] Opening AYAL gripper to {GRIPPER_HALF}")
    ayal.gripper.move_and_wait_for_pos(GRIPPER_HALF, GRIPPER_SPEED, GRIPPER_FORCE)
    print("  Done.")


def stage_5(uri, ayal, calib, state):
    print("\n[Stage 5] Sampling URI and AYAL TCP poses")
    uri_tcp, ayal_tcp = sample_tcps(uri, ayal)
    state["uri_tcp_5"]  = uri_tcp
    state["ayal_tcp_5"] = ayal_tcp


def stage_7(uri, ayal, calib, state):
    print("\n[Stage 7] Closing AYAL gripper")
    ayal.gripper.close(GRIPPER_SPEED, GRIPPER_FORCE)
    print("  Done.")


def stage_9(uri, ayal, calib, state):
    print("\n[Stage 9] Sampling URI and AYAL TCP poses")
    uri_tcp, ayal_tcp = sample_tcps(uri, ayal)
    state["uri_tcp_9"]  = uri_tcp
    state["ayal_tcp_9"] = ayal_tcp


def stage_10(uri, ayal, calib, state):
    print("\n[Stage 10] AYAL teach mode")
    ayal.control.teachMode()
    input("  AYAL is in teach mode. Position robot, then press Enter... ")
    ayal.control.endTeachMode()
    time.sleep(SETTLE_S)


def stage_11(uri, ayal, calib, state):
    print(f"\n[Stage 11] Opening AYAL gripper to {GRIPPER_HALF}")
    ayal.gripper.move_and_wait_for_pos(GRIPPER_HALF, GRIPPER_SPEED, GRIPPER_FORCE)
    print("  Done.")


# ── stages (manual only) ──────────────────────────────────────────────────────

def stage_4_manual(_uri, ayal, _calib):
    print("\n[Stage 4] AYAL teach mode")
    ayal.control.teachMode()
    input("  AYAL is in teach mode. Position robot, then press Enter... ")
    ayal.control.endTeachMode()
    time.sleep(SETTLE_S)


def stage_8_manual(_uri, ayal, _calib):
    print("\n[Stage 8] AYAL teach mode")
    ayal.control.teachMode()
    input("  AYAL is in teach mode. Position robot, then press Enter... ")
    ayal.control.endTeachMode()
    time.sleep(SETTLE_S)


# ── stages (auto only) ────────────────────────────────────────────────────────

def stage_uri_teach(uri, _ayal, _calib):
    print("\n[Auto pre-0] URI teach mode")
    uri.control.teachMode()
    input("  URI is in teach mode. Position robot, then press Enter... ")
    uri.control.endTeachMode()
    time.sleep(SETTLE_S)


def stage_0_auto(uri, ayal, calib):
    print("\n[Stage 0] Moving AYAL TCP to REL_POS_0")
    move_ayal_to_rel(uri, ayal, calib, REL_POS_0, "REL_POS_0")


def stage_4_auto(uri, ayal, calib):
    print("\n[Stage 4] Moving AYAL TCP to REL_POS_4")
    move_ayal_to_rel(uri, ayal, calib, REL_POS_4, "REL_POS_4")


def stage_8_auto(uri, ayal, calib):
    print("\n[Stage 8] Moving AYAL TCP to REL_POS_8")
    move_ayal_to_rel(uri, ayal, calib, REL_POS_8, "REL_POS_8")


def stage_move_out(uri, ayal, calib):
    print("\n[Auto] Moving AYAL TCP to REL_POS_OUT")
    move_ayal_to_rel(uri, ayal, calib, REL_POS_OUT, "REL_POS_OUT")


def stage_move_in(uri, ayal, calib):
    print("\n[Auto] Moving AYAL TCP to REL_POS_IN")
    move_ayal_to_rel(uri, ayal, calib, REL_POS_IN, "REL_POS_IN")


def stage_uri_open(uri, _ayal, _calib):
    print("\n[Auto] Opening URI gripper")
    uri.gripper.open(GRIPPER_SPEED, GRIPPER_FORCE)
    print("  Done.")


def stage_uri_close(uri, _ayal, _calib):
    print("\n[Auto] Closing URI gripper")
    uri.gripper.close(GRIPPER_SPEED, GRIPPER_FORCE)
    print("  Done.")


def stage_9_auto(uri, ayal, calib):
    print("\n[Stage 9] Moving AYAL TCP to REL_POS_9")
    move_ayal_to_rel(uri, ayal, calib, REL_POS_9, "REL_POS_9")


def stage_10_auto(_uri, ayal, _calib):
    print(f"\n[Stage 10] Moving AYAL TCP to AYAL_POS_FINAL: {fmt(AYAL_POS_FINAL)}")
    ayal.control.moveL(AYAL_POS_FINAL, AUTO_SPEED, AUTO_ACCEL, False)
    print("  Done.")


# ── dispatch ──────────────────────────────────────────────────────────────────

MANUAL_STAGES = {
    1:  stage_1,
    2:  stage_2,
    3:  stage_3,
    4:  stage_4_manual,
    5:  stage_5,
    7:  stage_7,
    8:  stage_8_manual,
    9:  stage_9,
    10: stage_10,
    11: stage_11,
}
MANUAL_ORDER = [1, 2, 3, 4, 5, 7, 8, 9, 10, 11]

AUTO_STAGES = {
    30: stage_uri_teach,  # pre-0: URI teach mode
    0:  stage_0_auto,
    20: stage_move_out,   # pre 1: move to REL_POS_OUT
    21: stage_7,          # pre 2: close ayal gripper
    24: stage_uri_open,   # pre 3: open uri gripper
    22: stage_move_in,    # pre 4: move to REL_POS_IN
    23: stage_move_out,   # pre 5: move to REL_POS_OUT
    25: stage_uri_close,  # pre 6: close uri gripper
    3:  stage_3,
    4:  stage_4_auto,
    7:  stage_7,
    8:  stage_8_auto,
    9:  stage_9_auto,
    10: stage_10_auto,
    11: stage_11,
}
AUTO_ORDER = [30, 0, 20, 21, 24, 22, 23, 25, 3, 4, 7, 8, 9, 10, 11]


def main():
    parser = argparse.ArgumentParser(description="Grip-stage sequence for URI-AYAL setup")
    parser.add_argument("stage", nargs="?", type=int, default=None,
                        help="Run a single stage number instead of the full sequence")
    parser.add_argument("--auto", action="store_true",
                        help="Auto mode: replace teach-mode stages with pre-recorded moves")
    args = parser.parse_args()

    stages = AUTO_STAGES if args.auto else MANUAL_STAGES
    order  = AUTO_ORDER  if args.auto else MANUAL_ORDER

    if args.stage is not None:
        if args.stage not in stages:
            valid = ", ".join(str(s) for s in order)
            print(f"Unknown stage {args.stage}. Valid stages: {valid}")
            sys.exit(1)
        run_stages = [args.stage]
    else:
        run_stages = order

    uri, ayal = connect_robots()
    calib = load_calibration()

    for s in run_stages:
        stages[s](uri, ayal, calib)
        # if args.auto:
        #     input("\n  Stage done. Press Enter to continue... ")

    print("\nDone.")


if __name__ == "__main__":
    main()
