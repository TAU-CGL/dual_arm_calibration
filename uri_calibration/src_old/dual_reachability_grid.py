#!/usr/bin/env python3
"""Sample a 3D grid of TCP positions and check feasibility on both robots.

EXAMPLE: python3 uri_calibration/src_old/dual_reachability_grid.py \
    --x=-0.5:0.5 --y=-1:0 --z=0:1 --rxyz 0,0,0 --step 0.04

For every (x, y, z) in the grid, each robot's pose is classified as:
   1  reachable, within safety limits, and not near a singularity
   0  reachable + safe but near a singularity
  -1  no IK solution OR outside safety limits

The grid cell stores the *worst* of the two robots (min): -1 if either is
unreachable, 0 if both are reachable but at least one is singular, 1 if both
are clean.

A short timing pass is run first to estimate total wall-clock cost; the user is
asked to confirm before the full sweep starts.

Output: an .npz with axes (`xs`, `ys`, `zs`), the int8 `grid` (shape
(NX, NY, NZ), values in {-1, 0, 1}), and metadata.
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import uri_if
from uri_calibration.src import utils as gemini
from uri_calibration.src.is_valid_pose import _manipulability, MIN_MANIPULABILITY_URI, MIN_MANIPULABILITY_AYAL

def pose_status(robot, pose, min_m):
    """Return 1 (clean), 0 (reachable+safe but singular), or -1 (no IK / unsafe)."""
    try:
        if not robot.control.getInverseKinematicsHasSolution(pose):
            return -1
    except RuntimeError:
        return -1
    try:
        if not robot.control.isPoseWithinSafetyLimits(pose):
            return -1
    except RuntimeError:
        return -1
    try:
        qnear = list(robot.recieve.getActualQ())
        q = robot.control.getInverseKinematics(pose, qnear)
    except RuntimeError:
        return -1
    if _manipulability(q) < min_m:
        return 0
    return 1

URI_HOST = "192.168.56.101"
AYAL_HOST = "192.168.57.101"
CALIBRATION_FILE = REPO_ROOT / "shared" / "calibration.json"

def parse_range(s):
    lo, hi = (float(v) for v in s.split(":"))
    if hi < lo:
        raise argparse.ArgumentTypeError(f"range hi < lo: {s}")
    return lo, hi

def parse_triplet(s):
    parts = [float(v) for v in s.strip("[]").split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"need 3 comma-separated values: {s}")
    return parts

def build_axis(lo, hi, step):
    n = int(np.floor((hi - lo) / step + 1e-9)) + 1
    return np.linspace(lo, lo + (n - 1) * step, n)

def time_one_call(uri, ayal, pose_uri, ayal_in_uri_pose):
    """Measure one full (Uri + transform + Ayal) check."""
    t0 = time.perf_counter()
    pose_status(uri, pose_uri, MIN_MANIPULABILITY_URI)
    pose_ayal = gemini.calculate_mirror_position(
        P_source=pose_uri,
        P_BaseT2BaseS=ayal_in_uri_pose,
        flip_axis="y",
        flip_trans=True,
    )
    pose_status(ayal, pose_ayal, MIN_MANIPULABILITY_AYAL)
    return time.perf_counter() - t0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=parse_range, required=True, help="lo:hi (m)")
    ap.add_argument("--y", type=parse_range, required=True, help="lo:hi (m)")
    ap.add_argument("--z", type=parse_range, required=True, help="lo:hi (m)")
    ap.add_argument("--rxyz", type=parse_triplet, required=True,
                    help="constant orientation Rx,Ry,Rz (rad), e.g. '2.22,-2.22,0.0'")
    ap.add_argument("--step", type=float, required=True, help="grid step (m)")
    ap.add_argument("--ayal-in-uri", type=str, default=None,
                    help="pose '[x,y,z,rx,ry,rz]' of Ayal base in Uri frame; "
                         "defaults to value from calibration.json")
    ap.add_argument("--out", type=str, default="reachability_grid.npz")
    ap.add_argument("--timing-samples", type=int, default=10)
    ap.add_argument("--yes", action="store_true",
                    help="skip the time-estimate confirmation")
    args = ap.parse_args()

    rx, ry, rz = args.rxyz
    xs = build_axis(args.x[0], args.x[1], args.step)
    ys = build_axis(args.y[0], args.y[1], args.step)
    zs = build_axis(args.z[0], args.z[1], args.step)
    NX, NY, NZ = len(xs), len(ys), len(zs)
    total = NX * NY * NZ
    print(f"Grid: {NX} x {NY} x {NZ} = {total} samples  (step = {args.step} m)")
    print(f"  x in [{xs[0]:.3f}, {xs[-1]:.3f}]")
    print(f"  y in [{ys[0]:.3f}, {ys[-1]:.3f}]")
    print(f"  z in [{zs[0]:.3f}, {zs[-1]:.3f}]")
    print(f"  RxRyRz = ({rx:.4f}, {ry:.4f}, {rz:.4f})")

    # --- Resolve ayal_in_uri ---
    if args.ayal_in_uri is None:
        print(f"\nLoading ayal_in_uri from {CALIBRATION_FILE} ...")
        optimal_pose, _, _ = gemini.calculate_optimal_calibration(
            str(CALIBRATION_FILE), single_calibration=True
        )
        if optimal_pose is None:
            print("ERROR: failed to load calibration.")
            sys.exit(1)
        ayal_in_uri_pose = list(optimal_pose)
    else:
        ayal_in_uri_pose = [float(v) for v in args.ayal_in_uri.strip("[]").split(",")]
        if len(ayal_in_uri_pose) != 6:
            print("ERROR: --ayal-in-uri must have 6 values")
            sys.exit(1)
    print(f"ayal_in_uri = {[round(v, 4) for v in ayal_in_uri_pose]}")

    # --- Connect ---
    print("\nConnecting to robots...")
    uri = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    uri.connect(False)
    ayal.connect(False)

    try:
        # --- Time estimate ---
        # Sample a few points spread across the grid to get an honest mean.
        idx_samples = np.linspace(0, total - 1, args.timing_samples, dtype=int)
        durations = []
        print(f"\nTiming {args.timing_samples} sample calls to estimate total runtime...")
        for k, idx in enumerate(idx_samples):
            iz = idx % NZ
            iy = (idx // NZ) % NY
            ix = idx // (NY * NZ)
            pose = [float(xs[ix]), float(ys[iy]), float(zs[iz]), rx, ry, rz]
            durations.append(time_one_call(uri, ayal, pose, ayal_in_uri_pose))
        per_call = float(np.mean(durations))
        eta_s = per_call * total
        print(f"  per-call mean: {per_call * 1000:.2f} ms "
              f"(min {min(durations) * 1000:.2f}, max {max(durations) * 1000:.2f})")
        print(f"  estimated total: {eta_s:.1f} s  ({eta_s / 60:.1f} min)")

        if not args.yes:
            ans = input("\nProceed with full sweep? [y/N] ").strip().lower()
            if ans not in ("y", "yes"):
                print("Aborted.")
                return

        # --- Full sweep ---
        grid = np.zeros((NX, NY, NZ), dtype=np.int8)
        t_start = time.perf_counter()
        last_print = t_start
        done = 0
        for ix, x in enumerate(xs):
            for iy, y in enumerate(ys):
                for iz, z in enumerate(zs):
                    pose_uri = [float(x), float(y), float(z), rx, ry, rz]
                    s_uri = pose_status(uri, pose_uri, MIN_MANIPULABILITY_URI)
                    if s_uri == -1:
                        grid[ix, iy, iz] = -1
                    else:
                        pose_ayal = gemini.calculate_mirror_position(
                            P_source=pose_uri,
                            P_BaseT2BaseS=ayal_in_uri_pose,
                            flip_axis="y",
                            flip_trans=True,
                        )
                        s_ayal = pose_status(ayal, pose_ayal, MIN_MANIPULABILITY_AYAL)
                        grid[ix, iy, iz] = min(s_uri, s_ayal)
                    done += 1
                    now = time.perf_counter()
                    if now - last_print > 5.0:
                        elapsed = now - t_start
                        rate = done / elapsed
                        remaining = (total - done) / max(rate, 1e-9)
                        n_pos = int((grid == 1).sum())
                        n_sing = int((grid == 0).sum())
                        n_neg = int((grid == -1).sum())
                        print(f"  {done}/{total}  ({100 * done / total:5.1f}%)  "
                              f"elapsed {elapsed:6.1f}s  eta {remaining:6.1f}s  "
                              f"+1: {n_pos}  0: {n_sing}  -1: {n_neg}")
                        last_print = now

        elapsed = time.perf_counter() - t_start
        n_pos = int((grid == 1).sum())
        n_sing = int((grid == 0).sum())
        n_neg = int((grid == -1).sum())
        print(f"\nDone in {elapsed:.1f} s.")
        print(f"  reachable + non-singular (+1): {n_pos}/{total} ({100 * n_pos / total:.2f}%)")
        print(f"  singularity              ( 0): {n_sing}/{total} ({100 * n_sing / total:.2f}%)")
        print(f"  unreachable / unsafe     (-1): {n_neg}/{total} ({100 * n_neg / total:.2f}%)")

        out_path = Path(args.out).resolve()
        np.savez_compressed(
            out_path,
            grid=grid,
            xs=xs,
            ys=ys,
            zs=zs,
            rxyz=np.array([rx, ry, rz]),
            ayal_in_uri=np.array(ayal_in_uri_pose),
            step=args.step,
        )
        print(f"Saved: {out_path}")

    finally:
        uri.disconnect()
        ayal.disconnect()

if __name__ == "__main__":
    main()
