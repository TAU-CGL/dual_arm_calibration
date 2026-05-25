#!/usr/bin/env python3
"""Sweep reachability grids over a 3D orientation grid.

For each orientation (Rx, Ry, Rz) in the orientation grid, this calls
`dual_reachability_grid.py` with the same x/y/z range and translation step,
saving each result to `playground/output/reachability_grid_rx<...>_ry<...>_rz<...>.npz`.

Example:
  python playground/src/sweep_reachability_orientations.py \\
      --x=-0.5:0.5 --y=-0.5:0.5 --z=0.0:0.6 --step 0.05 \\
      --rx=-3.14:3.14 --ry=-3.14:3.14 --rz=0.0:0.0 --rstep 1.57
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
GRID_SCRIPT = REPO_ROOT / "playground" / "src" / "dual_reachability_grid.py"
OUTPUT_DIR = REPO_ROOT / "playground" / "output"

def parse_range(s):
    lo, hi = (float(v) for v in s.split(":"))
    if hi < lo:
        raise argparse.ArgumentTypeError(f"range hi < lo: {s}")
    return lo, hi

def build_axis(lo, hi, step):
    n = int(np.floor((hi - lo) / step + 1e-9)) + 1
    return np.linspace(lo, lo + (n - 1) * step, n)

def fmt(v):
    """Format a float for filenames: 4 decimals, trim trailing zeros."""
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return s if s else "0"

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=parse_range, required=True, help="lo:hi (m)")
    ap.add_argument("--y", type=parse_range, required=True, help="lo:hi (m)")
    ap.add_argument("--z", type=parse_range, required=True, help="lo:hi (m)")
    ap.add_argument("--step", type=float, required=True, help="translation step (m)")
    ap.add_argument("--rx", type=parse_range, required=True, help="lo:hi (rad)")
    ap.add_argument("--ry", type=parse_range, required=True, help="lo:hi (rad)")
    ap.add_argument("--rz", type=parse_range, required=True, help="lo:hi (rad)")
    ap.add_argument("--rstep", type=float, required=True, help="orientation step (rad)")
    ap.add_argument("--ayal-in-uri", type=str, default=None,
                    help="passed through to dual_reachability_grid.py")
    ap.add_argument("--skip-existing", action="store_true",
                    help="skip orientations whose output file already exists")
    args = ap.parse_args()

    rxs = build_axis(args.rx[0], args.rx[1], args.rstep)
    rys = build_axis(args.ry[0], args.ry[1], args.rstep)
    rzs = build_axis(args.rz[0], args.rz[1], args.rstep)
    n_orient = len(rxs) * len(rys) * len(rzs)
    print(f"Orientations: {len(rxs)} x {len(rys)} x {len(rzs)} = {n_orient}")
    print(f"  Rx in [{rxs[0]:.4f}, {rxs[-1]:.4f}]")
    print(f"  Ry in [{rys[0]:.4f}, {rys[-1]:.4f}]")
    print(f"  Rz in [{rzs[0]:.4f}, {rzs[-1]:.4f}]")
    print(f"Position grid passed through: x={args.x}, y={args.y}, z={args.z}, step={args.step}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    t_start = time.perf_counter()
    done = 0
    skipped = 0
    failed = 0
    for rx in rxs:
        for ry in rys:
            for rz in rzs:
                done += 1
                name = f"reachability_grid_rx{fmt(rx)}_ry{fmt(ry)}_rz{fmt(rz)}.npz"
                out_path = OUTPUT_DIR / name
                header = f"[{done}/{n_orient}] {name}"
                if args.skip_existing and out_path.exists():
                    print(f"{header}  (skipped, exists)")
                    skipped += 1
                    continue
                print(f"{header}")

                cmd = [
                    sys.executable, str(GRID_SCRIPT),
                    f"--x={args.x[0]}:{args.x[1]}",
                    f"--y={args.y[0]}:{args.y[1]}",
                    f"--z={args.z[0]}:{args.z[1]}",
                    "--step", str(args.step),
                    f"--rxyz={rx},{ry},{rz}",
                    "--out", str(out_path),
                    "--yes",
                ]
                if args.ayal_in_uri is not None:
                    cmd += [f"--ayal-in-uri={args.ayal_in_uri}"]

                rc = subprocess.call(cmd)
                if rc != 0:
                    print(f"  ! dual_reachability_grid.py exited with code {rc}")
                    failed += 1

    elapsed = time.perf_counter() - t_start
    print(f"\nFinished {done} orientations in {elapsed / 60:.1f} min "
          f"(skipped: {skipped}, failed: {failed})")
    print(f"Output dir: {OUTPUT_DIR}")

if __name__ == "__main__":
    main()
