#!/usr/bin/env python3
"""Measure how long `_ik_ok()` takes against the live robot.

Run:  python playground/src/time_check_pose.py
"""

import time
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import uri_if
from uri_calibration.src.is_valid_pose import _ik_ok, _manipulability

URI_HOST = "192.168.57.101"
N_TRIALS = 50


def time_call(label, fn, n=N_TRIALS):
    samples = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        samples.append((time.perf_counter() - t0) * 1000.0)
    samples.sort()
    print(
        f"{label:35s}  n={n}  "
        f"min={samples[0]:7.2f} ms  "
        f"median={statistics.median(samples):7.2f} ms  "
        f"mean={statistics.mean(samples):7.2f} ms  "
        f"p95={samples[int(0.95 * n)]:7.2f} ms  "
        f"max={samples[-1]:7.2f} ms"
    )


def main():
    uri = uri_if.RMPLAB_Uri(URI_HOST)
    uri.connect(False)

    current = list(uri.recieve.getActualTCPPose())
    print(f"Current TCP: {[round(v, 4) for v in current]}\n")

    # A pose we expect to be reachable (the current one)
    reachable_pose = list(current)

    # A pose we expect to be unreachable (far outside workspace)
    unreachable_pose = [3.0, 3.0, 3.0, 0.0, 0.0, 0.0]

    # --- Individual RTDE calls ---
    time_call(
        "getInverseKinematicsHasSolution",
        lambda: uri.control.getInverseKinematicsHasSolution(reachable_pose),
    )
    time_call(
        "isPoseWithinSafetyLimits",
        lambda: uri.control.isPoseWithinSafetyLimits(reachable_pose),
    )
    time_call(
        "getActualQ",
        lambda: uri.recieve.getActualQ(),
    )
    qnear = list(uri.recieve.getActualQ())
    time_call(
        "getInverseKinematics(pose, qnear)",
        lambda: uri.control.getInverseKinematics(reachable_pose, qnear),
    )
    q = uri.control.getInverseKinematics(reachable_pose, qnear)
    time_call(
        "_manipulability (rtb jacob0)",
        lambda: _manipulability(q),
    )

    # --- Full _ik_ok ---
    print()
    time_call(
        "_ik_ok (reachable pose)",
        lambda: _ik_ok(uri, reachable_pose),
    )
    time_call(
        "_ik_ok (unreachable pose)",
        lambda: _ik_ok(uri, unreachable_pose),
    )

    uri.disconnect()


if __name__ == "__main__":
    main()
