"""
Resample F/T at known TCP configurations, storing joint coordinates.

Moves the robot to each of the 7 hardcoded TCP poses via moveJ (IK),
samples averaged force/torque, and saves results keyed by configuration
index with joint angles in radians.

Multiple loops accumulate multiple samples per configuration.
The output file is updated after every full loop so data survives
interruptions.
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import uri_if

# Defaults matching uris_gui/config.py
DEFAULT_SPEED = 0.5
DEFAULT_ACCELERATION = 0.5

SCRIPT_DIR = Path(__file__).parent
DEFAULT_OUTPUT = SCRIPT_DIR / "ft_joint_samples.json"

# 7 TCP poses [X, Y, Z, Rx, Ry, Rz] from uris_gui/tcp_force_data.json
POSES = [
    [ 0.2221, -0.2412,  0.7588,  1.0837, -1.5604,  1.3677],
    [ 0.8726, -0.2583,  0.5908, -2.2492,  1.1489, -1.5689],
    [ 0.3322, -0.1718,  0.2886,  1.9204, -1.5521,  1.4449],
    [-0.3043, -0.3813,  0.2856,  0.2058, -2.6234,  0.0639],
    [ 0.0025, -0.6084,  0.2197, -1.3646,  2.4152,  1.1194],
    [-0.1831, -0.4036,  1.1396,  0.3463, -0.6780,  1.2316],
    [ 0.0769, -0.0268,  0.3058,  0.2312, -2.8667, -0.1328],
]


def load_or_init_output(path: Path, poses):
    """Load existing output JSON or initialise a fresh structure."""
    if path.exists():
        with open(path, "r") as f:
            return json.load(f)
    output = {}
    for i, pose in enumerate(poses):
        output[str(i)] = {
            "pose": [round(v, 4) for v in pose],
            "samples": [],
        }
    return output


def save_output(path: Path, data):
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def r4(value):
    """Round a single float to 4 decimal places."""
    return round(value, 4)


def sample_ft(uri, avg_samples: int):
    """Collect avg_samples readings of TCP F/T and return the averaged 6-vector."""
    accum = [0.0] * 6
    for _ in range(avg_samples):
        forces = uri.recieve.getActualTCPForce()
        for j in range(6):
            accum[j] += forces[j]
        time.sleep(0.01)  # 10 ms between sub-samples
    return [v / avg_samples for v in accum]


def check_robot_ok(uri):
    """Return True if the robot is in a normal state, False on any error."""
    try:
        if not uri.is_connected():
            print("ERROR: Lost connection to robot.")
            return False
        if uri.recieve.isProtectiveStopped():
            print("ERROR: Robot is in protective stop.")
            return False
        if uri.recieve.isEmergencyStopped():
            print("ERROR: Robot is in emergency stop.")
            return False
        return True
    except Exception as e:
        print(f"ERROR: Failed to read robot status: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Move through saved TCP configs, resample F/T, store with joint coords.")
    parser.add_argument("--ip", default="192.168.56.101",
                        help="Robot IP (default: 192.168.56.101)")
    parser.add_argument("--loops", type=int, default=1,
                        help="Number of full loops over all configs (default: 1)")
    parser.add_argument("--avg-samples", type=int, default=10,
                        help="Sub-samples averaged into one F/T reading (default: 10)")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="Seconds to wait after moveJ before sampling (default: 2.0)")
    parser.add_argument("--zero-ft", action="store_true",
                        help="Zero the F/T sensor before each sample (off by default)")
    parser.add_argument("--output", type=str, default=str(DEFAULT_OUTPUT),
                        help="Path to output JSON file")
    args = parser.parse_args()

    output_path = Path(args.output)

    poses = POSES
    num_configs = len(poses)
    print(f"{num_configs} hardcoded configurations")

    # Load or init output
    output = load_or_init_output(output_path, poses)

    # Connect
    uri = uri_if.RMPLAB_Uri(args.ip)
    uri.connect(False)
    print(f"Connected to {args.ip}")

    try:
        for loop_idx in range(args.loops):
            print(f"\n=== Loop {loop_idx + 1}/{args.loops} ===")

            for cfg_idx, pose in enumerate(poses):
                print(f"  Config {cfg_idx + 1}/{num_configs}  "
                      f"pose=[{', '.join(f'{v:.4f}' for v in pose)}]")

                # Check robot state before moving
                if not check_robot_ok(uri):
                    print("Aborting — saving collected data ...")
                    save_output(output_path, output)
                    return

                # IK with current q as seed
                qnear = list(uri.recieve.getActualQ())
                q_target = uri.control.getInverseKinematics(pose, qnear)

                # Move (blocking)
                uri.control.moveJ(q_target, DEFAULT_SPEED, DEFAULT_ACCELERATION, False)

                # Settle
                print(f"    Settling {args.settle}s ...")
                time.sleep(args.settle)

                # Check robot state after settling
                if not check_robot_ok(uri):
                    print("Aborting — saving collected data ...")
                    save_output(output_path, output)
                    return

                # Optional F/T zero
                if args.zero_ft:
                    uri.control.zeroFtSensor()
                    time.sleep(0.5)

                # Sample F/T (averaged)
                ft = sample_ft(uri, args.avg_samples)

                # Read actual joint angles (radians)
                joints = list(uri.recieve.getActualQ())

                # Build sample record (all values 4-digit precision)
                sample = {
                    "timestamp": datetime.now().isoformat(),
                    "num_avg": args.avg_samples,
                    "joints_rad": [r4(j) for j in joints],
                    "FX": r4(ft[0]),
                    "FY": r4(ft[1]),
                    "FZ": r4(ft[2]),
                    "MX": r4(ft[3]),
                    "MY": r4(ft[4]),
                    "MZ": r4(ft[5]),
                }

                output[str(cfg_idx)]["samples"].append(sample)
                print(f"    F/T = [{', '.join(f'{v:.4f}' for v in ft)}]")

            # Save after every full loop
            save_output(output_path, output)
            print(f"  Saved after loop {loop_idx + 1} → {output_path}")

    except KeyboardInterrupt:
        print("\nInterrupted — saving collected data ...")
        save_output(output_path, output)
        print(f"Partial data saved → {output_path}")

    finally:
        uri.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
