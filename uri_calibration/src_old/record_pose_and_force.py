"""
this script would record the force and pose of ayal while connected to uri.
Uri would stay in place.
Start position is when the two are connected with the rectangular connector, and aligned such that the connector is free to move.
The connector used has 0.2 'Shpill' in each direction.
End position is when they are unconnected, facing each other.
Throughout the process, I will move Ayal to step by step, and record the pose and force at each step.
The puprose is to create a table of Delta_pose vs force, to be used for learning the pegging policy.

The steps are:
1. Set Ayal and Uri in the start position:
URI:  (+0.3695, -0.3221, 0.6966, 1.5708, 0.0000, 0.00000)
AYAL: (-0.1273, +0.6368, 0.4271, 0.0000, 2.2214, +2.2214)
connected with the rectangular connector
2. make sure the connector is free to move.
3. Run the script
4. it will record the pose and force at the start position, then ask you to move Ayal by a small step.
5. Move Ayal by the step, and press enter.
6. repeat 4-5 until you have covered the whole range of the connector.
"""
import json
import os
import random
import sys
import time
from pathlib import Path
from types import SimpleNamespace

_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _RMP_LAB_ROOT in sys.path:
    sys.path.remove(_RMP_LAB_ROOT)
sys.path.insert(0, _RMP_LAB_ROOT)

_CALIB = os.path.join(_RMP_LAB_ROOT, "calibration")
if _CALIB not in sys.path:
    sys.path.insert(0, _CALIB)

import uri_if
import gemini_calc_v2 as gemini

URI_HOST = "192.168.56.101"
AYAL_HOST = "192.168.57.101"

START_URI = [0.3695, -0.3221, 0.6966, 1.5708, 0.0000, 0.0000]
START_AYAL = [-0.1273, 0.6368, 0.4271, 0.0000, 2.2214, 2.2214]

N_SAMPLES = 50
SAMPLE_DELAY_S = 0.002
SETTLE_S = 0.5

BACK_Z_STEP = 0.001
AUTO_SPEED = 0.01
AUTO_ACCEL = 0.05
AUTO = "--auto" in sys.argv

# --- New Random Rotation Parameters ---
ROT_AVG = 0.00   # Average for random rotation perturbation (radians)
ROT_DEV = 0.02   # Standard deviation for random rotation perturbation (radians)

LOG_DIR = Path(__file__).parent.parent / "logs"
BASE_NAME = "record_pose_and_force"

# --- Noise & Bounding Parameters ---
ROT_AVG = 0.00   
ROT_DEV = 0.02   
ROT_BOUND = 0.04   # Maximum allowed rotation deviation (radians)

TRANS_AVG = 0.00
TRANS_DEV = 0.001
TRANS_BOUND = 0.002 # Maximum allowed translation deviation (meters)

def next_output_paths():
    n = 0
    while True:
        suffix = "" if n == 0 else f"_{n}"
        log = LOG_DIR / f"{BASE_NAME}{suffix}.log"
        data = LOG_DIR / f"{BASE_NAME}{suffix}.json"
        if not log.exists() and not data.exists():
            return log, data
        n += 1

def fmt(v):
    return "[" + ", ".join(f"{x:.4f}" for x in v) + "]"

def avg_wrench(robot):
    samples = []
    for i in range(N_SAMPLES):
        if i:
            time.sleep(SAMPLE_DELAY_S)
        samples.append(list(robot.recieve.getActualTCPForce())[:6])
    return [sum(s[j] for s in samples) / len(samples) for j in range(6)]

def wrench_to_tcp(wrench_base, pose_base):
    return list(gemini.wrench_trans(wrench_base, pose_base, base_to_tcp=True, include_translation=False))

def tcp_z_in_base(pose):
    R = gemini.rotvec_to_R(pose[3:6])
    return [R[0, 2], R[1, 2], R[2, 2]]

def connect_robots():
    uri = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    uri.connect(False)
    ayal.connect(False)
    return uri, ayal

def open_log(path):
    f = open(path, "w")

    def emit(line):
        print(line)
        f.write(line + "\n")
        f.flush()

    return f, emit

def setup_session():
    uri, ayal = connect_robots()
    log_path, data_path = next_output_paths()
    log_file, emit = open_log(log_path)
    emit(f"log_path:  {log_path}")
    emit(f"data_path: {data_path}")
    return SimpleNamespace(
        uri=uri, ayal=ayal,
        log_file=log_file, emit=emit,
        log_path=log_path, data_path=data_path,
        records=[], ref_ayal_pose=None,
        ayal_teach=False, uri_teach=False,
    )

def enable_teach_ayal(ctx):
    if not ctx.ayal_teach:
        ctx.ayal.control.teachMode()
        ctx.ayal_teach = True

def disable_teach_ayal(ctx):
    if ctx.ayal_teach:
        ctx.ayal.control.endTeachMode()
        ctx.ayal_teach = False
        time.sleep(SETTLE_S)

def enable_teach_uri(ctx):
    if not ctx.uri_teach:
        ctx.uri.control.teachMode()
        ctx.uri_teach = True

def disable_teach_uri(ctx):
    if ctx.uri_teach:
        ctx.uri.control.endTeachMode()
        ctx.uri_teach = False
        time.sleep(SETTLE_S)

def verify_start_state(ctx):
    uri_pose = list(ctx.uri.recieve.getActualTCPPose())
    ayal_pose = list(ctx.ayal.recieve.getActualTCPPose())
    ctx.emit(f"current uri:   {fmt(uri_pose)}")
    ctx.emit(f"expected uri:  {fmt(START_URI)}")
    ctx.emit(f"current ayal:  {fmt(ayal_pose)}")
    ctx.emit(f"expected ayal: {fmt(START_AYAL)}")
    # input("verify start state, then press enter to continue... ")

def find_new_location(ctx):
    enable_teach_uri(ctx)
    enable_teach_ayal(ctx)
    input("both arms in teach mode — find a new location, release, press enter... ")
    disable_teach_ayal(ctx)
    disable_teach_uri(ctx)
    new_uri = list(ctx.uri.recieve.getActualTCPPose())
    new_ayal = list(ctx.ayal.recieve.getActualTCPPose())
    ctx.emit(f"new uri:  {fmt(new_uri)}")
    ctx.emit(f"new ayal: {fmt(new_ayal)}")
    ctx.ref_ayal_pose = new_ayal

def zero_ayal_ft(ctx):
    ok = ctx.ayal.control.zeroFtSensor()
    time.sleep(SETTLE_S)
    ctx.emit(f"zeroFtSensor -> {ok}")

def record_step(ctx, step_idx):
    pose = list(ctx.ayal.recieve.getActualTCPPose())
    delta = [pose[j] - ctx.ref_ayal_pose[j] for j in range(6)]
    wrench_base = avg_wrench(ctx.ayal)
    wrench_tcp = wrench_to_tcp(wrench_base, pose)
    uri_pose = list(ctx.uri.recieve.getActualTCPPose())

    ctx.records.append({
        "step": step_idx,
        "ayal_pose": pose,
        "ayal_delta": delta,
        "ayal_wrench_base": wrench_base,
        "ayal_wrench_tcp": wrench_tcp,
        "uri_pose": uri_pose,
    })

    ctx.emit(f"step {step_idx}")
    ctx.emit(f"  ayal_delta:      {fmt(delta)}")
    ctx.emit(f"  ayal_wrench_tcp: {fmt(wrench_tcp)}")

def step_teach(ctx, step_idx):
    enable_teach_ayal(ctx)
    response = input(f"move ayal, release, press enter (step {step_idx}) or 'q' to stop: ")
    disable_teach_ayal(ctx)
    if response.strip().lower() == "q":
        return None
    record_step(ctx, step_idx)
    return step_idx + 1

def bounded_noise(avg, dev, bound_max):
    """Returns Gaussian noise strictly clamped to a maximum +/- boundary."""
    noise = random.gauss(avg, dev)
    return max(-bound_max, min(bound_max, noise))

def step_auto_back_z(ctx, step_idx, rot_avg, rot_dev):
    # response = input(f"press enter to advance {BACK_Z_STEP * 1000:.1f}mm backward (step {step_idx}) or 'q' to stop: ")
    # if response.strip().lower() == "q":
    #     return None
        
    # 1. Use the ORIGINAL reference pose to prevent compounding drift
    ref_pose = ctx.ref_ayal_pose
    z_axis_base = tcp_z_in_base(ref_pose)
    
    # 2. Calculate the exact distance from the start for this step
    total_back_dist = step_idx * BACK_Z_STEP
    
    # 3. Create the nominal target perfectly aligned to the original -Z axis
    target = list(ref_pose)
    for axis in range(3):
        # Subtract the translation along the Z vector
        target[axis] -= total_back_dist * z_axis_base[axis]
        
        # Add BOUNDED translation noise
        target[axis] += bounded_noise(TRANS_AVG, TRANS_DEV, TRANS_BOUND)

    # 4. Add BOUNDED rotation noise to the original orientation
    target[3] += bounded_noise(rot_avg, rot_dev, ROT_BOUND)
    target[4] += bounded_noise(rot_avg, rot_dev, ROT_BOUND)
    target[5] += bounded_noise(rot_avg, rot_dev, ROT_BOUND)
    
    # 5. Execute as a single, smooth linear movement
    ctx.ayal.control.moveL(target, AUTO_SPEED, AUTO_ACCEL, False)
    time.sleep(SETTLE_S)
    
    record_step(ctx, step_idx)
    
    return step_idx + 1

def run_record_loop(ctx):
    record_step(ctx, 0)
    step_idx = 1
    
    # Wrap step_auto_back_z to pass the global parameters
    if AUTO:
        step_fn = lambda c, idx: step_auto_back_z(c, idx, ROT_AVG, ROT_DEV)
        ctx.emit(f"strategy: auto_back_z (with rot avg: {ROT_AVG}, dev: {ROT_DEV})")
    else:
        step_fn = step_teach
        ctx.emit("strategy: teach")
        
    for i in range(25):  # 25 mm contraption limit at 1mm steps
        next_idx = step_fn(ctx, step_idx)
        if next_idx is None:
            break
        step_idx = next_idx

def save_dataset(ctx):
    with open(ctx.data_path, "w") as f:
        json.dump(ctx.records, f, indent=2)
    ctx.emit(f"saved {len(ctx.records)} records to {ctx.data_path}")

def teardown(ctx):
    disable_teach_ayal(ctx)
    disable_teach_uri(ctx)
    ctx.log_file.close()
    ctx.uri.disconnect()
    ctx.ayal.disconnect()

def main():
    ctx = setup_session()
    try:
        find_new_location(ctx)
        verify_start_state(ctx)
        zero_ayal_ft(ctx)
        run_record_loop(ctx)
        save_dataset(ctx)
    finally:
        teardown(ctx)


if __name__ == "__main__":
    main()