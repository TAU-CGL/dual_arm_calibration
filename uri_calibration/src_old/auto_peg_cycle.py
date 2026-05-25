"""
i need a script with these steps:
1. wait for user approval that the two robots are in start position (connector is mounted on uri, ayal is nearby).
repeat N times:
2. insert into the connector (with noise) (+record force/torque + relative position)
3. pull out of the connector (with noise) (+record force/torque + relative position)
4. choose new valid location
5. move uri to it
6. meet ayal with uri.

----
implementation notes:

- every move goes through `RobotFSM.move(lambda: cmovel(...))` so each move is
  guarded by the dual-robot safety check AND uses the project's controller
  (smooth deceleration + dashboard-stop polling).
- step (2)/(3): noisy linear stepping along ayal's TCP -Z, recording averaged
  F/T and relative pose at each step. pattern lifted from
  playground/src/record_pose_and_force.py (auto mode).
- step (4): samples from reachability_grid.npz (cells == 1 = reachable + non-
  singular for both robots), then validates with check_pose. orientation is
  whatever the grid was generated with.
- step (5)/(6): when uri moves, ayal's mating pose is recomputed from the
  rigid base-to-base transform captured at start (calculate_ayal_in_uri /
  calculate_mirror_position). meet_ayal does approach-offset, then slow mate.
"""
import json
import os
import random
import smtplib
import sys
import time
from email.message import EmailMessage
from pathlib import Path
from types import SimpleNamespace

import numpy as np

_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_PLAYGROUND_SRC = os.path.join(_RMP_LAB_ROOT, "playground", "src")
for _p in (_RMP_LAB_ROOT, _PLAYGROUND_SRC):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import uri_if  # noqa: E402
from uri_if.robot_fsm import RobotFSM, DashboardHandler, FaultError, RobotState  # noqa: E402
from rmp_controller import cmovel  # noqa: E402
from check_pose import check_pose, MIN_MANIPULABILITY_URI  # noqa: E402
from uri_calibration.src import utils as gemini  # noqa: E402

URI_HOST = "192.168.56.101"
AYAL_HOST = "192.168.57.101"

# Email notification on protective stop. Set these env vars to enable:
#   AUTOPEG_SMTP_USER  — gmail address sending the alert
#   AUTOPEG_SMTP_PASS  — gmail app password (not the account password!)
#                        create one at https://myaccount.google.com/apppasswords
NOTIFY_EMAIL = "yuvarbiv@gmail.com"
SMTP_USER_ENV = "AUTOPEG_SMTP_USER"
SMTP_PASS_ENV = "AUTOPEG_SMTP_PASS"

CALIBRATION = [0.4973, -0.9608, 0.2694, 0.00004, 0.00001, 0.00389]
N_CYCLES = 100
SPEED, ACCEL = 0.5, 1
HIGH_SPEED = 1
SLOW_SPEED = 0.1

INSERT_STEPS = 25
INSERT_STEP_SIZE = 0.001       # 1 mm per step toward mate
INSERT_FORCE_SEEK_STEP_SIZE = INSERT_STEP_SIZE*0.1  # 0.1 mm per step forward during force-seek
INSERT_OFFSET_Z = 0.030       # ayal starts this far back along its -Z
INSERT_REL_DISTANCE_TARGET = 0.0015   # stop noisy stepping when TCPs are within 1.5 mm
INSERT_FORCE_CONTACT_N = 5.0         # |F_z|_tcp considered "contact" during force-seek
INSERT_FORCE_ABORT = 20.0         # |F_z|_tcp considered "contact" during force-seek
INSERT_FORCE_SEEK_STEPS = 15         # safety cap for force-seek phase

PULL_STEPS = 25
PULL_STEP_SIZE = 0.001         # 1 mm per step away from mate
PULL_REL_DISTANCE_TARGET = 0.027     # stop noisy stepping once TCPs are 27 mm apart
PULL_BACKOFF_AFTER_TARGET = 0.020    # then pull a further 20 mm in ayal's -Z

APPROACH_OFFSET_Z = 0.05       # 5 cm standoff before final mate

TRANS_AVG, TRANS_DEV, TRANS_BOUND = 0.0, 0.001, 0.002
ROT_AVG, ROT_DEV, ROT_BOUND = 0.0, 0.02, 0.04

N_FT_SAMPLES = 30
FT_SAMPLE_DELAY_S = 0.002
SETTLE_S = 0.5

MAX_LOCATION_TRIES = 200
URI_LOC_ROT_BOUND = float(np.radians(10.0))  # ±10° on each rotvec component, around the first uri pose
URI_MIN_X = 0.09  # reject reach-grid cells with x < this (units match rg.xs)

AYAL_COMPACT_J = [144.59796, -106.820178, -131.55096, 132.949286, 83.934822, 176.720599]
AYAL_COMPACT_J = tuple(float(x) / 180 * np.pi for x in AYAL_COMPACT_J)

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "output"
BASE_NAME = Path(__file__).stem

# ---- helpers (lifted from record_pose_and_force.py) -------------------------
def fmt(v):
    return "[" + ", ".join(f"{x:.4f}" for x in v) + "]"

def avg_wrench(robot):
    samples = []
    for i in range(N_FT_SAMPLES):
        if i:
            time.sleep(FT_SAMPLE_DELAY_S)
        samples.append(list(robot.recieve.getActualTCPForce())[:6])
    return [sum(s[j] for s in samples) / N_FT_SAMPLES for j in range(6)]

def wrench_to_tcp(wrench_base, pose_base):
    return list(gemini.wrench_trans(wrench_base, pose_base, base_to_tcp=True, include_translation=False))

def tcp_z_in_base(pose):
    R = gemini.rotvec_to_R(pose[3:6])
    return [float(R[0, 2]), float(R[1, 2]), float(R[2, 2])]

def bounded_noise(avg, dev, bound_max):
    n = random.gauss(avg, dev)
    return max(-bound_max, min(bound_max, n))

def next_output_paths():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    n = 1
    while True:
        suffix = f"_{n}"
        log = OUTPUT_DIR / f"{BASE_NAME}{suffix}.log"
        data = OUTPUT_DIR / f"{BASE_NAME}{suffix}.json"
        if not log.exists() and not data.exists():
            return log, data
        n += 1

# ---- session lifecycle ------------------------------------------------------
def setup_session():
    uri = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    uri.connect(False)
    ayal.connect(False)

    uri_dash = DashboardHandler(URI_HOST)
    ayal_dash = DashboardHandler(AYAL_HOST)

    uri_fsm = RobotFSM(uri, uri_dash, ayal_dash, name="uri", peer_name="ayal")
    ayal_fsm = RobotFSM(
        ayal, ayal_dash, uri_dash, name="ayal", peer_name="uri",
        auto_clamp_recover=False,
    )
    uri_fsm.connect()
    ayal_fsm.connect()

    log_path, data_path = next_output_paths()
    log_file = open(log_path, "w")
    print(f"log:  {log_path}")
    print(f"data: {data_path}")

    npz = np.load(REACH_GRID_PATH)
    reach_grid = SimpleNamespace(
        grid=npz["grid"],
        xs=npz["xs"],
        ys=npz["ys"],
        zs=npz["zs"],
        rxyz=npz["rxyz"],
        feasible=np.argwhere(npz["grid"] == 1),
    )
    print(f"reach grid: {reach_grid.grid.shape}, {len(reach_grid.feasible)} feasible cells, rxyz={list(reach_grid.rxyz)}")

    return SimpleNamespace(
        uri=uri,
        ayal=ayal,
        uri_dash=uri_dash,
        ayal_dash=ayal_dash,
        uri_fsm=uri_fsm,
        ayal_fsm=ayal_fsm,
        reach_grid=reach_grid,
        base_to_base=CALIBRATION,
        uri_mate_pose=None,
        ayal_mate_pose=None,
        ayal_unmate_pose=None,
        ayal_in_uri=None,
        ayal_q_prev=None,
        uri_q_prev=None,
        uri_rxyz_anchor=None,
        ayal_pull_anchor=None,
        last_step=None,
        records=[],
        log_file=log_file,
        log_path=log_path,
        data_path=data_path,
    )

def emit(ctx, line):
    print(line)
    ctx.log_file.write(line + "\n")
    ctx.log_file.flush()

def log_only(ctx, line):
    ctx.log_file.write(line + "\n")
    ctx.log_file.flush()

def wait_for_start_approval(ctx):
    ctx.uri_mate_pose = list(ctx.uri.recieve.getActualTCPPose())
    if ctx.uri_rxyz_anchor is None:
        ctx.uri_rxyz_anchor = list(ctx.uri_mate_pose[3:6])
        emit(ctx, f"uri rxyz anchor (first pose): {fmt(ctx.uri_rxyz_anchor)}")
    ctx.ayal_mate_pose = list(gemini.calculate_mirror_position(ctx.uri_mate_pose, ctx.base_to_base,flip_trans=True))
    ctx.ayal_unmate_pose = gemini.T_to_pose(gemini.pose_to_T(ctx.ayal_mate_pose) @ gemini.translate_matrix(-INSERT_OFFSET_Z))  
    emit(ctx, f"ayal current position: {fmt(ctx.ayal.recieve.getActualTCPPose())}")
    emit(ctx, f"uri  mate: {fmt(ctx.uri_mate_pose)}")
    emit(ctx, f"ayal mate: {fmt(ctx.ayal_mate_pose)}")
    emit(ctx, f"ayal_unmate: {fmt(ctx.ayal_unmate_pose)}")
    # input("place uri with connector.")

# ---- recording --------------------------------------------------------------
def ayal_pose_in_uri_base(ayal_pose, base_to_base):
    """Express an AYAL TCP pose (in AYAL base frame) in URI's base frame.

    `base_to_base` is the calibration pose of AYAL's base in URI's base frame.
    """
    T_uri_ayalbase = gemini.pose_to_T(base_to_base)
    T_ayalbase_ayaltcp = gemini.pose_to_T(ayal_pose)
    return gemini.T_to_pose(T_uri_ayalbase @ T_ayalbase_ayaltcp)

def record_step(ctx, cycle_idx, phase, step_idx):
    ctx.last_step = step_idx
    ayal_pose = list(ctx.ayal.recieve.getActualTCPPose())
    uri_pose = list(ctx.uri.recieve.getActualTCPPose())
    ayal_q = list(ctx.ayal.recieve.getActualQ())
    uri_q = list(ctx.uri.recieve.getActualQ())
    delta = [ayal_pose[j] - ctx.ayal_mate_pose[j] for j in range(6)]

    ayal_q_delta = [0.0] * 6 if ctx.ayal_q_prev is None else [ayal_q[j] - ctx.ayal_q_prev[j] for j in range(6)]
    uri_q_delta = [0.0] * 6 if ctx.uri_q_prev is None else [uri_q[j] - ctx.uri_q_prev[j] for j in range(6)]
    ctx.ayal_q_prev = ayal_q
    ctx.uri_q_prev = uri_q

    ayal_wrench_base = avg_wrench(ctx.ayal)
    ayal_wrench_tcp = wrench_to_tcp(ayal_wrench_base, ayal_pose)
    uri_wrench_base = avg_wrench(ctx.uri)
    uri_wrench_tcp = wrench_to_tcp(uri_wrench_base, uri_pose)

    ayal_in_uri = list(ayal_pose_in_uri_base(ayal_pose, ctx.base_to_base))
    rel_delta_uri_base = [ayal_in_uri[j] - uri_pose[j] for j in range(6)]
    rel_distance = float(np.linalg.norm(rel_delta_uri_base[:3]))

    rec = {
        "cycle": cycle_idx,
        "phase": phase,
        "step": step_idx,
        "ayal_pose": ayal_pose,
        "ayal_delta_to_mate": delta,
        "ayal_q": ayal_q,
        "ayal_q_delta": ayal_q_delta,
        "ayal_wrench_base": ayal_wrench_base,
        "ayal_wrench_tcp": ayal_wrench_tcp,
        "uri_pose": uri_pose,
        "uri_q": uri_q,
        "uri_q_delta": uri_q_delta,
        "uri_wrench_base": uri_wrench_base,
        "uri_wrench_tcp": uri_wrench_tcp,
        "ayal_in_uri_base": ayal_in_uri,
        "rel_delta_uri_base": rel_delta_uri_base,
        "rel_distance": rel_distance,
    }
    ctx.records.append(rec)
    tag = f"  [{phase} c{cycle_idx} s{step_idx}]"
    print(tag)
    log_only(
        ctx,
        f"{tag} delta={fmt(delta)} rel_d={rel_distance:.4f}m "
        f"ayal_F/T_tcp={fmt(ayal_wrench_tcp)} uri_F/T_tcp={fmt(uri_wrench_tcp)}",
    )
    return rec

# ---- noisy linear stepping --------------------------------------------------
def _noisy_target(ref_pose, displacement_along_local_z, clean=False):
    """ref_pose + displacement * (ref_pose's local Z in base) + bounded noise on all 6 axes."""
    z_axis = tcp_z_in_base(ref_pose)
    target = list(ref_pose)
    for axis in range(3):
        target[axis] += displacement_along_local_z * z_axis[axis]
        if not clean:
            target[axis] += bounded_noise(TRANS_AVG, TRANS_DEV, TRANS_BOUND)
    for axis in range(3, 6):
        if not clean:
            target[axis] += bounded_noise(ROT_AVG, ROT_DEV, ROT_BOUND)
    return target

def insert_with_noise(ctx, cycle_idx, start_step=0):
    """Step ayal toward mate until TCPs are within INSERT_REL_DISTANCE_TARGET, then
    push forward until z-axis force is felt. Step counts act as safety caps."""
    emit(
        ctx,
        f"[cycle {cycle_idx}] insert: target rel_d <= {INSERT_REL_DISTANCE_TARGET*1000:.1f} mm "
        f"(cap {INSERT_STEPS} steps × {INSERT_STEP_SIZE*1000:.1f} mm), "
        f"then seek |F_z|>={INSERT_FORCE_CONTACT_N:.1f} N (cap {INSERT_FORCE_SEEK_STEPS} steps)"
        + (f" — resuming from step {start_step}" if start_step else ""),
    )

    if start_step == 0:
        ctx.ayal.control.zeroFtSensor()
        ctx.uri.control.zeroFtSensor()
        time.sleep(SETTLE_S)
        
        # Move to the start-of-insert pose (offset back along ayal's -Z from the mate).
        print(f"  moving to insert start pose (offset {PULL_BACKOFF_AFTER_TARGET*1000:.1f} mm back along ayal -Z from mate)")
        backoff_target = _shifted_along_local_z(ctx.ayal_unmate_pose, -PULL_BACKOFF_AFTER_TARGET)
        _fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, backoff_target, SPEED, ACCEL)
        _fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, ctx.ayal_unmate_pose, SPEED, ACCEL)

    reached_distance = False
    exeeded_wrench = False
    count_steps = 0
    for step_idx in range(start_step, INSERT_STEPS):
        # Each step targets mate-pose minus a shrinking back-off — so the last
        # step lands on the mate pose.
        count_steps += 1
        remaining_back = (INSERT_STEPS - step_idx - 1) * INSERT_STEP_SIZE
        target = _noisy_target(ctx.ayal_mate_pose, -remaining_back)
        _fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, target, SLOW_SPEED, ACCEL)
        time.sleep(SETTLE_S)
        rec = record_step(ctx, cycle_idx, "insert", step_idx)
        for i in range(6):
            if abs(rec["ayal_wrench_tcp"][i]) > INSERT_FORCE_ABORT:
                emit(ctx, f"  large wrench over {INSERT_FORCE_ABORT:.1f} N detected, aborting insert: {fmt(rec['ayal_wrench_tcp'])}. distance was {rec['rel_distance']:.4f} m")
                exeeded_wrench = True
                break
        if exeeded_wrench:
            break
        if rec["rel_distance"] <= INSERT_REL_DISTANCE_TARGET:
            emit(ctx, f"  reached target rel_d={rec['rel_distance']:.4f} m at step {step_idx}; switching to force-seek")
            reached_distance = True
            break

    if exeeded_wrench:
        emit(ctx, f"  exceeded wrench. straightening then entering force-seek")
        target = _noisy_target(ctx.ayal_mate_pose, -remaining_back, clean=True)  # target the clean mate pose for force-seek
        _fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, target, SLOW_SPEED, ACCEL)
        time.sleep(SETTLE_S)
        record_step(ctx, cycle_idx, "straightening", step_idx)
        exeeded_wrench = False
        for i in range(6):
            if abs(rec["ayal_wrench_tcp"][i]) > INSERT_FORCE_ABORT:
                emit(ctx, f"  large wrench over {INSERT_FORCE_ABORT:.1f} N detected, aborting insert: {fmt(rec['ayal_wrench_tcp'])}. distance was {rec['rel_distance']:.4f} m")
                emit(ctx, f"  exceeded wrench during straightening. skipping force-seek and moving to next cycle.")
                exeeded_wrench = True
                return
    if not reached_distance:
        emit(ctx, f"  step cap hit before reaching rel_d target; entering force-seek anyway")

        for k in range(INSERT_FORCE_SEEK_STEPS):
            rec = record_step(ctx, cycle_idx, "insert_seek", k)
            fz = abs(rec["ayal_wrench_tcp"][2])
            if fz >= INSERT_FORCE_CONTACT_N:
                emit(ctx, f"  felt |F_z|={fz:.2f} N at seek step {k}; insert complete")
                return
            cur = list(ctx.ayal.recieve.getActualTCPPose())
            target = _noisy_target(cur, INSERT_FORCE_SEEK_STEP_SIZE)  # +Z in ayal TCP frame
            _fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, target, SLOW_SPEED, ACCEL)
            time.sleep(SETTLE_S)
    emit(ctx, f"  force-seek cap hit without contact")

def pull_with_noise(ctx, cycle_idx, start_step=0):
    """Step ayal back along its -Z until TCPs are at least PULL_REL_DISTANCE_TARGET apart,
    then add an extra fixed back-off. Step count acts as a safety cap."""
    emit(
        ctx,
        f"[cycle {cycle_idx}] pull: target rel_d >= {PULL_REL_DISTANCE_TARGET*1000:.1f} mm "
        f"(cap {PULL_STEPS} steps × {PULL_STEP_SIZE*1000:.1f} mm), "
        f"then -Z back-off {PULL_BACKOFF_AFTER_TARGET*1000:.1f} mm"
        + (f" — resuming from step {start_step}" if start_step else ""),
    )

    if start_step == 0:
        # Build the pull anchor: keep ayal's current xyz, but apply the orientation
        # that mirrors uri's current orientation (i.e. the "ideal" mating orientation).
        # This straightens ayal in place so the subsequent -Z stepping pulls along the
        # connector axis even if the insert ended with the peg slightly off-axis.
        cur_ayal = list(ctx.ayal.recieve.getActualTCPPose())
        cur_uri = list(ctx.uri.recieve.getActualTCPPose())
        expected_mate = list(gemini.calculate_mirror_position(cur_uri, ctx.base_to_base, flip_trans=True))
        ctx.ayal_pull_anchor = list(cur_ayal[:3]) + list(expected_mate[3:6])
        emit(ctx, f"  pull anchor xyz={fmt(ctx.ayal_pull_anchor[:3])} rxyz={fmt(ctx.ayal_pull_anchor[3:])}")
        _fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, ctx.ayal_pull_anchor, SLOW_SPEED, ACCEL)
        time.sleep(SETTLE_S)
        record_step(ctx, cycle_idx, "pull_straighten", 0)

    reached_distance = False
    wrench_count = 0
    for step_idx in range(start_step, PULL_STEPS):
        exeeded_wrench = False
        back_dist = (step_idx + 1) * PULL_STEP_SIZE
        target = _noisy_target(ctx.ayal_pull_anchor, -back_dist)
        _fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, target, SLOW_SPEED, ACCEL)
        time.sleep(SETTLE_S)
        rec = record_step(ctx, cycle_idx, "pull", step_idx)
        for i in range(6):
            if abs(rec["ayal_wrench_tcp"][i]) > INSERT_FORCE_ABORT:
                emit(ctx, f"  large wrench over {INSERT_FORCE_ABORT:.1f} N detected, aborting insert: {fmt(rec['ayal_wrench_tcp'])}. distance was {rec['rel_distance']:.4f} m")
                exeeded_wrench = True
                break
        if exeeded_wrench:
            emit(ctx, f"  exceeded wrench. straightening then moving to next cycle")
            target = _noisy_target(ctx.ayal_pull_anchor, -back_dist, clean=True)  # target the clean mate pose for straightening
            _fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, target, SLOW_SPEED, ACCEL)
            time.sleep(SETTLE_S)
            record_step(ctx, cycle_idx, "straightening", step_idx)
            wrench_count += 1
            if wrench_count >= 2:
                emit(ctx, f"  exceeded wrench during straightening {wrench_count} times. skipping remaining pull steps and moving to next cycle.")
                _fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, ctx.ayal_unmate_pose, SLOW_SPEED, ACCEL)
                break
        else:
            wrench_count = 0
    
        if rec["rel_distance"] >= PULL_REL_DISTANCE_TARGET:
            emit(ctx, f"  reached target rel_d={rec['rel_distance']:.4f} m at step {step_idx}")
            reached_distance = True
            break
    if not reached_distance:
        emit(ctx, f"  step cap hit before reaching rel_d target; backing off anyway")

    cur = list(ctx.ayal.recieve.getActualTCPPose())
    backoff_target = _shifted_along_local_z(cur, -PULL_BACKOFF_AFTER_TARGET)
    emit(ctx, f"  extra pull-back {PULL_BACKOFF_AFTER_TARGET*1000:.1f} mm along ayal -Z")
    _fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, backoff_target, SLOW_SPEED, ACCEL)
    time.sleep(SETTLE_S)
    record_step(ctx, cycle_idx, "pull_backoff", 0)

# ---- location selection -----------------------------------------------------
def choose_new_uri_location(ctx):
    rg = ctx.reach_grid
    anchor_rxyz = list(ctx.uri_rxyz_anchor) if ctx.uri_rxyz_anchor is not None else list(rg.rxyz)

    for attempt in range(MAX_LOCATION_TRIES):
        ix, iy, iz = rg.feasible[random.randrange(len(rg.feasible))]
        x = float(rg.xs[ix])
        if x < URI_MIN_X:
            continue
        rxyz = [
            anchor_rxyz[k] + random.uniform(-URI_LOC_ROT_BOUND, URI_LOC_ROT_BOUND)
            for k in range(3)
        ]
        pose = [x, float(rg.ys[iy]), float(rg.zs[iz])] + rxyz
        ok, has_line = check_pose(ctx.uri, pose, min_m=MIN_MANIPULABILITY_URI)
        if ok and has_line:
            emit(ctx, f"choose_new_uri_location: picked {fmt(pose)} (attempt {attempt + 1})")
            return pose
    emit(ctx, f"choose_new_uri_location: no valid pose after {MAX_LOCATION_TRIES} tries")
    return None

# ---- moves through fsm + cmovel ---------------------------------------------
FORCE_STOP_THRESHOLD = 30.0    # N — abort move if |F| exceeds this
TORQUE_STOP_THRESHOLD = 5.0    # Nm — abort move if |M| exceeds this

def _fsm_cmovel(fsm, robot, dash, target, speed, accel,
                pose_tolerance=0.0001,
                force_threshold=FORCE_STOP_THRESHOLD,
                torque_threshold=TORQUE_STOP_THRESHOLD):
    fsm.move(lambda: cmovel(
        robot.control, robot.recieve, target, speed, accel, dash,
        pose_tolerance=pose_tolerance,
        force_threshold=force_threshold,
        torque_threshold=torque_threshold,
    ))

def _shifted_along_local_z(pose, distance):
    """Return `pose` translated by `distance * (local Z axis of pose, expressed in base)`."""
    z = tcp_z_in_base(pose)
    out = list(pose)
    for axis in range(3):
        out[axis] += distance * z[axis]
    return out

def move_uri_to(ctx, target_pose):
    if target_pose is None:
        emit(ctx, "move_uri_to: no target, skipping")
        return
    emit(ctx, f"move uri to {fmt(target_pose)}")
    _fsm_cmovel(ctx.uri_fsm, ctx.uri, ctx.uri_dash, target_pose, SPEED, ACCEL)
    ctx.uri_mate_pose = list(target_pose)
    new_ayal_mate = gemini.calculate_mirror_position(
        P_source=ctx.uri_mate_pose,
        P_BaseT2BaseS=ctx.base_to_base,
        flip_axis="y",
        flip_trans=True,
    )
    ctx.ayal_mate_pose = [float(v) for v in new_ayal_mate]
    emit(ctx, f"  computed ayal mate: {fmt(ctx.ayal_mate_pose)}")

def move_ayal_to_compact(ctx):
    emit(ctx, f"move ayal to compact position")
    # ayal_compact_j = list(AYAL_COMPACT_J)
    ayal_compact_j = ctx.ayal.recieve.getActualQ()  
    ayal_compact_j[0] = ctx.ayal.recieve.getActualQ()[0] + np.radians(25)  # add a bit of noise to the first joint to avoid singularity

    ctx.ayal.control.moveJ(ayal_compact_j, HIGH_SPEED, ACCEL, False)

def meet_ayal_with_uri(ctx):
    if ctx.ayal_mate_pose is None:
        emit(ctx, "meet_ayal_with_uri: no target, skipping")
        return
    ctx.ayal_unmate_pose = _shifted_along_local_z(ctx.ayal_mate_pose, -APPROACH_OFFSET_Z)
    emit(ctx, f"meet ayal: unmate {fmt(ctx.ayal_unmate_pose)}")
    _fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, ctx.ayal_unmate_pose, SPEED, ACCEL)
    # emit(ctx, f"meet ayal: mate    {fmt(ctx.ayal_mate_pose)}")
    # _fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, ctx.ayal_mate_pose, SLOW_SPEED, ACCEL)

# ---- IO ---------------------------------------------------------------------
def save_dataset(ctx):
    with open(ctx.data_path, "w") as f:
        json.dump(ctx.records, f, indent=2)
    emit(ctx, f"saved {len(ctx.records)} records to {ctx.data_path}")

def teardown(ctx):
    if getattr(ctx, "log_file", None) is not None:
        try:
            ctx.log_file.close()
        except Exception:
            pass
    if ctx.uri is not None:
        ctx.uri.disconnect()
    if ctx.ayal is not None:
        ctx.ayal.disconnect()
    if ctx.uri_dash is not None:
        ctx.uri_dash.disconnect()
    if ctx.ayal_dash is not None:
        ctx.ayal_dash.disconnect()

def check_delta():
    """Print the calibration-residual delta between AYAL's actual pose and the
    mating pose calculated from URI's actual pose via CALIBRATION.

    Usage: place both robots manually so the pegs are mating, then run
        python dual_arm_peg/src/auto_peg_cycle.py check_delta
    """
    uri = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    uri.connect(False)
    ayal.connect(False)
    try:
        uri_pose = list(uri.recieve.getActualTCPPose())
        ayal_pose = list(ayal.recieve.getActualTCPPose())
        ayal_mate_pose = list(
            gemini.calculate_mirror_position(uri_pose, CALIBRATION, flip_trans=True)
        )
        delta = [ayal_pose[j] - ayal_mate_pose[j] for j in range(6)]
        trans_err_mm = float(np.linalg.norm(delta[:3])) * 1000.0
        rot_err_deg = float(np.degrees(np.linalg.norm(delta[3:])))
        print(f"uri pose:         {fmt(uri_pose)}")
        print(f"ayal pose:        {fmt(ayal_pose)}")
        print(f"ayal mate (calc): {fmt(ayal_mate_pose)}")
        print(f"delta (ayal-mate):{fmt(delta)}")
        print(f"|trans| = {trans_err_mm:.2f} mm   |rotvec| = {rot_err_deg:.2f} deg")
    finally:
        uri.disconnect()
        ayal.disconnect()

def send_protective_stop_email(phase_name, cycle_idx, step):
    """Best-effort Gmail SMTP notification. Silently skipped if env vars not set."""
    user = os.environ.get(SMTP_USER_ENV)
    password = os.environ.get(SMTP_PASS_ENV)
    if not user or not password:
        print(f"  email skipped: set ${SMTP_USER_ENV} + ${SMTP_PASS_ENV} to enable")
        return
    try:
        msg = EmailMessage()
        msg["From"] = user
        msg["To"] = NOTIFY_EMAIL
        msg["Subject"] = f"[auto_peg_cycle] AYAL protective stop — {phase_name} c{cycle_idx} s{step}"
        msg.set_content(
            f"AYAL hit a protective stop during {phase_name}.\n"
            f"  cycle: {cycle_idx}\n"
            f"  step:  {step}\n"
            f"  time:  {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n"
            f"Waiting for [c]ontinue / [f]reedrive / [q]uit on the host."
        )
        with smtplib.SMTP_SSL("smtp.gmail.com", 465, timeout=10) as s:
            s.login(user, password)
            s.send_message(msg)
        print(f"  email sent to {NOTIFY_EMAIL}")
    except Exception as e:
        print(f"  email failed: {e}")

def _ayal_unlock_and_resume(ctx):
    """Unlock AYAL's protective stop, close popup, wait until ready, reupload script."""
    try:
        ctx.ayal_dash.unlock_protective_stop()
    except Exception as e:
        print(f"  ayal unlock failed: {e}")
        return False
    try:
        ctx.ayal_dash.close_safety_popup()
    except Exception:
        pass
    ctx.ayal_dash.wait_until_ready(timeout=10)
    time.sleep(1.0)
    try:
        ctx.ayal.control.reuploadScript()
    except Exception as e:
        print(f"  ayal reuploadScript failed: {e}")
        return False
    # Give the freshly-reuploaded RTDE script time to actually start running on
    # the controller before we try to send any further control commands.
    time.sleep(2.0)
    return True

def _enter_teach_mode(robot, name, retries=3, delay=0.8):
    for attempt in range(retries):
        try:
            robot.control.teachMode()
            print(f"  {name}: freedrive ON")
            return True
        except Exception as e:
            print(f"  {name}: teachMode attempt {attempt + 1}/{retries} failed: {e}")
            time.sleep(delay)
    print(f"  {name}: freedrive FAILED — manually enable on pendant if needed")
    return False

def _exit_teach_mode(robot, name):
    try:
        robot.control.endTeachMode()
        print(f"  {name}: freedrive OFF")
    except Exception as e:
        print(f"  {name}: endTeachMode failed: {e}")

def _both_freedrive(ctx):
    """Put both arms in freedrive/teach mode until the user presses Enter."""
    # Settle: RTDE control scripts may still be coming online after any unlock/reupload.
    time.sleep(1.0)
    _enter_teach_mode(ctx.uri, "uri")
    time.sleep(0.5)
    _enter_teach_mode(ctx.ayal, "ayal")

    input("  press Enter to exit freedrive...")

    _exit_teach_mode(ctx.uri, "uri")
    time.sleep(0.3)
    _exit_teach_mode(ctx.ayal, "ayal")
    time.sleep(0.3)

class RecoveryAction(Exception):
    """Signal raised by the recovery handler to redirect the cycle state machine."""
    def __init__(self, action):
        super().__init__(action)
        self.action = action  # one of: "insert", "pull", "new_loc", "quit"

def handle_protective_stop(ctx, phase_name, cycle_idx):
    """Unlock + freedrive both arms, then ask the user which macro-step to resume at.
    Returns one of 'insert', 'pull', 'new_loc', 'quit'."""
    last = ctx.last_step if ctx.last_step is not None else 0
    print(f"\n[recovery] {phase_name} c{cycle_idx} stopped at/near step {last}.")
    send_protective_stop_email(phase_name, cycle_idx, last)
    if not _ayal_unlock_and_resume(ctx):
        print("  unlock failed; quitting")
        return "quit"
    _both_freedrive(ctx)
    while True:
        choice = input("  resume with [i]nsert/peg / [p]ull / [n]ew location / [q]uit: ").strip().lower()
        if choice in ("i", "insert", "peg", "e"):
            return "insert"
        if choice in ("p", "pull"):
            return "pull"
        if choice in ("n", "new", "new_loc"):
            return "new_loc"
        if choice in ("q", "quit"):
            return "quit"
        print("  unknown choice")

def run_phase_with_recovery(ctx, phase_name, phase_fn, cycle_idx):
    """Run a single phase. On AYAL protective stop, prompt the user and raise
    RecoveryAction with the chosen jump target."""
    try:
        phase_fn(ctx, cycle_idx, start_step=0)
    except FaultError as e:
        if e.state != RobotState.PROTECTIVE_STOP:
            raise
        action = handle_protective_stop(ctx, phase_name, cycle_idx)
        raise RecoveryAction(action)

def main():
    ctx = setup_session()
    try:
        i = 0
        next_step = "approve"
        while i < N_CYCLES:
            try:
                if next_step == "approve":
                    wait_for_start_approval(ctx)
                    next_step = "insert"
                elif next_step == "insert":
                    run_phase_with_recovery(ctx, "insert", insert_with_noise, i)
                    next_step = "pull"
                elif next_step == "pull":
                    run_phase_with_recovery(ctx, "pull", pull_with_noise, i)
                    next_step = "new_loc"
                elif next_step == "new_loc":
                    target = choose_new_uri_location(ctx)
                    if target is None:
                        emit(ctx, f"[cycle {i}] no new location available — stopping early")
                        break
                    move_ayal_to_compact(ctx)
                    move_uri_to(ctx, target)
                    i += 1
                    next_step = "approve"
            except RecoveryAction as ra:
                if ra.action == "quit":
                    raise SystemExit("user requested quit")
                emit(ctx, f"[recovery] jumping to '{ra.action}' (cycle {i})")
                # If user picks insert or pull, re-read uri's pose first — they may
                # have moved it during freedrive, and the mating math depends on it.
                if ra.action in ("insert", "pull"):
                    wait_for_start_approval(ctx)
                next_step = ra.action
    finally:
        try:
            save_dataset(ctx)
        except Exception as e:
            print(f"save_dataset failed: {e}")
        teardown(ctx)

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "check_delta":
        check_delta()
    else:
        main()
