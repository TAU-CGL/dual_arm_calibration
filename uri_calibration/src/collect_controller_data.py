
import json
import os
import random
import time
import uri_if
from uri_calibration.src import utils
import numpy as np
from pathlib import Path
from types import SimpleNamespace
from uri_if.robot_fsm import RobotFSM, DashboardHandler, FaultError, RobotState
from uri_calibration.src.is_valid_pose import is_valid_pose, MIN_MANIPULABILITY_URI

FILE_NAME = Path(__file__).stem
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

"""
Helper Functions
"""
def next_output_paths():
    utils.OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    n = 1
    while True:
        suffix = f"_{n}"
        log = utils.OUTPUT_DIR / f"{FILE_NAME}{suffix}.log"
        data = utils.OUTPUT_DIR / f"{FILE_NAME}{suffix}.json"
        if not log.exists() and not data.exists():
            return log, data
        n += 1

def _noisy_target(ref_pose, displacement_along_local_z, clean=False):
    """ref_pose + displacement * (ref_pose's local Z in base) + bounded noise on all 6 axes."""
    z_axis = utils.tcp_z_in_base(ref_pose)
    target = list(ref_pose)
    for axis in range(3):
        target[axis] += displacement_along_local_z * z_axis[axis]
        if not clean:
            target[axis] += utils.bounded_noise(TRANS_AVG, TRANS_DEV, TRANS_BOUND)
    for axis in range(3, 6):
        if not clean:
            target[axis] += utils.bounded_noise(ROT_AVG, ROT_DEV, ROT_BOUND)
    return target

"""
Main Functions
"""
def setup_session():
    uri = uri_if.RMPLAB_Uri(uri_if.HOST_URI)
    ayal = uri_if.RMPLAB_Uri(uri_if.HOST_AYAL)
    uri.connect(False)
    ayal.connect(False)

    uri_dash = DashboardHandler(uri_if.HOST_URI)
    ayal_dash = DashboardHandler(uri_if.HOST_AYAL)

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

    npz = np.load(utils.REACH_GRID_PATH)
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
        base_to_base= utils.load_calibration(uri_if.CALIB_FILE),
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

def wait_for_start_approval(ctx):
    ctx.uri_mate_pose = list(ctx.uri.recieve.getActualTCPPose())
    if ctx.uri_rxyz_anchor is None:
        ctx.uri_rxyz_anchor = list(ctx.uri_mate_pose[3:6])
        utils.utils.emit(ctx, f"uri rxyz anchor (first pose): {utils.fmt(ctx.uri_rxyz_anchor)}")
    ctx.ayal_mate_pose = list(utils.calculate_mirror_position(ctx.uri_mate_pose, ctx.base_to_base,flip_trans=True))
    ctx.ayal_unmate_pose = utils.T_to_pose(utils.pose_to_T(ctx.ayal_mate_pose) @ utils.translate_matrix(-INSERT_OFFSET_Z))  
    utils.utils.emit(ctx, f"ayal current position: {utils.fmt(ctx.ayal.recieve.getActualTCPPose())}")
    utils.utils.emit(ctx, f"uri  mate: {utils.fmt(ctx.uri_mate_pose)}")
    utils.utils.emit(ctx, f"ayal mate: {utils.fmt(ctx.ayal_mate_pose)}")
    utils.utils.emit(ctx, f"ayal_unmate: {utils.fmt(ctx.ayal_unmate_pose)}")
    # input("place uri with connector.")

def insert_with_noise(ctx, cycle_idx, start_step=0):
    """Step ayal toward mate until TCPs are within INSERT_REL_DISTANCE_TARGET, then
    push forward until z-axis force is felt. Step counts act as safety caps."""
    utils.emit(
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
        backoff_target = utils._shifted_along_local_z(ctx.ayal_unmate_pose, -PULL_BACKOFF_AFTER_TARGET)
        utils._fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, backoff_target, SPEED, ACCEL)
        utils._fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, ctx.ayal_unmate_pose, SPEED, ACCEL)

    reached_distance = False
    exeeded_wrench = False
    count_steps = 0
    for step_idx in range(start_step, INSERT_STEPS):
        # Each step targets mate-pose minus a shrinking back-off — so the last
        # step lands on the mate pose.
        count_steps += 1
        remaining_back = (INSERT_STEPS - step_idx - 1) * INSERT_STEP_SIZE
        target = _noisy_target(ctx.ayal_mate_pose, -remaining_back)
        utils._fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, target, SLOW_SPEED, ACCEL)
        time.sleep(SETTLE_S)
        rec = utils.record_step(ctx, cycle_idx, "insert", step_idx)
        for i in range(6):
            if abs(rec["ayal_wrench_tcp"][i]) > INSERT_FORCE_ABORT:
                utils.emit(ctx, f"  large wrench over {INSERT_FORCE_ABORT:.1f} N detected, aborting insert: {utils.fmt(rec['ayal_wrench_tcp'])}. distance was {rec['rel_distance']:.4f} m")
                exeeded_wrench = True
                break
        if exeeded_wrench:
            break
        if rec["rel_distance"] <= INSERT_REL_DISTANCE_TARGET:
            utils.emit(ctx, f"  reached target rel_d={rec['rel_distance']:.4f} m at step {step_idx}; switching to force-seek")
            reached_distance = True
            break

    if exeeded_wrench:
        utils.utils.emit(ctx, f"  exceeded wrench. straightening then entering force-seek")
        target = _noisy_target(ctx.ayal_mate_pose, -remaining_back, clean=True)  # target the clean mate pose for force-seek
        utils._fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, target, SLOW_SPEED, ACCEL)
        time.sleep(SETTLE_S)
        utils.record_step(ctx, cycle_idx, "straightening", step_idx)
        exeeded_wrench = False
        for i in range(6):
            if abs(rec["ayal_wrench_tcp"][i]) > INSERT_FORCE_ABORT:
                utils.utils.emit(ctx, f"  large wrench over {INSERT_FORCE_ABORT:.1f} N detected, aborting insert: {utils.fmt(rec['ayal_wrench_tcp'])}. distance was {rec['rel_distance']:.4f} m")
                utils.utils.emit(ctx, f"  exceeded wrench during straightening. skipping force-seek and moving to next cycle.")
                exeeded_wrench = True
                return
    if not reached_distance:
        utils.utils.emit(ctx, f"  step cap hit before reaching rel_d target; entering force-seek anyway")

        for k in range(INSERT_FORCE_SEEK_STEPS):
            rec = utils.record_step(ctx, cycle_idx, "insert_seek", k)
            fz = abs(rec["ayal_wrench_tcp"][2])
            if fz >= INSERT_FORCE_CONTACT_N:
                utils.utils.emit(ctx, f"  felt |F_z|={fz:.2f} N at seek step {k}; insert complete")
                return
            cur = list(ctx.ayal.recieve.getActualTCPPose())
            target = _noisy_target(cur, INSERT_FORCE_SEEK_STEP_SIZE)  # +Z in ayal TCP frame
            utils._fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, target, SLOW_SPEED, ACCEL)
            time.sleep(SETTLE_S)
    utils.utils.emit(ctx, f"  force-seek cap hit without contact")

def pull_with_noise(ctx, cycle_idx, start_step=0):
    """Step ayal back along its -Z until TCPs are at least PULL_REL_DISTANCE_TARGET apart,
    then add an extra fixed back-off. Step count acts as a safety cap."""
    utils.emit(
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
        expected_mate = list(utils.calculate_mirror_position(cur_uri, ctx.base_to_base, flip_trans=True))
        ctx.ayal_pull_anchor = list(cur_ayal[:3]) + list(expected_mate[3:6])
        utils.emit(ctx, f"  pull anchor xyz={utils.fmt(ctx.ayal_pull_anchor[:3])} rxyz={utils.fmt(ctx.ayal_pull_anchor[3:])}")
        utils._fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, ctx.ayal_pull_anchor, SLOW_SPEED, ACCEL)
        time.sleep(SETTLE_S)
        utils.record_step(ctx, cycle_idx, "pull_straighten", 0)

    reached_distance = False
    wrench_count = 0
    for step_idx in range(start_step, PULL_STEPS):
        exeeded_wrench = False
        back_dist = (step_idx + 1) * PULL_STEP_SIZE
        target = _noisy_target(ctx.ayal_pull_anchor, -back_dist)
        utils._fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, target, SLOW_SPEED, ACCEL)
        time.sleep(SETTLE_S)
        rec = utils.record_step(ctx, cycle_idx, "pull", step_idx)
        for i in range(6):
            if abs(rec["ayal_wrench_tcp"][i]) > INSERT_FORCE_ABORT:
                utils.emit(ctx, f"  large wrench over {INSERT_FORCE_ABORT:.1f} N detected, aborting insert: {utils.fmt(rec['ayal_wrench_tcp'])}. distance was {rec['rel_distance']:.4f} m")
                exeeded_wrench = True
                break
        if exeeded_wrench:
            utils.emit(ctx, f"  exceeded wrench. straightening then moving to next cycle")
            target = _noisy_target(ctx.ayal_pull_anchor, -back_dist, clean=True)  # target the clean mate pose for straightening
            utils._fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, target, SLOW_SPEED, ACCEL)
            time.sleep(SETTLE_S)
            utils.record_step(ctx, cycle_idx, "straightening", step_idx)
            wrench_count += 1
            if wrench_count >= 2:
                utils.emit(ctx, f"  exceeded wrench during straightening {wrench_count} times. skipping remaining pull steps and moving to next cycle.")
                utils._fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, ctx.ayal_unmate_pose, SLOW_SPEED, ACCEL)
                break
        else:
            wrench_count = 0
    
        if rec["rel_distance"] >= PULL_REL_DISTANCE_TARGET:
            utils.emit(ctx, f"  reached target rel_d={rec['rel_distance']:.4f} m at step {step_idx}")
            reached_distance = True
            break
    if not reached_distance:
        utils.emit(ctx, f"  step cap hit before reaching rel_d target; backing off anyway")

    cur = list(ctx.ayal.recieve.getActualTCPPose())
    backoff_target = utils._shifted_along_local_z(cur, -PULL_BACKOFF_AFTER_TARGET)
    utils.emit(ctx, f"  extra pull-back {PULL_BACKOFF_AFTER_TARGET*1000:.1f} mm along ayal -Z")
    utils._fsm_cmovel(ctx.ayal_fsm, ctx.ayal, ctx.ayal_dash, backoff_target, SLOW_SPEED, ACCEL)
    time.sleep(SETTLE_S)
    utils.record_step(ctx, cycle_idx, "pull_backoff", 0)

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
        ok, has_line = is_valid_pose(ctx.uri, pose, min_m=MIN_MANIPULABILITY_URI)
        if ok and has_line:
            utils.emit(ctx, f"choose_new_uri_location: picked {utils.fmt(pose)} (attempt {attempt + 1})")
            return pose
    utils.emit(ctx, f"choose_new_uri_location: no valid pose after {MAX_LOCATION_TRIES} tries")
    return None

def move_ayal_to_compact(ctx):
    utils.emit(ctx, f"move ayal to compact position")
    # ayal_compact_j = list(AYAL_COMPACT_J)
    ayal_compact_j = ctx.ayal.recieve.getActualQ()  
    ayal_compact_j[0] = ctx.ayal.recieve.getActualQ()[0] + np.radians(25)  # add a bit of noise to the first joint to avoid singularity

    ctx.ayal.control.moveJ(ayal_compact_j, HIGH_SPEED, ACCEL, False)

def move_uri_to(ctx, target_pose):
    if target_pose is None:
        utils.emit(ctx, "move_uri_to: no target, skipping")
        return
    utils.emit(ctx, f"move uri to {utils.fmt(target_pose)}")
    utils._fsm_cmovel(ctx.uri_fsm, ctx.uri, ctx.uri_dash, target_pose, SPEED, ACCEL)
    ctx.uri_mate_pose = list(target_pose)
    new_ayal_mate = utils.calculate_mirror_position(
        P_source=ctx.uri_mate_pose,
        P_BaseT2BaseS=ctx.base_to_base,
        flip_axis="y",
        flip_trans=True,
    )
    ctx.ayal_mate_pose = [float(v) for v in new_ayal_mate]
    utils.emit(ctx, f"  computed ayal mate: {utils.fmt(ctx.ayal_mate_pose)}")

def main():
    ctx = setup_session()

    i = 0
    next_step = "approve"
    while i < N_CYCLES:

        if next_step == "approve":
            wait_for_start_approval(ctx)
            next_step = "insert"

        elif next_step == "insert":
            utils.run_phase_with_recovery(ctx, "insert", insert_with_noise, i)
            next_step = "pull"

        elif next_step == "pull":
            utils.run_phase_with_recovery(ctx, "pull", pull_with_noise, i)
            next_step = "new_loc"
            
        elif next_step == "new_loc":
            target = choose_new_uri_location(ctx)
            if target is None:  
                break
            move_ayal_to_compact(ctx)
            move_uri_to(ctx, target)
            next_step = "approve"
            i += 1
