"""
Calibration module for RMPLAB URIs. full process.
"""

from uri_calibration.src import utils
import uri_if
from argparse import Namespace
from types import SimpleNamespace
from uri_calibration.src.unmount import unmount
from uri_calibration.src.pnp import run_dual_robot_pnp
from uri_calibration.src.solve import mean_measurements
from uri_calibration.src.align import run_find_center_by_grad, run_find_center_by_value, run_find_center_direct
from uri_calibration.src.collect_controller_data import choose_new_uri_location, move_ayal_to_compact, move_uri_to
from uri_calibration.src.reach_grid import dual_reach_grid_const_rot, dual_reach_grid_sweep_rot
from uri_calibration.src.connect import (
    connect,
    MAX_STEPS, Z_STEP, FORCE_LIMIT,
    APPROACH_STOP_DIST, SLOW_STEP_SIZE, SLOW_FORCE_THRESH, SLOW_MAX_STEPS,
)

"""
uri - passive, ayal - active
"""
def dual_reach_grid_wrapper(uri: uri_if.RMPLAB_Uri, ayal: uri_if.RMPLAB_Uri, grid_type="const_rot"):
    args = Namespace(
        x=(-0.5, 0.5),
        y=(-1.0, 0.0),
        z=(0.0, 1.0),
        rxyz=(2.22, -2.22, 0.0),
        step=0.04,
        ayal_in_uri=None,           # falls back to calibration.json
        out="reach_grid.npz",
        timing_samples=10,
        yes=True,                   # skip confirmation prompt
    )
    if grid_type == "const_rot":
        dual_reach_grid_const_rot(args, uri, ayal)
    elif grid_type == "sweep_rot":
        dual_reach_grid_sweep_rot(args, uri, ayal)
    else:
        raise ValueError(f"Unknown grid type: {grid_type}")

def mount(uri: uri_if.RMPLAB_Uri):
    pass

def approach(ayal: uri_if.RMPLAB_Uri):
    pass

def connect_wrapper(uri: uri_if.RMPLAB_Uri, ayal: uri_if.RMPLAB_Uri):
    args = Namespace(
        steps=MAX_STEPS,
        step_size=Z_STEP,
        force_limit=FORCE_LIMIT,
        approach_stop=APPROACH_STOP_DIST,   # break out of approach when |ayal_xyz - mate_xyz| <= this (m)
        slow_step=SLOW_STEP_SIZE,           # step size during slow contact seek (m)
        slow_force_thresh=SLOW_FORCE_THRESH, # |dF_z| change (N) on either robot that ends the slow seek
        slow_max_steps=SLOW_MAX_STEPS,
    )
    connect(uri, ayal, args)

def align(uri: uri_if.RMPLAB_Uri, ayal: uri_if.RMPLAB_Uri, method="grad"):
    if method == "grad":
        run_find_center_by_grad(uri, ayal)
    elif method == "value":
        run_find_center_by_value(uri, ayal)
    elif method == "direct":
        run_find_center_direct(uri, ayal)
    else:
        raise ValueError(f"Unknown alignment method: {method}")

def sample(ctx, uri: uri_if.RMPLAB_Uri, ayal: uri_if.RMPLAB_Uri, cycle_idx: int, step_idx: int):
    ctx.uri_mate_pose = list(ctx.uri.recieve.getActualTCPPose())
    ctx.ayal_mate_pose = list(utils.calculate_mirror_position(ctx.uri_mate_pose, ctx.base_to_base,flip_trans=True))
    ctx.ayal = ayal
    ctx.uri = uri
    utils.record_step(ctx, cycle_idx, "insert", step_idx)

def move(ctx, uri: uri_if.RMPLAB_Uri, ayal: uri_if.RMPLAB_Uri):
    target = choose_new_uri_location(ctx)
    if target is None:  
        return False
    move_ayal_to_compact(ctx)
    move_uri_to(ctx, target)

def sample_loop(ctx, uri: uri_if.RMPLAB_Uri, ayal: uri_if.RMPLAB_Uri, nof_iterations=100):
    align(uri, ayal, method="direct")
    sample(ctx, uri, ayal, 0, 0)
    for i in range(1, nof_iterations+1):
        move(ctx, uri, ayal)
        align(uri, ayal, method="direct")
        sample(ctx, uri, ayal, i, 0)

def parse_samples_for_solver(samples):
    # TODO convert raw sample data to the format expected by the solver
    pass

def solve(samples):
    parse_samples_for_solver(samples)
    mean_measurements(samples)

def verify(uri: uri_if.RMPLAB_Uri, ayal: uri_if.RMPLAB_Uri):
    run_dual_robot_pnp(uri, ayal)

def set_ctx(uri: uri_if.RMPLAB_Uri, ayal: uri_if.RMPLAB_Uri):
    return SimpleNamespace(
        uri=uri,
        ayal=ayal,
        # uri_dash=uri_dash,
        # ayal_dash=ayal_dash,
        # uri_fsm=uri_fsm,
        # ayal_fsm=ayal_fsm,
        # reach_grid=reach_grid,
        # base_to_base=CALIBRATION,
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
        # log_file=log_file,
        # log_path=log_path,
        # data_path=data_path,
    )

def full_calibration(uri: uri_if.RMPLAB_Uri, ayal: uri_if.RMPLAB_Uri, nof_iterations=100):
    ctx = set_ctx(uri, ayal)
    input("before dual_reach_grid_wrapper, press Enter")
    dual_reach_grid_wrapper(uri, ayal)
    input("before mount, press Enter")
    mount(uri)
    input("before approach, press Enter")
    approach(ayal)
    input("before connect, press Enter")
    connect_wrapper(uri, ayal)
    input("before sample_loop, press Enter")
    samples = sample_loop(ctx, uri, ayal, nof_iterations)
    input("before solve, press Enter")
    solve(samples)
    input("before unmount, press Enter")
    unmount(uri, ayal)
    input("before verify, press Enter")
    verify(uri, ayal)

def main():
    uri, ayal = utils.connect_robots()
    full_calibration(uri, ayal)

def run(backend):
    """Entry point for uri_gui_pyqt5/src/main.py script mode.

    The sim has already registered its robots with uri_if by the time
    we get here, so RMPLAB_Uri(...).connect() resolves to sim.
    """
    del backend  # uri_if dispatches to sim/real on its own
    uri = uri_if.RMPLAB_Uri(uri_if.HOST_URI);   uri.connect(False)
    ayal = uri_if.RMPLAB_Uri(uri_if.HOST_AYAL); ayal.connect(False)
    full_calibration(uri, ayal)

if __name__ == "__main__":
    main()