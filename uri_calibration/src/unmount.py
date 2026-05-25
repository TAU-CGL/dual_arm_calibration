import time
import uri_if
from uri_calibration.src import utils
import numpy as np
from cProfile import label
from pathlib import Path

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
# REL_POS_10 = [ 0.2154, -0.6870,  0.1385, -1.2072,  2.0441, -1.3963]

# URI_POS_STATIC = [0.3945, -0.4743, 0.5918, 0.9645, 0.5197, -1.0038]
# AYAL_POS_AWAY = [-0.1367, 0.3151, 0.4239, -1.2304, -2.048, -1.0349]
# AYAL_POS_FAR_AWAY = [-0.6168, 0.1359, -0.0694, 0.0396, -3.0058, 0.0929]
AYAL_POS_FINAL = [-0.1612, 0.3302, -0.234, 1.3443, 2.8289, 0.0119]

def teach_mode(uri: uri_if.RMPLAB_Uri):
    uri.control.teachMode()
    input("URI is in teach mode. Position robot, then press Enter... ")
    uri.control.endTeachMode()
    time.sleep(SETTLE_S)

def rel_to_ayal_tcp(rel_pose: list, uri_tcp: list, calib_pose: list):
    """
    Convert a relative pose (AYAL TCP in URI TCP frame) to AYAL TCP in AYAL base frame.
    Inverse of ayal_tcp_in_uri_tcp_frame:
      T_A_TCP = inv(T_U_A) @ T_U_TCP @ T_UTCP_ATCP
    """
    T_U_TCP      = utils.pose_to_T(uri_tcp)    # URI base → URI TCP
    T_U_A        = utils.pose_to_T(calib_pose) # URI base → AYAL base
    T_UTCP_ATCP  = utils.pose_to_T(rel_pose)   # URI TCP  → AYAL TCP
    T_A_TCP = np.linalg.inv(T_U_A) @ T_U_TCP @ T_UTCP_ATCP

    return list(utils.T_to_pose(T_A_TCP))

def move_ayal_in_uri_tcp_frame(uri: uri_if.RMPLAB_Uri, ayal: uri_if.RMPLAB_Uri, calib: dict, rel_pose: list, label: str, axis: str=None, new_val: float=0):
    """Move AYAL TCP to rel_pose expressed in current URI TCP frame."""
    uri_tcp = list(uri.recieve.getActualTCPPose())
    target  = rel_to_ayal_tcp(rel_pose, uri_tcp, calib)
    idx = "xyz".index(axis)
    target[idx] = new_val
    ayal.control.moveL(target, AUTO_SPEED, AUTO_ACCEL, False)
    print(f"  Moved to {label}.")
    
def unmount(uri, ayal):
    calib = utils.load_calibration(uri_if.CALIB_FILE)

    teach_mode(uri)
    move_ayal_in_uri_tcp_frame(uri, ayal, calib, REL_POS_0, "REL_POS_0") # move ayal in front of uri
    move_ayal_in_uri_tcp_frame(uri, ayal, calib, REL_POS_OUT, "REL_POS_OUT") # move out (?)
    ayal.gripper.close(GRIPPER_SPEED, GRIPPER_FORCE)
    uri.gripper.open(GRIPPER_SPEED, GRIPPER_FORCE)
    move_ayal_in_uri_tcp_frame(uri, ayal, calib, REL_POS_IN, "REL_POS_IN")
    move_ayal_in_uri_tcp_frame(uri, ayal, calib, REL_POS_OUT, "REL_POS_OUT")
    uri.gripper.close(GRIPPER_SPEED, GRIPPER_FORCE)
    ayal.gripper.move_and_wait_for_pos(GRIPPER_HALF, GRIPPER_SPEED, GRIPPER_FORCE)
    move_ayal_in_uri_tcp_frame(uri, ayal, calib, REL_POS_4, "REL_POS_4")
    ayal.gripper.close(GRIPPER_SPEED, GRIPPER_FORCE)
    move_ayal_in_uri_tcp_frame(uri, ayal, calib, REL_POS_8, "REL_POS_8")
    move_ayal_in_uri_tcp_frame(uri, ayal, calib, REL_POS_9, "REL_POS_9")
    ayal.control.moveL(AYAL_POS_FINAL, AUTO_SPEED, AUTO_ACCEL)
    ayal.gripper.move_and_wait_for_pos(GRIPPER_HALF, GRIPPER_SPEED, GRIPPER_FORCE)

if __name__ == "__main__":
    uri, ayal = utils.connect_robots()
    unmount(uri, ayal)
