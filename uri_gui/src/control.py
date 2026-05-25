import uri_if
import uri_gui.src.config as _cfg

from uri_gui.src.utils import *
from uri_gui.src.config import *


def toggle_connect(uri: uri_if.RMPLAB_Uri, calibrate=True):
    if uri.is_connected():
        print("in control: Disconnecting...")
        uri.disconnect()
    else:
        print("in control: Connecting...")
        uri.connect(calibrate)


# @requires_connection
def toggle_teachmode(uri: uri_if.RMPLAB_Uri):
    if uri.teachmode:
        uri.control.endTeachMode()
        uri.teachmode = False
    else:
        uri.control.teachMode()
        uri.teachmode = True

# @requires_connection
def get_tcp_pose(uri: uri_if.RMPLAB_Uri):
    x, y, z, rx, ry, rz = uri.recieve.getActualTCPPose()
    return x, y, z, rx, ry, rz

# @requires_connection
def tcp_movej(uri: uri_if.RMPLAB_Uri, x, y, z, rx, ry, rz):
    uri.control.moveJ_IK([x, y, z, rx, ry, rz], _cfg.DEFAULT_SPEED, _cfg.DEFAULT_ACCELERATION, False)

# @requires_connection
def tcp_movel(uri: uri_if.RMPLAB_Uri, x, y, z, rx, ry, rz):
    uri.control.moveL([x, y, z, rx, ry, rz], _cfg.DEFAULT_SPEED, _cfg.DEFAULT_ACCELERATION, False)

# @requires_connection
def get_q_pose(uri: uri_if.RMPLAB_Uri):
    q_pose = np.array(uri.recieve.getActualQ()) * 180 / np.pi
    base, shoulder, elbow, wrist1, wrist2, wrist3 = q_pose
    return base, shoulder, elbow, wrist1, wrist2, wrist3

# @requires_connection
def q_movej(uri: uri_if.RMPLAB_Uri, base, shoulder, elbow, wrist1, wrist2, wrist3):
    uri.control.moveJ([base, shoulder, elbow, wrist1, wrist2, wrist3], _cfg.DEFAULT_SPEED, _cfg.DEFAULT_ACCELERATION, False)

# @requires_connection
def q_movel(uri: uri_if.RMPLAB_Uri, base, shoulder, elbow, wrist1, wrist2, wrist3):
    uri.control.moveL_FK([base, shoulder, elbow, wrist1, wrist2, wrist3], _cfg.DEFAULT_SPEED, _cfg.DEFAULT_ACCELERATION, False)

# @requires_connection
def get_gripper_pos(uri: uri_if.RMPLAB_Uri):
    return uri.gripper.get_current_position()

# @requires_connection
def move_gripper(uri:uri_if.RMPLAB_Uri, pos, speed, force):
    uri.gripper.move_and_wait_for_pos(pos, speed, force)

# @requires_connection
def close_gripper(uri:uri_if.RMPLAB_Uri, speed, force):
    uri.gripper.close(speed, force)

# @requires_connection
def open_gripper(uri:uri_if.RMPLAB_Uri, speed, force):
    uri.gripper.open(speed, force)