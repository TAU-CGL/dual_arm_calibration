"""Hardware backend: UR RTDE + Robotiq gripper (requires ``ur_rtde``)."""

import rtde_receive
import rtde_control

from .constants import GRIPPER_PORT, HOST
from .robotiq_gripper import RobotiqGripper


class RealRMPLAB_Uri:
    """Direct RTDE + gripper interface (same behavior as upstream ``rmplab_uri``)."""

    def __init__(self, host=None):
        if host is None:
            host = HOST
        self.host = host
        self.control = None
        self.recieve = None
        self.gripper = RobotiqGripper()

    def connect(self, gripper_calibrate=True):
        if self.control is None:
            self.control = rtde_control.RTDEControlInterface(self.host)
        else:
            self.control.reconnect()

        if self.recieve is None:
            self.recieve = rtde_receive.RTDEReceiveInterface(self.host)
        else:
            self.recieve.reconnect()
        self.gripper.connect(self.host, GRIPPER_PORT)
        self.gripper.activate(gripper_calibrate)

    def disconnect(self):
        if self.control is not None:
            self.control.disconnect()
        if self.recieve is not None:
            self.recieve.disconnect()
        self.gripper.disconnect()

    def is_connected(self):
        if self.control is None:
            return False
        return self.control.isConnected()
