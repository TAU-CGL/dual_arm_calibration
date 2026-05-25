"""
PyBullet simulation backend for ``rmplab_uri``.

Register robots via ``sim_uri.register_robot`` (re-exported from the package root).
"""

import warnings
import math
import threading
import queue
import numpy as np
import pybullet as p

try:
    import roboticstoolbox as _rtb
    from spatialmath import SE3 as _SE3
    try:
        _UR5E_MODEL = _rtb.models.UR5e()
    except AttributeError:
        _UR5E_MODEL = _rtb.models.UR5()
except ImportError:
    _UR5E_MODEL = None
    _SE3 = None

# ---------------------------------------------------------------------------
# Global registry: host_string -> (pybullet_robot, env)
# ---------------------------------------------------------------------------
_robot_registry = {}
_gripper_lengths = {}

MOVEJ_MODES = ["Teleport Interp", "PD Interp", "Pure Physics"]
_movej_mode = "Teleport Interp"

_physics_params = {
    "force_multiplier": 50,
    "position_gain": 1.0,
    "velocity_gain": 2.0,
    "converge_threshold": 0.01,
    "max_converge_steps": 4800,
}

_joint_zero_offsets = {}

def register_robot(host, robot, env, gripper_length=0.0, joint_zero_offsets=None):
    _robot_registry[host] = (robot, env)
    _gripper_lengths[host] = gripper_length
    _joint_zero_offsets[host] = joint_zero_offsets or [0.0] * 6

def set_movej_mode(mode):
    global _movej_mode
    if mode in MOVEJ_MODES:
        _movej_mode = mode
        print(f"[rmplab_uri] moveJ mode set to: {mode}")

def get_movej_mode():
    return _movej_mode

def set_physics_params(params):
    _physics_params.update(params)
    print(f"[rmplab_uri] Physics params updated: {_physics_params}")

def get_physics_params():
    return dict(_physics_params)

# ---------------------------------------------------------------------------
# Orientation helpers
# ---------------------------------------------------------------------------

def _quat_to_rotvec(quat):
    """PyBullet quaternion (x,y,z,w) -> UR-style rotation vector."""
    x, y, z, w = quat
    w = max(-1.0, min(1.0, w))
    angle = 2.0 * math.acos(abs(w))
    if angle < 1e-10:
        return (0.0, 0.0, 0.0)
    s = math.sin(angle / 2.0)
    sign = 1.0 if w >= 0 else -1.0
    ax = sign * x / s
    ay = sign * y / s
    az = sign * z / s
    return (ax * angle, ay * angle, az * angle)

def _rotvec_to_quat(rx, ry, rz):
    """UR-style rotation vector -> PyBullet quaternion (x,y,z,w)."""
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-10:
        return (0.0, 0.0, 0.0, 1.0)
    ax, ay, az = rx / angle, ry / angle, rz / angle
    s = math.sin(angle / 2.0)
    return (ax * s, ay * s, az * s, math.cos(angle / 2.0))


def _pose_to_SE3(pose):
    """Convert (x, y, z, rx, ry, rz) with UR rotation vector -> spatialmath SE3."""
    x, y, z, rx, ry, rz = pose
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-10:
        R = np.eye(3)
    else:
        ax, ay, az = rx / angle, ry / angle, rz / angle
        K = np.array([[0.0, -az, ay], [az, 0.0, -ax], [-ay, ax, 0.0]])
        R = np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return _SE3(T, check=False)


# ---------------------------------------------------------------------------
# Trapezoidal velocity profile generator
# ---------------------------------------------------------------------------

def _trapezoidal_profile(dist, speed, accel, dt=1.0 / 240.0):
    """Generate normalized [0..1] interpolation values for a trapezoidal profile."""
    if dist < 1e-8:
        return [1.0]
    speed = min(speed, math.sqrt(dist * accel))
    t_accel = speed / accel
    d_accel = 0.5 * accel * t_accel ** 2
    d_cruise = dist - 2 * d_accel
    t_cruise = max(0.0, d_cruise / speed)
    t_total = 2 * t_accel + t_cruise

    fractions = []
    t = 0.0
    while t < t_total:
        if t < t_accel:
            d = 0.5 * accel * t ** 2
        elif t < t_accel + t_cruise:
            d = d_accel + speed * (t - t_accel)
        else:
            t_decel = t - t_accel - t_cruise
            d = d_accel + d_cruise + speed * t_decel - 0.5 * accel * t_decel ** 2
        fractions.append(min(d / dist, 1.0))
        t += dt
    if not fractions or fractions[-1] < 1.0:
        fractions.append(1.0)
    return fractions

# ---------------------------------------------------------------------------
# World <-> Robot base frame transforms
# ---------------------------------------------------------------------------

def _world_to_base(pos_world, orn_world_quat, base_pos, base_ori_quat):
    """Transform a world-frame pose into the robot's base frame."""
    inv_base_pos, inv_base_ori = p.invertTransform(base_pos, base_ori_quat)
    pos_local, orn_local = p.multiplyTransforms(inv_base_pos, inv_base_ori,
                                                 pos_world, orn_world_quat)
    return np.array(pos_local), orn_local

def _base_to_world(pos_local, orn_local_quat, base_pos, base_ori_quat):
    """Transform a robot base-frame pose into world frame."""
    pos_world, orn_world = p.multiplyTransforms(base_pos, base_ori_quat,
                                                 pos_local, orn_local_quat)
    return np.array(pos_world), orn_world

# ---------------------------------------------------------------------------
# Adapter classes that mirror the real rmplab_uri sub-objects
# ---------------------------------------------------------------------------

class _Recieve:
    def __init__(self, robot, gripper_length=0.0, joint_zero_offsets=None):
        self._robot = robot
        self._grip_len = gripper_length
        self._q_offsets = joint_zero_offsets or [0.0] * 6

    def getActualTCPPose(self):
        state = p.getLinkState(self._robot.id, self._robot.eef_id)
        pos_w = np.array(state[0])
        orn_w = state[1]
        if self._grip_len > 0:
            rot = np.array(p.getMatrixFromQuaternion(orn_w)).reshape(3, 3)
            pos_w = pos_w + rot[:, 0] * self._grip_len
        pos_local, orn_local = _world_to_base(
            pos_w, orn_w, self._robot.base_pos, self._robot.base_ori)
        rv = _quat_to_rotvec(orn_local)
        return (pos_local[0], pos_local[1], pos_local[2], rv[0], rv[1], rv[2])

    def getActualQ(self):
        positions = []
        for i, joint_id in enumerate(self._robot.arm_controllable_joints):
            pos, _, _, _ = p.getJointState(self._robot.id, joint_id)
            positions.append(pos + self._q_offsets[i])
        return tuple(positions)

    def getActualTCPForce(self):
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

class _Control:
    RENDER_EVERY_N_STEPS = 8

    def __init__(self, robot, env, gripper_length=0.0, joint_zero_offsets=None):
        self._robot = robot
        self._env = env
        self._grip_len = gripper_length
        self._q_offsets = joint_zero_offsets or [0.0] * 6
        self._cmd_queue = queue.Queue()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def _worker_loop(self):
        while True:
            func, args, kwargs, done_event = self._cmd_queue.get()
            try:
                func(*args, **kwargs)
            except Exception as e:
                print(f"[rmplab_uri] Command error: {e}")
            finally:
                done_event.set()
                self._cmd_queue.task_done()

    _main_thread_id = threading.main_thread().ident

    def _enqueue(self, func, *args, **kwargs):
        """Queue a command and block until it finishes.
        If called from the main thread, execute directly to avoid deadlock
        with tkinter's event loop."""
        if threading.current_thread().ident == self._main_thread_id:
            func(*args, **kwargs)
        else:
            done = threading.Event()
            self._cmd_queue.put((func, args, kwargs, done))
            done.wait()

    @property
    def _force_mult(self):
        return _physics_params["force_multiplier"]

    @property
    def _pos_gain(self):
        return _physics_params["position_gain"]

    @property
    def _vel_gain(self):
        return _physics_params["velocity_gain"]

    @property
    def _converge_thresh(self):
        return _physics_params["converge_threshold"]

    @property
    def _max_steps(self):
        return int(_physics_params["max_converge_steps"])

    def _maybe_render(self):
        cb = getattr(self._env, 'on_step_render', None)
        if cb:
            cb()

    def _get_current_q(self):
        return [p.getJointState(self._robot.id, jid)[0]
                for jid in self._robot.arm_controllable_joints]

    def _set_arm_targets(self, q):
        for i, jid in enumerate(self._robot.arm_controllable_joints):
            p.setJointMotorControl2(
                self._robot.id, jid, p.POSITION_CONTROL,
                q[i],
                force=self._robot.joints[jid].maxForce * self._force_mult,
                maxVelocity=self._robot.joints[jid].maxVelocity,
            )

    def _set_arm_targets_tuned(self, q):
        for i, jid in enumerate(self._robot.arm_controllable_joints):
            p.setJointMotorControl2(
                self._robot.id, jid, p.POSITION_CONTROL,
                q[i],
                force=self._robot.joints[jid].maxForce * self._force_mult,
                maxVelocity=self._robot.joints[jid].maxVelocity,
                positionGain=self._pos_gain,
                velocityGain=self._vel_gain,
            )

    def _lock_step(self, n=1):
        lock = getattr(self._env, 'sim_lock', None)
        if lock:
            with lock:
                for _ in range(n):
                    self._env.step_simulation()
        else:
            for _ in range(n):
                self._env.step_simulation()

    # ----- Mode 1: Teleport interpolation -----
    def _moveJ_teleport_interp(self, target_q, speed, accel):
        current_q = self._get_current_q()
        max_dist = max(abs(c - t) for c, t in zip(current_q, target_q))
        fracs = _trapezoidal_profile(max_dist, speed, accel)

        lock = getattr(self._env, 'sim_lock', None)
        for step, alpha in enumerate(fracs):
            waypoint = [c + alpha * (t - c) for c, t in zip(current_q, target_q)]
            if lock:
                with lock:
                    for i, jid in enumerate(self._robot.arm_controllable_joints):
                        p.resetJointState(self._robot.id, jid, waypoint[i])
                    self._set_arm_targets(waypoint)
                    self._env.step_simulation()
            else:
                for i, jid in enumerate(self._robot.arm_controllable_joints):
                    p.resetJointState(self._robot.id, jid, waypoint[i])
                self._set_arm_targets(waypoint)
                self._env.step_simulation()
            if step % self.RENDER_EVERY_N_STEPS == 0:
                self._maybe_render()

    # ----- Mode 2: Position control interpolation -----
    def _moveJ_pd_interp(self, target_q, speed, accel):
        current_q = self._get_current_q()
        max_dist = max(abs(c - t) for c, t in zip(current_q, target_q))
        fracs = _trapezoidal_profile(max_dist, speed, accel)

        for step, alpha in enumerate(fracs):
            waypoint = [c + alpha * (t - c) for c, t in zip(current_q, target_q)]
            self._set_arm_targets_tuned(waypoint)
            self._lock_step(1)
            if step % self.RENDER_EVERY_N_STEPS == 0:
                self._maybe_render()

    # ----- Mode 3: Pure physics -----
    def _moveJ_physics(self, target_q, speed, accel):
        self._set_arm_targets_tuned(target_q)
        for step in range(self._max_steps):
            self._lock_step(1)
            if step % self.RENDER_EVERY_N_STEPS == 0:
                self._maybe_render()
            if step % 24 == 23:
                cur = self._get_current_q()
                max_err = max(abs(c - t) for c, t in zip(cur, target_q))
                if max_err < self._converge_thresh:
                    return

    # ----- Dispatch -----
    def _dispatch_moveJ(self, target_q, speed, accel):
        mode = get_movej_mode()
        if mode == "Teleport Interp":
            self._moveJ_teleport_interp(target_q, speed, accel)
        elif mode == "PD Interp":
            self._moveJ_pd_interp(target_q, speed, accel)
        elif mode == "Pure Physics":
            self._moveJ_physics(target_q, speed, accel)

    # ----- Public API (queued) -----

    def moveJ_IK(self, pose, speed=0.5, acceleration=0.5, asynchronous=False):
        self._enqueue(self._do_moveJ_IK, pose, speed, acceleration)

    def moveL(self, pose, speed=0.5, acceleration=0.5, asynchronous=False):
        self.moveJ_IK(pose, speed, acceleration, asynchronous)

    def moveJ(self, q, speed=0.5, acceleration=0.5, asynchronous=False):
        self._enqueue(self._do_moveJ, q, speed, acceleration)

    def moveL_FK(self, q, speed=0.5, acceleration=0.5, asynchronous=False):
        self.moveJ(q, speed, acceleration, asynchronous)

    def servoJ(self, q, velocity=0, acceleration=0, time=0.008,
                lookahead_time=0.1, gain=300):
        self._enqueue(self._do_servoJ, q)

    def servoStop(self):
        self._enqueue(self._do_servoStop)

    # ----- IK queries (rtb-based, mirrors RTDE Control API) -----

    def _ik(self, pose, q0=None):
        if _UR5E_MODEL is None:
            raise RuntimeError("roboticstoolbox not available for IK queries")
        x, y, z, rx, ry, rz = pose
        if self._grip_len > 0:
            quat_local = _rotvec_to_quat(rx, ry, rz)
            rot_local = np.array(p.getMatrixFromQuaternion(quat_local)).reshape(3, 3)
            pos = np.array([x, y, z]) - rot_local[:, 0] * self._grip_len
            pose = (pos[0], pos[1], pos[2], rx, ry, rz)
        if q0 is None:
            q0 = self._get_current_q()
        return _UR5E_MODEL.ikine_LM(_pose_to_SE3(pose), q0=list(q0))

    def getInverseKinematicsHasSolution(self, pose):
        return bool(self._ik(pose).success)

    def getInverseKinematics(self, pose, qnear=None):
        sol = self._ik(pose, q0=qnear)
        if not sol.success:
            raise RuntimeError("IK failed")
        return tuple(float(q) for q in sol.q)

    def isPoseWithinSafetyLimits(self, pose):
        return True

    def teachMode(self):
        pass

    def endTeachMode(self):
        pass

    def stopScript(self):
        pass

    # ----- Internal implementations -----

    def _do_moveJ_IK(self, pose, speed, acceleration):
        x, y, z, rx, ry, rz = pose
        quat_local = _rotvec_to_quat(rx, ry, rz)
        pos_local = np.array([x, y, z])
        # Gripper offset: shift back from fingertip to ee_link (in local frame)
        if self._grip_len > 0:
            rot_local = np.array(p.getMatrixFromQuaternion(quat_local)).reshape(3, 3)
            pos_local = pos_local - rot_local[:, 0] * self._grip_len
        # Convert from robot base frame to world frame for IK
        pos_w, orn_w = _base_to_world(
            pos_local, quat_local,
            self._robot.base_pos, self._robot.base_ori)
        joint_poses = p.calculateInverseKinematics(
            self._robot.id, self._robot.eef_id,
            pos_w.tolist(), orn_w,
            self._robot.arm_lower_limits,
            self._robot.arm_upper_limits,
            self._robot.arm_joint_ranges,
            self._robot.arm_rest_poses,
            maxNumIterations=100,
        )
        target_q = list(joint_poses[:self._robot.arm_num_dofs])
        self._dispatch_moveJ(target_q, speed, acceleration)

    def _do_moveJ(self, q, speed, acceleration):
        target_q = [q[i] - self._q_offsets[i] for i in range(self._robot.arm_num_dofs)]
        self._dispatch_moveJ(target_q, speed, acceleration)

    def _do_servoJ(self, q):
        lock = getattr(self._env, 'sim_lock', None)
        if lock:
            lock.acquire()
        try:
            for i, jid in enumerate(self._robot.arm_controllable_joints):
                p.setJointMotorControl2(
                    self._robot.id, jid, p.POSITION_CONTROL,
                    q[i] - self._q_offsets[i],
                    force=self._robot.joints[jid].maxForce * self._force_mult,
                    maxVelocity=self._robot.joints[jid].maxVelocity,
                )
        finally:
            if lock:
                lock.release()

    def _do_servoStop(self):
        for jid in self._robot.arm_controllable_joints:
            p.setJointMotorControl2(
                self._robot.id, jid, p.VELOCITY_CONTROL,
                targetVelocity=0, force=0,
            )


class _Gripper:
    def __init__(self, robot, env):
        self._robot = robot
        self._env = env

    def get_current_position(self):
        if hasattr(self._robot, 'mimic_parent_id'):
            pos, _, _, _ = p.getJointState(self._robot.id, self._robot.mimic_parent_id)
            return pos
        return 0.0

    def move_and_wait_for_pos(self, pos, speed=10, force=10):
        self._robot.move_gripper(pos)
        for _ in range(60):
            self._env.step_simulation()

    def open(self, speed=10, force=10):
        self._robot.open_gripper()
        for _ in range(60):
            self._env.step_simulation()

    def close(self, speed=10, force=10):
        self._robot.close_gripper()
        for _ in range(60):
            self._env.step_simulation()


class _StubNoOp:
    """Fallback for when no robot is registered — returns zeros for everything."""

    def __getattr__(self, name):
        return _StubNoOp()

    def __call__(self, *args, **kwargs):
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def __iter__(self):
        return iter((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))


def get_registered_env(host):
    """Return the simulation env object for ``host``, or ``None`` if not registered."""
    entry = _robot_registry.get(host)
    return entry[1] if entry else None


def step_env_for_duration(env, seconds):
    """Advance the PyBullet env by ``seconds`` of simulation time (steps ``env``)."""
    dt = getattr(env, "time_step", None)
    if dt is None or dt <= 0:
        dt = 1.0 / 240.0
    n = max(1, int(seconds / dt))
    lock = getattr(env, "sim_lock", None)
    if lock:
        with lock:
            for _ in range(n):
                env.step_simulation()
    else:
        for _ in range(n):
            env.step_simulation()
