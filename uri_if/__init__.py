"""
``rmplab_uri`` — UR5 control for RMP Lab (hardware RTDE + optional PyBullet sim).

**Backends**

* ``backend="sim"`` — PyBullet (call :func:`register_robot` first).
* ``backend="real"`` — UR RTDE + Robotiq (requires ``ur_rtde``).
* ``backend="auto"`` (default) — ``RMPLAB_BACKEND`` env, else sim if this ``host``
  is registered, else real.

**Trajectory delays**

Use :func:`trajectory_sleep` instead of ``time.sleep`` in scripts that must run on
both sim and real: sim steps physics; real uses wall-clock sleep.
"""

from __future__ import annotations

import os
import warnings
from types import SimpleNamespace
from typing import Callable, Literal, Optional

from .constants import _RMP_LAB_ROOT, CALIB_FILE, GRIPPER_PORT, HOST, HOST_AYAL, HOST_URI

try:
    from . import sim_impl
except ImportError:
    class _StubNoOp:
        def __getattr__(self, name):
            return self

        def __call__(self, *args, **kwargs):
            return self

        def __iter__(self):
            return iter((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def _missing_sim_impl(*args, **kwargs):
        raise ImportError(
            "PyBullet simulation backend is unavailable. Install `pybullet` to use sim mode."
        )

    sim_impl = SimpleNamespace(
        register_robot=_missing_sim_impl,
        set_movej_mode=_missing_sim_impl,
        get_movej_mode=_missing_sim_impl,
        set_physics_params=_missing_sim_impl,
        get_physics_params=_missing_sim_impl,
        MOVEJ_MODES=["Teleport Interp", "PD Interp", "Pure Physics"],
        _quat_to_rotvec=_missing_sim_impl,
        _rotvec_to_quat=_missing_sim_impl,
        _world_to_base=_missing_sim_impl,
        _base_to_world=_missing_sim_impl,
        _StubNoOp=_StubNoOp,
        _robot_registry={},
        _gripper_lengths={},
        _joint_zero_offsets={},
        get_registered_env=lambda host: None,
        step_env_for_duration=lambda env, seconds: None,
    )

# Backwards-compatible alias: allows ``from sim_uri import sim_uri``.
sim_uri = None

# ---------------------------------------------------------------------------
# Re-exports (sim / PyBullet)
# ---------------------------------------------------------------------------

register_robot = sim_impl.register_robot
set_movej_mode = sim_impl.set_movej_mode
get_movej_mode = sim_impl.get_movej_mode
set_physics_params = sim_impl.set_physics_params
get_physics_params = sim_impl.get_physics_params
MOVEJ_MODES = sim_impl.MOVEJ_MODES
_quat_to_rotvec = sim_impl._quat_to_rotvec
_rotvec_to_quat = sim_impl._rotvec_to_quat
_world_to_base = sim_impl._world_to_base
_base_to_world = sim_impl._base_to_world

# ---------------------------------------------------------------------------
# Trajectory timing (sim steps physics; real sleeps)
# ---------------------------------------------------------------------------

_trajectory_sleep_override: Optional[Callable[..., None]] = None
_default_trajectory_sim_host: Optional[str] = None


def set_trajectory_sleep_override(fn: Optional[Callable[..., None]]) -> None:
    """If set, :func:`trajectory_sleep` delegates to ``fn(seconds, host=...)``."""
    global _trajectory_sleep_override
    _trajectory_sleep_override = fn


def trajectory_sleep(seconds: float, host: Optional[str] = None) -> None:
    """
    Wait ``seconds`` — on sim, step the registered PyBullet env; on real, ``time.sleep``.

    Pass ``host`` to pick a specific registered robot; otherwise the last sim
    :meth:`sim_uri.connect` sets the default, or set ``RMPLAB_TRAJECTORY_HOST``.
    """
    if _trajectory_sleep_override is not None:
        _trajectory_sleep_override(seconds, host)
        return

    import time

    h = host or _default_trajectory_sim_host or os.environ.get("RMPLAB_TRAJECTORY_HOST")
    env = sim_impl.get_registered_env(h) if h else None
    if env is not None:
        sim_impl.step_env_for_duration(env, float(seconds))
    else:
        time.sleep(float(seconds))


BackendName = Literal["auto", "sim", "real"]


def _resolve_backend(requested: str, host: str) -> str:
    req = (requested or "auto").strip().lower()
    if req in ("sim", "real"):
        return req
    env = os.environ.get("RMPLAB_BACKEND", "").strip().lower()
    if env in ("sim", "real"):
        return env
    if host in sim_impl._robot_registry:
        return "sim"
    return "real"


class RMPLAB_Uri:
    """
    Unified interface: PyBullet sim or real RTDE + Robotiq.

    Parameters
    ----------
    host:
        Robot IP / key used with :func:`register_robot`. Defaults to :data:`HOST`.
    backend:
        ``"auto"`` | ``"sim"`` | ``"real"`` — see module docstring.
    """

    def __init__(self, host: Optional[str] = None, backend: BackendName = "auto"):
        self.host = host if host is not None else HOST
        self._backend_requested: BackendName = backend
        self.teachmode = False
        self._mode: Optional[str] = None
        self._connected = False
        self._real_uri = None

        self.recieve = sim_impl._StubNoOp()
        self.control = sim_impl._StubNoOp()
        self.dashboard = sim_impl._StubNoOp()
        self.gripper = sim_impl._StubNoOp()

    def connect(
        self,
        gripper_calibrate: bool = True,
        *,
        calibrate: Optional[bool] = None,
        **kwargs,
    ) -> None:
        """
        Connect to sim or hardware.

        Accepts ``gripper_calibrate`` (real) and legacy ``calibrate=`` (alias for sim scripts).
        """
        if kwargs:
            warnings.warn(f"Ignoring unknown connect() kwargs: {kwargs!r}", stacklevel=2)
        if calibrate is not None:
            gripper_calibrate = calibrate

        mode = _resolve_backend(self._backend_requested, self.host)
        if mode == "sim":
            self._connect_sim(gripper_calibrate)
        else:
            self._connect_real(gripper_calibrate)

    def _connect_sim(self, gripper_calibrate: bool) -> None:
        del gripper_calibrate  # unused on sim
        entry = sim_impl._robot_registry.get(self.host)
        if entry is None:
            warnings.warn(
                f"rmplab_uri SIM: no PyBullet robot registered for {self.host!r}. "
                f"Available hosts: {list(sim_impl._robot_registry.keys())}",
                stacklevel=2,
            )
            return
        robot, env = entry
        grip_len = sim_impl._gripper_lengths.get(self.host, 0.0)
        q_offsets = sim_impl._joint_zero_offsets.get(self.host, [0.0] * 6)
        self.recieve = sim_impl._Recieve(robot, grip_len, q_offsets)
        self.control = sim_impl._Control(robot, env, grip_len, q_offsets)
        self.gripper = sim_impl._Gripper(robot, env)
        self._connected = True
        self._mode = "sim"
        global _default_trajectory_sim_host
        _default_trajectory_sim_host = self.host
        print(
            f"[rmplab_uri SIM] Connected to PyBullet robot at {self.host!r} "
            f"(body id={robot.id})"
        )

    def _connect_real(self, gripper_calibrate: bool) -> None:
        try:
            from . import real_impl
        except ImportError as e:
            raise ImportError(
                "Real robot backend requires optional dependency 'ur_rtde'. "
                "Install with: pip install ur_rtde"
            ) from e

        self._real_uri = real_impl.RealRMPLAB_Uri(self.host)
        self._real_uri.connect(gripper_calibrate=gripper_calibrate)
        self.control = self._real_uri.control
        self.recieve = self._real_uri.recieve
        self.gripper = self._real_uri.gripper
        self._connected = True
        self._mode = "real"
        print(f"[rmplab_uri REAL] Connected RTDE + gripper at {self.host!r}")

    def disconnect(self) -> None:
        global _default_trajectory_sim_host

        if self._mode == "real" and self._real_uri is not None:
            self._real_uri.disconnect()
            self._real_uri = None
        elif self._mode == "sim":
            if _default_trajectory_sim_host == self.host:
                _default_trajectory_sim_host = None

        self._connected = False
        self._mode = None
        self.recieve = sim_impl._StubNoOp()
        self.control = sim_impl._StubNoOp()
        self.gripper = sim_impl._StubNoOp()
        print(f"[rmplab_uri] Disconnected {self.host!r}")

    def is_connected(self) -> bool:
        if self._mode == "real" and self._real_uri is not None:
            return self._real_uri.is_connected()
        if self._mode == "sim":
            return self._connected
        return False


__all__ = [
    "HOST",
    "HOST_AYAL",
    "HOST_URI",
    "GRIPPER_PORT",
    "CALIB_FILE",
    "sim_uri",
    "RMPLAB_Uri",
    "register_robot",
    "set_movej_mode",
    "get_movej_mode",
    "MOVEJ_MODES",
    "set_physics_params",
    "get_physics_params",
    "trajectory_sleep",
    "set_trajectory_sleep_override",
    "_quat_to_rotvec",
    "_rotvec_to_quat",
    "_world_to_base",
    "_base_to_world",
]

sim_uri = RMPLAB_Uri
