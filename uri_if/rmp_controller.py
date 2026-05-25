"""Simple controlled motion helpers for the UR robot."""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from typing import Sequence

_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _RMP_LAB_ROOT in sys.path:
    sys.path.remove(_RMP_LAB_ROOT)
sys.path.insert(0, _RMP_LAB_ROOT)
import uri_if

try:
	from dashboard_handler import DashboardHandler
except ImportError:  # pragma: no cover - fallback when used as a package module
	from .dashboard_handler import DashboardHandler

MAX_LINEAR_SPEED = 0.25
MAX_LINEAR_ACCELERATION = 0.8
SLOWDOWN_RATIO = 0.30
DEFAULT_SLOWDOWN_DISTANCE = 0.05
DEFAULT_POLL_INTERVAL = 0.05
DEFAULT_POSE_TOLERANCE = 0.001
DEFAULT_PREPARE_RADIUS = 0.5
URI_HOST = "192.168.56.101"
AYAL_HOST = "192.168.57.101"
AYAL_START_JOINT = [67.058292, -75.464195, -142.502813, 129.047076, -90.814158, -188.902655]
AYAL_START_POS = [0.163066, 0.048994, 0.060508, -3.098797, 0.372343, 0.006158]

def _pose_to_list(pose: Sequence[float]) -> list[float]:
	if len(pose) != 6:
		raise ValueError(f"moveJ_IK pose must contain 6 values, got {len(pose)}")
	return [float(value) for value in pose]

def _clamp(value: float, minimum: float, maximum: float) -> float:
	return max(minimum, min(maximum, value))

def _translation_distance(a: Sequence[float], b: Sequence[float]) -> float:
	return math.dist(a[:3], b[:3])

def _interpolate_pose(start: Sequence[float], target: Sequence[float], alpha: float) -> list[float]:
	alpha = _clamp(alpha, 0.0, 1.0)
	return [s + alpha * (t - s) for s, t in zip(start, target)]

def _current_tcp_pose(receive) -> list[float]:
	if hasattr(receive, "getActualTCPPose"):
		return [float(value) for value in receive.getActualTCPPose()]
	raise AttributeError("The receive interface must provide getActualTCPPose()")

def _norm3(v: Sequence[float]) -> float:
	return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

def _unit3(v: Sequence[float]) -> list[float]:
	n = _norm3(v)
	if n <= 1e-12:
		raise ValueError("Cannot normalize near-zero vector")
	return [v[0] / n, v[1] / n, v[2] / n]

def _cross(a: Sequence[float], b: Sequence[float]) -> list[float]:
	return [
		a[1] * b[2] - a[2] * b[1],
		a[2] * b[0] - a[0] * b[2],
		a[0] * b[1] - a[1] * b[0],
	]

def _rotmat_to_rotvec(R: Sequence[Sequence[float]]) -> list[float]:
	trace = R[0][0] + R[1][1] + R[2][2]
	c = _clamp(0.5 * (trace - 1.0), -1.0, 1.0)
	theta = math.acos(c)

	if theta < 1e-9:
		return [0.0, 0.0, 0.0]

	s = math.sin(theta)
	if abs(s) < 1e-9:
		x = math.sqrt(max(0.0, 0.5 * (R[0][0] + 1.0)))
		y = math.sqrt(max(0.0, 0.5 * (R[1][1] + 1.0)))
		z = math.sqrt(max(0.0, 0.5 * (R[2][2] + 1.0)))
		if R[2][1] - R[1][2] < 0:
			x = -x
		if R[0][2] - R[2][0] < 0:
			y = -y
		if R[1][0] - R[0][1] < 0:
			z = -z
		axis = _unit3([x, y, z])
		return [axis[0] * theta, axis[1] * theta, axis[2] * theta]

	kx = (R[2][1] - R[1][2]) / (2.0 * s)
	ky = (R[0][2] - R[2][0]) / (2.0 * s)
	kz = (R[1][0] - R[0][1]) / (2.0 * s)
	return [kx * theta, ky * theta, kz * theta]

def _dashboard_stop_requested(dashboard: DashboardHandler | None) -> bool:
	return bool(dashboard and dashboard.is_robot_stopped())

def _watch_dashboard_stop(control, dashboard: DashboardHandler | None, stop_event: threading.Event,
							poll_interval: float, status: dict) -> None:
	if dashboard is None:
		return
	while not stop_event.is_set():
		if _dashboard_stop_requested(dashboard):
			status.setdefault("reason", "dashboard requested stop")
			stop_event.set()
			try:
				control.stopL(0.2)
			except Exception:
				pass
			print(f"cmovel stopped: {status['reason']}")
			return
		time.sleep(poll_interval)

def _watch_force_threshold(control, receive, force_threshold, torque_threshold,
							stop_event: threading.Event, poll_interval: float,
							status: dict) -> None:
	if force_threshold is None and torque_threshold is None:
		return
	while not stop_event.is_set():
		try:
			wrench = list(receive.getActualTCPForce())[:6]
		except Exception:
			return
		f_mag = math.sqrt(wrench[0] * wrench[0] + wrench[1] * wrench[1] + wrench[2] * wrench[2])
		m_mag = math.sqrt(wrench[3] * wrench[3] + wrench[4] * wrench[4] + wrench[5] * wrench[5])
		reason = None
		if force_threshold is not None and f_mag > float(force_threshold):
			reason = f"force {f_mag:.2f} N > {float(force_threshold):.2f} N"
		elif torque_threshold is not None and m_mag > float(torque_threshold):
			reason = f"torque {m_mag:.3f} Nm > {float(torque_threshold):.3f} Nm"
		if reason is not None:
			status["reason"] = reason
			status["wrench"] = wrench
			stop_event.set()
			try:
				control.stopL(0.2)
			except Exception:
				pass
			print(f"cmovel stopped: {reason}")
			return
		time.sleep(poll_interval)

def cmovel(
	control,
	receive,
	target_pose: Sequence[float],
	speed: float,
	acceleration: float,
	dashboard: DashboardHandler | None = None,
	*,
	max_speed: float = MAX_LINEAR_SPEED,
	max_acceleration: float = MAX_LINEAR_ACCELERATION,
	slowdown_ratio: float = SLOWDOWN_RATIO,
	slowdown_distance: float = DEFAULT_SLOWDOWN_DISTANCE,
	poll_interval: float = DEFAULT_POLL_INTERVAL,
	pose_tolerance: float = DEFAULT_POSE_TOLERANCE,
	timeout: float = 60.0,
	force_threshold: float | None = None,
	torque_threshold: float | None = None,
) -> str:
	"""Execute a controlled linear move.

	Parameters
	----------
	control:
		Robot control interface with ``moveJ_IK`` and ``stopL`` methods.
	receive:
		Robot receive interface with ``getActualTCPPose`` and
		``getActualTCPForce``.
	target_pose:
		Target pose ``[x, y, z, rx, ry, rz]``.
	speed / acceleration:
		Requested motion profile, clamped to the configured maxima.
	dashboard:
		Optional dashboard wrapper used to stop the move if the robot is
		externally stopped.
	force_threshold / torque_threshold:
		If set, a watcher thread polls ``getActualTCPForce`` and triggers
		``stopL`` when ``|F| > force_threshold`` (N) or
		``|M| > torque_threshold`` (Nm). Caller is responsible for zeroing the
		F/T sensor before the move.

	Returns
	-------
	str
		``"finished"``, ``"already_at_target"``, ``"stopped_by_dashboard"``,
		or ``"stopped_by_force"``.
	"""

	target_pose = _pose_to_list(target_pose)
	speed = _clamp(abs(float(speed)), 0.0, float(max_speed))
	acceleration = _clamp(abs(float(acceleration)), 0.0, float(max_acceleration))
	slowdown_ratio = _clamp(float(slowdown_ratio), 0.0, 1.0)
	slowdown_distance = max(0.0, float(slowdown_distance))
	poll_interval = max(0.01, float(poll_interval))
	pose_tolerance = max(1e-5, float(pose_tolerance))
	timeout = max(0.1, float(timeout))

	start_pose = _current_tcp_pose(receive)
	total_distance = _translation_distance(start_pose, target_pose)

	if total_distance <= pose_tolerance:
		print("cmovel finished: already at target")
		return "already_at_target"

	stop_event = threading.Event()
	status: dict = {}
	watchers = []
	watchers.append(threading.Thread(
		target=_watch_dashboard_stop,
		args=(control, dashboard, stop_event, poll_interval, status),
		daemon=True,
	))
	if force_threshold is not None or torque_threshold is not None:
		watchers.append(threading.Thread(
			target=_watch_force_threshold,
			args=(control, receive, force_threshold, torque_threshold,
				  stop_event, poll_interval, status),
			daemon=True,
		))
	for w in watchers:
		w.start()

	def _finish_status() -> str:
		stop_event.set()
		for w in watchers:
			w.join(timeout=1.0)
		reason = status.get("reason")
		if reason is None:
			# print("cmovel finished: mission completed")
			return "finished"
		if "force" in reason or "torque" in reason:
			return "stopped_by_force"
		return "stopped_by_dashboard"

	slow_distance = min(slowdown_distance, total_distance)
	slow_speed = max(speed * slowdown_ratio, 0.01)

	try:
		if slow_distance <= 0.0 or total_distance <= slow_distance:
			control.moveJ_IK(target_pose, slow_speed, acceleration, False)
		else:
			fast_fraction = (total_distance - slow_distance) / total_distance
			fast_target = _interpolate_pose(start_pose, target_pose, fast_fraction)
			control.moveJ_IK(fast_target, speed, acceleration, False)
			if stop_event.is_set():
				return _finish_status()
			control.moveJ_IK(target_pose, slow_speed, acceleration, False)
	except Exception:
		# moveJ_IK can raise if a watcher called stopL mid-motion; treat as a
		# normal early termination and report the reason captured by the watcher.
		return _finish_status()

	return _finish_status()

def cmover(
	control,
	receive,
	x: float,
	y: float,
	z: float,
	speed: float,
	acceleration: float,
	dashboard: DashboardHandler | None = None,
	*,
	radial_epsilon: float = 1e-9,
	**kwargs,
) -> str:
	"""Move to [x,y,z] with TCP Z axis aligned radially from the base axis."""
	x = float(x)
	y = float(y)
	z = float(z)
	radial_epsilon = max(0.0, float(radial_epsilon))

	r = math.hypot(x, y)
	if r < radial_epsilon:
		raise ValueError("Radial direction undefined near base axis (x,y≈0)")

	z_tool = [x / r, y / r, 0.0]
	x_guess = [0.0, 0.0, 1.0]
	y_tool = _unit3(_cross(z_tool, x_guess))
	x_tool = _unit3(_cross(y_tool, z_tool))

	R = [
		[x_tool[0], y_tool[0], z_tool[0]],
		[x_tool[1], y_tool[1], z_tool[1]],
		[x_tool[2], y_tool[2], z_tool[2]],
	]
	rx, ry, rz = _rotmat_to_rotvec(R)
	target_pose = [x, y, z, rx, ry, rz]
	return cmovel(
		control,
		receive,
		target_pose,
		speed,
		acceleration,
		dashboard,
		**kwargs,
	)

def cmovep(
	control,
	receive,
	x: float,
	y: float,
	z: float,
	speed: float,
	acceleration: float,
	dashboard: DashboardHandler | None = None,
	*,
	prepare_radius: float = DEFAULT_PREPARE_RADIUS,
	radial_epsilon: float = 1e-9,
	**kwargs,
) -> str:
	"""Move to a radial-prepare point (fixed radius) using ``cmover`` behavior."""
	x = float(x)
	y = float(y)
	z = float(z)
	prepare_radius = abs(float(prepare_radius))
	radial_epsilon = max(0.0, float(radial_epsilon))

	r = math.hypot(x, y)
	if r < radial_epsilon:
		raise ValueError("Radial direction undefined near base axis (x,y≈0)")

	ux = x / r
	uy = y / r
	x_prepare = ux * prepare_radius
	y_prepare = uy * prepare_radius

	return cmover(
		control,
		receive,
		x_prepare,
		y_prepare,
		z,
		speed,
		acceleration,
		dashboard,
		radial_epsilon=radial_epsilon,
		**kwargs,
	)

def cmoves(
	ayal_control,
	ayal_receive,
	uri_control,
	uri_receive,
	*,
	base_step_rad: float = math.radians(0.25),
	base_speed: float = 0.03,
	base_acceleration: float = 0.08,
	tcp_shift_threshold_m: float = 0.002,
	consecutive_samples: int = 3,
	max_base_rotation_rad: float = math.radians(25.0),
	max_steps: int = 300,
	direction: int = 1,
	settle_s: float = 0.03,
) -> str:
	"""Rotate only Ayal base joint until Uri TCP shifts beyond threshold.

	Uri is put in teach mode during the scan. The routine exits with one of:
	``triggered``, ``max_rotation``, ``max_steps``, ``invalid_parameters``.
	"""
	direction = 1 if int(direction) >= 0 else -1
	base_step_rad = abs(float(base_step_rad))
	base_speed = abs(float(base_speed))
	base_acceleration = abs(float(base_acceleration))
	tcp_shift_threshold_m = max(0.0, float(tcp_shift_threshold_m))
	consecutive_samples = max(1, int(consecutive_samples))
	max_base_rotation_rad = abs(float(max_base_rotation_rad))
	max_steps = max(1, int(max_steps))
	settle_s = max(0.0, float(settle_s))

	if base_step_rad <= 0.0:
		print("cmoves stopped: invalid base_step_rad")
		return "invalid_parameters"

	q_start = list(ayal_receive.getActualQ())
	base_start = float(q_start[0])
	tcp_start = list(uri_receive.getActualTCPPose())

	trigger_count = 0
	status = "max_steps"

	uri_control.teachMode()
	try:
		for _ in range(max_steps):
			q_now = list(ayal_receive.getActualQ())
			q_target = q_now.copy()
			q_target[0] = float(q_now[0]) + direction * base_step_rad
			ayal_control.moveJ(q_target, base_speed, base_acceleration, False)

			if settle_s > 0.0:
				time.sleep(settle_s)

			tcp_now = list(uri_receive.getActualTCPPose())
			shift_m = _translation_distance(tcp_start, tcp_now)

			if shift_m >= tcp_shift_threshold_m:
				trigger_count += 1
				if trigger_count >= consecutive_samples:
					status = "triggered"
					print(f"cmoves stopped: tcp shift detected ({shift_m:.6f} m)")
					break
			else:
				trigger_count = 0

			base_now = float(ayal_receive.getActualQ()[0])
			if abs(base_now - base_start) >= max_base_rotation_rad:
				status = "max_rotation"
				print("cmoves stopped: reached max base rotation")
				break
	finally:
		try:
			uri_control.endTeachMode()
		except Exception:
			pass

	if status == "max_steps":
		print("cmoves stopped: reached max steps")
	return status

def main() -> None:
	parser = argparse.ArgumentParser(description="Run a simple controlled moveJ_IK demo.")
	parser.add_argument("--host", default=URI_HOST, help="Robot host IP")
	target_group = parser.add_mutually_exclusive_group(required=True)
	target_group.add_argument("--target", nargs=6, type=float, metavar=("x", "y", "z", "rx", "ry", "rz"),
							  help="Target pose: x y z rx ry rz")
	target_group.add_argument("--xyz", nargs=3, type=float, metavar=("x", "y", "z"),
							  help="Target position for radial mode: x y z")
	parser.add_argument("--spin", type=int, help="Spin direction: 1 for clockwise, -1 for counterclockwise")
	parser.add_argument("--speed", type=float, default=0.10, help="Requested linear speed")
	parser.add_argument("--acceleration", type=float, default=0.20, help="Requested linear acceleration")
	parser.add_argument("--no-dashboard", action="store_true", help="Disable dashboard stop checks")
	args = parser.parse_args()

	uri = uri_if.RMPLAB_Uri(URI_HOST)
	ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
	dashboard = None if args.no_dashboard else DashboardHandler(URI_HOST)

	uri.connect(False)
	try:
		if args.spin is not None:
			cmoves(
                ayal.control,
                ayal.recieve,
                uri.control,
                uri.recieve,
                direction=args.spin,
            )
		elif args.xyz is not None:
			result = cmover(
				uri.control,
				uri.recieve,
				args.xyz[0],
				args.xyz[1],
				args.xyz[2],
				args.speed,
				args.acceleration,
				dashboard,
			)
		else:
			result = cmovel(
				uri.control,
				uri.recieve,
				args.target,
				args.speed,
				args.acceleration,
				dashboard,
			)
		print(f"motion result: {result}")
	finally:
		uri.disconnect()
		if dashboard is not None:
			dashboard.disconnect()

__all__ = ["cmovel", "cmover", "cmovep", "cmoves", "main"]

def main2() -> None:
	dashboard = DashboardHandler(URI_HOST)
	dashboard.unlock_protective_stop()

if __name__ == "__main__":
	main()

