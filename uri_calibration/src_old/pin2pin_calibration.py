#!/usr/bin/env python3
"""Pin-to-pin base calibration analysis from paired TCP samples.

Input file format (current):
	1: ((uri_pose_6d), (ayal_pose_6d))

Where each pose is [x, y, z, rx, ry, rz].

Pipeline:
1) Parse all samples.
2) Convert both poses to transforms with :func:`pose_to_T`.
3) Primary estimate (robust to varying TCP orientation):
	solve R,t from positions only: p_src ~= R @ p_tgt + t
4) Legacy estimate for reference:
	T_base_src_to_base_tgt = T_src_tcp @ inv(T_tgt_tcp)
5) Report residuals/outliers/trend.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Add calibration module path
_REPO_ROOT = Path(__file__).resolve().parents[2]
_CALIB_DIR = _REPO_ROOT / "calibration"
if str(_CALIB_DIR) not in sys.path:
	sys.path.insert(0, str(_CALIB_DIR))

from gemini_calc_v2 import (  # type: ignore[import-not-found]
	pose_to_T,
	T_to_pose,
	rotvec_to_R,
	calculate_optimal_calibration,
)

@dataclass
class Sample:
	idx: int
	src_pose: np.ndarray
	tgt_pose: np.ndarray

def parse_holding_pin_file(file_path: str | Path) -> list[Sample]:
	samples: list[Sample] = []
	with open(file_path, "r", encoding="utf-8") as f:
		for line_no, raw in enumerate(f, start=1):
			line = raw.strip()
			if not line:
				continue
			if ":" not in line:
				print(f"Skipping line {line_no}: missing ':'")
				continue

			idx_text, payload = line.split(":", 1)
			try:
				idx = int(idx_text.strip())
			except Exception:
				idx = line_no

			try:
				pair = ast.literal_eval(payload.strip())
				if not isinstance(pair, (tuple, list)) or len(pair) != 2:
					raise ValueError("Expected pair of poses")
				src, tgt = pair
				if len(src) != 6 or len(tgt) != 6:
					raise ValueError("Each pose must have 6 components")
				samples.append(
					Sample(
						idx=idx,
						src_pose=np.asarray(src, dtype=float),
						tgt_pose=np.asarray(tgt, dtype=float),
					)
				)
			except Exception as e:
				print(f"Skipping line {line_no}: parse error: {e}")
	return samples

def apply_tip_offset(pose: np.ndarray, offset_in_tcp: np.ndarray) -> np.ndarray:
	"""Return pose translated to a new tool tip whose position in the TCP frame is offset_in_tcp.
	Rotation is preserved; only the position is shifted by R(rotvec) @ offset_in_tcp."""
	T = pose_to_T(pose)
	t_new = T[:3, 3] + T[:3, :3] @ np.asarray(offset_in_tcp, dtype=float)
	return np.array([t_new[0], t_new[1], t_new[2], pose[3], pose[4], pose[5]], dtype=float)


def apply_tip_offsets_to_samples(
	samples: list[Sample],
	src_offset: np.ndarray,
	tgt_offset: np.ndarray,
) -> list[Sample]:
	return [
		Sample(
			idx=s.idx,
			src_pose=apply_tip_offset(s.src_pose, src_offset),
			tgt_pose=apply_tip_offset(s.tgt_pose, tgt_offset),
		)
		for s in samples
	]


def estimate_b2b_poses(samples: list[Sample]) -> np.ndarray:
	"""Estimate base-to-base pose for each sample.

	Convention used:
		T_srcBase_to_tgtBase = T_srcBase_to_srcTcp @ inv(T_tgtBase_to_tgtTcp)
	"""
	poses = []
	for s in samples:
		T_src_tcp = pose_to_T(s.src_pose)
		T_tgt_tcp = pose_to_T(s.tgt_pose)
		T_src_tgt = T_src_tcp @ np.linalg.inv(T_tgt_tcp)
		poses.append(T_to_pose(T_src_tgt))
	return np.asarray(poses, dtype=float)

def estimate_b2b_from_positions(samples: list[Sample]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
	"""Estimate a single base-to-base transform from xyz only.

	Model:
		p_src ~= R @ p_tgt + t
	where R,t are constant across samples.

	Returns:
		(T_src_tgt, pose_src_tgt, residuals_m)
	"""
	p_src = np.asarray([s.src_pose[:3] for s in samples], dtype=float)
	p_tgt = np.asarray([s.tgt_pose[:3] for s in samples], dtype=float)

	c_src = np.mean(p_src, axis=0)
	c_tgt = np.mean(p_tgt, axis=0)
	X = p_tgt - c_tgt
	Y = p_src - c_src

	H = X.T @ Y
	U, _, Vt = np.linalg.svd(H)
	R = Vt.T @ U.T
	if np.linalg.det(R) < 0:
		Vt[-1, :] *= -1
		R = Vt.T @ U.T

	t = c_src - R @ c_tgt

	T = np.eye(4)
	T[:3, :3] = R
	T[:3, 3] = t
	pose = T_to_pose(T)

	pred = (R @ p_tgt.T).T + t
	residuals = np.linalg.norm(pred - p_src, axis=1)
	return T, pose, residuals

def _unwrap_rotvec_columns(rotvecs: np.ndarray) -> np.ndarray:
	"""Unwrap each rotvec component along sample order to reduce 2π jumps."""
	out = rotvecs.copy()
	for j in range(3):
		out[:, j] = np.unwrap(out[:, j])
	return out

def mse_mean_pose(poses: np.ndarray) -> np.ndarray:
	"""Mean vector minimizing MSE in 6D Euclidean vector space."""
	work = poses.copy()
	work[:, 3:] = _unwrap_rotvec_columns(work[:, 3:])
	return np.mean(work, axis=0)

def rot_angle_between(rv_a: np.ndarray, rv_b: np.ndarray) -> float:
	"""Absolute rotation difference angle (rad)."""
	Ra = rotvec_to_R(rv_a)
	Rb = rotvec_to_R(rv_b)
	R = Ra.T @ Rb
	c = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
	return float(math.acos(c))

def robust_z_scores(values: np.ndarray) -> np.ndarray:
	med = float(np.median(values))
	mad = float(np.median(np.abs(values - med)))
	if mad < 1e-12:
		return np.zeros_like(values)
	return 0.6745 * (values - med) / mad

def analyze(poses: np.ndarray) -> dict:
	mean_pose = mse_mean_pose(poses)

	trans_err = np.linalg.norm(poses[:, :3] - mean_pose[:3], axis=1)
	rot_err = np.array([rot_angle_between(mean_pose[3:], p[3:]) for p in poses], dtype=float)
	vec_mse = np.mean((poses - mean_pose) ** 2, axis=1)

	z_t = robust_z_scores(trans_err)
	z_r = robust_z_scores(rot_err)
	z_m = robust_z_scores(vec_mse)

	outlier_mask = (np.abs(z_t) > 3.5) | (np.abs(z_r) > 3.5) | (np.abs(z_m) > 3.5)
	outliers = np.where(outlier_mask)[0].tolist()

	n = poses.shape[0]
	idx = np.arange(n, dtype=float)

	trans_from_first = np.linalg.norm(poses[:, :3] - poses[0, :3], axis=1)
	rot_from_first = np.array([rot_angle_between(poses[0, 3:], poses[i, 3:]) for i in range(n)], dtype=float)

	# slope per sample index; positive slope may indicate drift over acquisition order
	slope_trans = float(np.polyfit(idx, trans_from_first, 1)[0]) if n >= 2 else 0.0
	slope_rot = float(np.polyfit(idx, rot_from_first, 1)[0]) if n >= 2 else 0.0

	return {
		"mean_pose": mean_pose,
		"trans_err": trans_err,
		"rot_err": rot_err,
		"vec_mse": vec_mse,
		"outlier_indices": outliers,
		"trans_err_mean": float(np.mean(trans_err)),
		"trans_err_std": float(np.std(trans_err)),
		"rot_err_mean_deg": float(np.degrees(np.mean(rot_err))),
		"rot_err_std_deg": float(np.degrees(np.std(rot_err))),
		"mse_mean": float(np.mean(vec_mse)),
		"trend": {
			"trans_slope_m_per_sample": slope_trans,
			"rot_slope_deg_per_sample": float(np.degrees(slope_rot)),
		},
		"z_scores": {
			"translation": z_t,
			"rotation": z_r,
			"mse": z_m,
		},
	}

def format_pose(p: np.ndarray, digits: int = 6) -> str:
	return "[" + ", ".join(f"{float(v):.{digits}f}" for v in p) + "]"

def print_report(samples: list[Sample], poses: np.ndarray, report: dict) -> None:
	print("\n=== Pin2Pin Base Calibration Analysis ===")
	print(f"Samples: {len(samples)}")
	print("Convention: T_srcBase_to_tgtBase = T_srcTcp @ inv(T_tgtTcp)")
	print(f"MSE-mean pose [x,y,z,rx,ry,rz]: {format_pose(report['mean_pose'])}")
	print(
		"Dispersion: "
		f"translation mean={report['trans_err_mean']:.6f} m, std={report['trans_err_std']:.6f} m; "
		f"rotation mean={report['rot_err_mean_deg']:.3f} deg, std={report['rot_err_std_deg']:.3f} deg"
	)
	print(f"Mean vector MSE: {report['mse_mean']:.6e}")

	print("\nPer-sample estimates:")
	for i, s in enumerate(samples):
		mark = " OUTLIER" if i in report["outlier_indices"] else ""
		print(
			f"  sample {s.idx:>2}: pose={format_pose(poses[i], digits=4)} | "
			f"dT={report['trans_err'][i]:.4f} m, dR={math.degrees(report['rot_err'][i]):.2f} deg, "
			f"mse={report['vec_mse'][i]:.3e}{mark}"
		)

	if report["outlier_indices"]:
		ids = [samples[i].idx for i in report["outlier_indices"]]
		print(f"\nOutliers detected (robust MAD z-score > 3.5): sample ids {ids}")
	else:
		print("\nOutliers detected: none")

	t_slope = report["trend"]["trans_slope_m_per_sample"]
	r_slope = report["trend"]["rot_slope_deg_per_sample"]
	print("\nTrend check (vs sample index):")
	print(f"  translation slope: {t_slope:+.6f} m/sample")
	print(f"  rotation slope:    {r_slope:+.3f} deg/sample")
	if abs(t_slope) > 0.002 or abs(r_slope) > 1.0:
		print("  -> possible calibration drift/change during sampling")
	else:
		print("  -> no strong monotonic drift signal")


def print_position_only_report(samples: list[Sample], pose: np.ndarray, residuals: np.ndarray) -> None:
	print("\n=== Position-only fit (recommended when TCP angle varies) ===")
	print(f"Single fitted pose [x,y,z,rx,ry,rz]: {format_pose(pose)}")
	print(
		"Residuals (point alignment): "
		f"mean={float(np.mean(residuals)):.6f} m, std={float(np.std(residuals)):.6f} m, "
		f"max={float(np.max(residuals)):.6f} m"
	)

	z = robust_z_scores(residuals)
	outliers = [samples[i].idx for i in np.where(np.abs(z) > 3.5)[0].tolist()]

	for i, s in enumerate(samples):
		mark = " OUTLIER" if abs(z[i]) > 3.5 else ""
		print(f"  sample {s.idx:>2}: residual={residuals[i]:.6f} m (z={z[i]:+.2f}){mark}")

	if outliers:
		print(f"Outliers by residual (MAD z>3.5): {outliers}")
	else:
		print("Outliers by residual: none")

def main() -> int:
	parser = argparse.ArgumentParser(description="Analyze base-to-base calibration from paired TCP samples")
	parser.add_argument(
		"samples_file",
		nargs="?",
		default=str(_REPO_ROOT / "playground" / "logs" / "holding_pin_poses.txt"),
		help="Path to sample text file",
	)
	parser.add_argument(
		"--json-out",
		default=None,
		help="Optional output JSON report path",
	)
	parser.add_argument(
		"--src-tip-offset",
		nargs=3,
		type=float,
		metavar=("X", "Y", "Z"),
		default=[0.0, 0.0, 0.0],
		help="Pin tip position in URI (src) TCP frame, meters. e.g. 0 0 0.238",
	)
	parser.add_argument(
		"--tgt-tip-offset",
		nargs=3,
		type=float,
		metavar=("X", "Y", "Z"),
		default=[0.0, 0.0, 0.0],
		help="Pin tip position in AYAL (tgt) TCP frame, meters.",
	)
	args = parser.parse_args()

	samples = parse_holding_pin_file(args.samples_file)
	if len(samples) < 2:
		print("Need at least 2 valid samples")
		return 1
	if len(samples) < 3:
		print("Warning: fewer than 3 samples; rigid 3D fit may be weakly constrained")

	if any(args.src_tip_offset) or any(args.tgt_tip_offset):
		print(
			f"Applying TCP->tip offsets: src={args.src_tip_offset} m, tgt={args.tgt_tip_offset} m"
		)
		samples = apply_tip_offsets_to_samples(
			samples,
			np.asarray(args.src_tip_offset, dtype=float),
			np.asarray(args.tgt_tip_offset, dtype=float),
		)

	# Primary method: position-only fit (robust to changing TCP orientation)
	_, pos_pose, residuals = estimate_b2b_from_positions(samples)
	print_position_only_report(samples, pos_pose, residuals)

	# Legacy method for comparison/debugging only
	poses = estimate_b2b_poses(samples)
	report = analyze(poses)
	print("\n(Reference only) full-pose method that assumes fixed TCP relative orientation:")
	print_report(samples, poses, report)

	# Optional cross-check with Gemini's calibration pipeline (flip model + geometric median)
	cal_rows = [
		{
			"P_uri_when_calib": s.src_pose.tolist(),
			"P_ayal_when_calib": s.tgt_pose.tolist(),
		}
		for s in samples
	]
	with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as tf:
		json.dump(cal_rows, tf, indent=2)
		temp_cal_path = tf.name

	try:
		opt_pose, all_poses, _ = calculate_optimal_calibration(temp_cal_path, single_calibration=False)
		if opt_pose is not None and all_poses:
			gemini_poses = np.asarray(all_poses, dtype=float)
			gemini_report = analyze(gemini_poses)
			print("\n=== Gemini-model cross-check (flip + geometric median) ===")
			print(f"Gemini optimal pose: {format_pose(np.asarray(opt_pose), digits=6)}")
			print(
				"Gemini dispersion: "
				f"translation mean={gemini_report['trans_err_mean']:.6f} m, "
				f"rotation mean={gemini_report['rot_err_mean_deg']:.3f} deg, "
				f"MSE={gemini_report['mse_mean']:.6e}"
			)
	finally:
		try:
			Path(temp_cal_path).unlink(missing_ok=True)
		except Exception:
			pass

	if args.json_out:
		payload = {
			"sample_ids": [s.idx for s in samples],
			"position_only_pose": pos_pose.tolist(),
			"position_only_residuals_m": residuals.tolist(),
			"estimated_poses": poses.tolist(),
			"mean_pose": report["mean_pose"].tolist(),
			"trans_err": report["trans_err"].tolist(),
			"rot_err_deg": np.degrees(report["rot_err"]).tolist(),
			"vec_mse": report["vec_mse"].tolist(),
			"outlier_indices": report["outlier_indices"],
			"trend": report["trend"],
		}
		with open(args.json_out, "w", encoding="utf-8") as f:
			json.dump(payload, f, indent=2)
			f.write("\n")
		print(f"\nSaved JSON report: {args.json_out}")

	return 0

if __name__ == "__main__":
	raise SystemExit(main())

