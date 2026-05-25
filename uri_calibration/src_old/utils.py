#!/usr/bin/env python3
import numpy as np
import json
from pathlib import Path

# from calibration.move_ayal_in_uri_coor import P_uri_when_calib
from scipy.optimize import minimize

# UR-style pose: [x, y, z, rx, ry, rz] where r* is rotation-vector (axis * angle), radians.

def rotvec_to_R(rv):
    rv = np.asarray(rv, dtype=float)
    theta = np.linalg.norm(rv)
    if theta < 1e-12:
        return np.eye(3)
    k = rv / theta
    K = np.array([[0, -k[2],  k[1]],
                  [k[2],   0, -k[0]],
                  [-k[1], k[0], 0]], dtype=float)
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

def R_to_rotvec(R):
    R = np.asarray(R, dtype=float)
    cos_theta = (np.trace(R) - 1.0) / 2.0
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    theta = np.arccos(cos_theta)
    if theta < 1e-12:
        return np.zeros(3)

    # Near pi: use a more stable extraction
    if abs(theta - np.pi) < 1e-5:
        axis = np.array([
            np.sqrt(max(0.0, (R[0, 0] + 1) / 2)),
            np.sqrt(max(0.0, (R[1, 1] + 1) / 2)),
            np.sqrt(max(0.0, (R[2, 2] + 1) / 2)),
        ], dtype=float)
        # Fix signs from off-diagonals
        if (R[2, 1] - R[1, 2]) < 0: axis[0] = -axis[0]
        if (R[0, 2] - R[2, 0]) < 0: axis[1] = -axis[1]
        if (R[1, 0] - R[0, 1]) < 0: axis[2] = -axis[2]
        n = np.linalg.norm(axis)
        axis = axis / n if n > 1e-12 else np.array([1.0, 0.0, 0.0])
        return axis * theta

    w = np.array([R[2, 1] - R[1, 2],
                  R[0, 2] - R[2, 0],
                  R[1, 0] - R[0, 1]], dtype=float) / (2.0 * np.sin(theta))
    return w * theta

def pose_to_T(p):
    x, y, z, rx, ry, rz = map(float, p)
    T = np.eye(4)
    T[:3, :3] = rotvec_to_R([rx, ry, rz])
    T[:3,  3] = [x, y, z]
    return T

def wrench_trans(wrench, tcp_pose_base, base_to_tcp=True, include_translation=True):
    """
    Transform a 6D wrench between base and TCP frames.

    Parameters
    ----------
    wrench : sequence[float]
        [Fx, Fy, Fz, Mx, My, Mz] expressed in the source frame.
    tcp_pose_base : sequence[float]
        UR pose [x, y, z, rx, ry, rz] of TCP in base coordinates
        (i.e. T_base_tcp).
    base_to_tcp : bool
        True  -> source is base frame, destination is TCP frame.
        False -> source is TCP frame, destination is base frame.
    include_translation : bool
        If True, apply full wrench translation (moment arm term p x f).
        If False, apply rotation only.

    Returns
    -------
    list[float]
        Transformed wrench [Fx, Fy, Fz, Mx, My, Mz].
    """
    if wrench is None or len(wrench) < 6:
        raise ValueError("wrench must be a 6-element sequence")
    if tcp_pose_base is None or len(tcp_pose_base) < 6:
        raise ValueError("tcp_pose_base must be a 6-element pose")

    T_base_tcp = pose_to_T(tcp_pose_base)
    R_base_tcp = T_base_tcp[:3, :3]
    p_base_tcp = T_base_tcp[:3, 3]

    wrench = np.asarray(wrench[:6], dtype=float)
    f_src = wrench[:3]
    m_src = wrench[3:]

    if base_to_tcp:
        # base -> tcp
        f_dst = R_base_tcp.T @ f_src
        if include_translation:
            m_dst = R_base_tcp.T @ (m_src - np.cross(p_base_tcp, f_src))
        else:
            m_dst = R_base_tcp.T @ m_src
    else:
        # tcp -> base
        f_dst = R_base_tcp @ f_src
        if include_translation:
            m_dst = np.cross(p_base_tcp, f_dst) + (R_base_tcp @ m_src)
        else:
            m_dst = R_base_tcp @ m_src

    return list(np.concatenate([f_dst, m_dst]))

def wrench_tans(wrench, tcp_pose_base, base_to_tcp=True, include_translation=True):
    """Backward-compatible alias for a common typo of `wrench_trans`."""
    return wrench_trans(
        wrench,
        tcp_pose_base,
        base_to_tcp=base_to_tcp,
        include_translation=include_translation,
    )

def T_to_pose(T):
    T = np.asarray(T, dtype=float)
    x, y, z = T[:3, 3]
    rx, ry, rz = R_to_rotvec(T[:3, :3])
    return np.array([x, y, z, rx, ry, rz], dtype=float)

def flip_matrix(axis="y"):
    # 180° rotation around chosen axis
    if axis == "x":
        R = np.array([[1, 0, 0],
                      [0,-1, 0],
                      [0, 0,-1]], dtype=float)
    elif axis == "y":
        R = np.array([[-1, 0, 0],
                      [ 0, 1, 0],
                      [ 0, 0,-1]], dtype=float)
    elif axis == "z":
        R = np.array([[-1, 0, 0],
                      [ 0,-1, 0],
                      [ 0, 0, 1]], dtype=float)
    else:
        raise ValueError("axis must be one of: 'x', 'y', 'z'")
    F = np.eye(4)
    F[:3, :3] = R
    return F

def translate_matrix(distance = 0.0):
    F = np.eye(4)
    F[2, 3] = distance  # Translate along Z-axis (assuming distance is along Z)
    return F

def calculate_ayal_in_uri(P_uri_when_calib, P_ayal_when_calib, flip_axis="y"):
    # P_uri_when_calib  = [-0.050598, -0.494921, 0.400037, -0.033123, -2.16515, 2.21521]
    # P_ayal_when_calib = [-0.549835,  0.47687,  0.128805, -1.56539,  0.057976, -0.015374]
    
    P_uri_when_calib = [f"{x:.4f}" for x in P_uri_when_calib]
    P_ayal_when_calib = [f"{x:.4f}" for x in P_ayal_when_calib]

    print("Calculating Ayal base pose in Uri frame...")
    print(f"P_uri_when_calib: {P_uri_when_calib}")
    print(f"P_ayal_when_calib: {P_ayal_when_calib}")
    T_U_TCP = pose_to_T(P_uri_when_calib)     # Uri base -> Uri TCP
    T_A_TCP = pose_to_T(P_ayal_when_calib)    # Ayal base -> Ayal TCP

    F = flip_matrix(axis=flip_axis)                 # TCP flip (180° about Y)

    # If TCPs coincide and differ by flip F, then:
    # T_U_A (Ayal base expressed in Uri base) = T_U_TCP @ F @ inv(T_A_TCP)
    T_U_A = T_U_TCP @ F @ np.linalg.inv(T_A_TCP)

    pose_ayal_in_uri = T_to_pose(T_U_A)
    return pose_ayal_in_uri

def calculate_mirror_position(P_source, P_BaseT2BaseS, flip_axis="y", flip_trans=False, translation=0.0):
    print("Calculating target pose corresponding to source pose...")
    print(f"P_source: ", [f'{x:.4f}' for x in P_source])
    T_source = pose_to_T(P_source)

    T_translation = translate_matrix(distance=translation)
    F = flip_matrix(axis=flip_axis)

    T_BaseT2BaseS = pose_to_T(P_BaseT2BaseS)
    if flip_trans:
        T_BaseT2BaseS = np.linalg.inv(T_BaseT2BaseS)
    T_target = T_BaseT2BaseS @ T_source @ T_translation @ F
    P_target = T_to_pose(T_target)
    return P_target

def _is_pose_like(value):
    try:
        return isinstance(value, (list, tuple, np.ndarray)) and len(value) == 6 and all(v is not None for v in value)
    except Exception:
        return False

def calculate_optimal_calibration(calibration_file, single_calibration=None):
    """
    Load calibration data from file and return a calibration pose.

    Supported formats:
    - single pose: [x, y, z, rx, ry, rz]
    - list of calibration samples:
      [{"P_uri_when_calib": [...], "P_ayal_when_calib": [...]}, ...]

    If `single_calibration` is True, the file is treated as a single pose.
    If `single_calibration` is False, the file is treated as a list of samples.
    If `single_calibration` is None, the format is auto-detected.

    Minimizes: sum(||p_i - p*||) where ||.|| is Euclidean distance
    """
    try:
        with open(calibration_file, 'r') as f:
            data = json.load(f)
        all_poses = []
        all_errors = []

        if single_calibration is True or (single_calibration is None and _is_pose_like(data)):
            if not _is_pose_like(data):
                print("Error: expected a single calibration pose [x, y, z, rx, ry, rz]")
                return None, None, None

            pose = np.array(data, dtype=float)
            print(f"Loaded single calibration pose from {calibration_file}")
            print(f"  Pose: {pose}")
            return pose, [pose], 0.0

        if single_calibration is False or single_calibration is None:
            if not isinstance(data, list) or len(data) == 0:
                print("Error: calibration file must contain a list of calibration samples")
                return None, None, None

            print(f"Loaded {len(data)} calibration samples from {calibration_file}")

        # Calculate transformation matrix for each pair
        
        for i, sample in enumerate(data):
            try:
                if not isinstance(sample, dict):
                    print(f"  Sample {i}: Expected an object with calibration poses, skipping")
                    continue

                P_uri = sample.get("P_uri_when_calib")
                P_ayal = sample.get("P_ayal_when_calib")
                
                if P_uri is None or P_ayal is None:
                    print(f"  Sample {i}: Missing calibration data, skipping")
                    continue
                
                pose = calculate_ayal_in_uri(P_uri, P_ayal)
                all_poses.append(pose)
                print(f"  Sample {i}: {pose}")
            except Exception as e:
                print(f"  Sample {i}: Error calculating pose: {e}")
                continue
        
        if len(all_poses) < 2:
            print("Error: Need at least 2 valid calibration samples")
            return None, None, None
        
        # Convert to numpy array
        all_poses_array = np.array(all_poses)
        
        def total_distance(current_point, data_points):
            """Objective function: sum of Euclidean distances to all data points"""
            diff = data_points - current_point
            distances = np.linalg.norm(diff, axis=1)
            return np.sum(distances)
        
        # Use arithmetic mean as initial guess
        initial_guess = np.mean(all_poses_array, axis=0)
        
        # Optimize to find geometric median
        result = minimize(total_distance, initial_guess, args=(all_poses_array,))
        
        optimal_pose = result.x
        optimal_distance = result.fun
        
        print(f"\nOptimal calibration (geometric median):")
        print(f"  Pose: {optimal_pose}")
        print(f"  Total distance: {optimal_distance:.6f}")
        print(f"  Arithmetic mean: {initial_guess}")
        
        return optimal_pose, all_poses, optimal_distance
            
    except FileNotFoundError:
        print(f"Error: calibration file '{calibration_file}' not found")
        return None, None, None
    except json.JSONDecodeError:
        print(f"Error: calibration file is not valid JSON")
        return None, None, None
    except Exception as e:
        print(f"Error loading calibration file: {e}")
        return None, None, None

def test_calibration_calculation():
    # Example usage
    P_uri_when_calib = [0.1806, -0.5127, 0.3625, -0.0403, -0.0213, 3.1149]
    P_ayal_when_calib = [-0.3146, 0.4500, 0.0917, 0.0189, 3.1227, 0.0452]
    print(calculate_ayal_in_uri(P_uri_when_calib, P_ayal_when_calib, flip_axis="x"))

if __name__ == "__main__":
    # import argparse
    # parser = argparse.ArgumentParser(description="Calculate optimal calibration from data file")
    # parser.add_argument("calibration_file", help="Path to JSON file containing calibration samples")
    # args = parser.parse_args()
    
    # optimal_pose, all_poses, all_errors = calculate_optimal_calibration(args.calibration_file)
    
    test_calibration_calculation()
    # uri_pose = [0.174295, -0.408903, 0.413234, 0.000152, 3.7e-05, 0.000165]
    # relative_pose = [0.497726, -0.961106, 0.273767, 3.9e-05, 1.2e-05, 0.003888]
    # mirror_pose = calculate_mirror_position(uri_pose, relative_pose, flip_trans=True)
    # print(f"Mirror pose: [{', '.join(f'{v:.8f}' for v in mirror_pose)}]")
