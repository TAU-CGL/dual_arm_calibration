#!/usr/bin/env python3
"""
id like you to make a new script. with the following functions: 
1. gets a point xyzrxryrz and a robot uri, and returns if it is valid/non-valid and the manipulabillity of the robot at that point.
2. gets a point xyzrxryrz (in uri's base frame), manipulabillity threshold, and two robots uri and ayal. it returns true if it is a valid piont for uri, and her mirror in Y axis is valid for ayal, and both's manipulabillities are over the threshold. both needs make sure you get the frame transitions right. (check in calibration/gemini_calc_v2.py to see the mirroring). use function 1.
3. gets two points xyzrxryrz (in uri's base frame), manipulabillity threshold, step_size, and two robots uri and ayal. assuming the points are valid, using RRT and function 1/2 to find a route to get from the first point to the second one. returns a series of points in uri's reference frame.
"""

import math
import random
import numpy as np
import roboticstoolbox as rtb
import os
import sys

# Add parents to path for importing local modules
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from uri_calibration.src import utils as gemini

# Fixed UR5e model for manipulability check
_UR5E_MODEL = rtb.models.UR5()

def get_manipulability(q):
    """Compute Yoshikawa manipulability index given joint config."""
    J = _UR5E_MODEL.jacob0(q)
    return float(np.sqrt(max(0.0, np.linalg.det(J @ J.T))))

def check_pose_validity(pose, robot):
    """
    1. Returns a tuple (is_valid, manipulability) for a given pose and robot.
       Pose format: [x, y, z, rx, ry, rz]
    """
    try:
        if not robot.control.getInverseKinematicsHasSolution(pose):
            return False, 0.0
        if not robot.control.isPoseWithinSafetyLimits(pose):
            return False, 0.0
            
        qnear = list(robot.recieve.getActualQ())
        q = robot.control.getInverseKinematics(pose, qnear)
        manip = get_manipulability(q)
        return True, manip
    except RuntimeError:
        return False, 0.0
    except Exception:
        return False, 0.0

def is_valid_dual_pose(pose_uri, threshold, uri_robot, ayal_robot, ayal_in_uri_pose):
    """
    2. Returns True if the pose is valid for Uri and its mirrored pose is valid for Ayal,
       and both manipulabilities are over the threshold.
       Assumes pose_uri is in Uri's base frame.
    """
    uri_valid, uri_manip = check_pose_validity(pose_uri, uri_robot)
    if not uri_valid or uri_manip <= threshold:
        return False

    # Calculate mirrored pose for Ayal
    # flip_trans=True means we are transforming from Uri's frame to Ayal's frame
    pose_ayal = gemini.calculate_mirror_position(
        P_source=pose_uri, 
        P_BaseT2BaseS=ayal_in_uri_pose, 
        flip_axis="y", 
        flip_trans=True
    )

    ayal_valid, ayal_manip = check_pose_validity(pose_ayal, ayal_robot)
    if not ayal_valid or ayal_manip <= threshold:
        return False

    return True

# RRT Helper Functions
def _distance(p1, p2):
    """Euclidean distance based only on translation, or translation + rotation."""
    return np.linalg.norm(np.array(p1[:3]) - np.array(p2[:3]))

def _steer(from_node, to_node, step_size):
    """Returns a new pose stepped in the direction of to_node by step_size."""
    p_from = np.array(from_node)
    p_to = np.array(to_node)
    dist = _distance(from_node, to_node)
    
    if dist < step_size:
        return to_node
        
    # Interpolate linearly (this is a simple Cartesian + Euler interpolation)
    t = step_size / dist
    return (p_from + t * (p_to - p_from)).tolist()

def generate_random_pose(bounds):
    """Bounds is a list of (min, max) for [x, y, z, rx, ry, rz]."""
    return [random.uniform(b[0], b[1]) for b in bounds]

def rrt_path_planner(start_pose, end_pose, threshold, step_size, uri_robot, ayal_robot, ayal_in_uri_pose, max_iter=10000):
    """
    3. Finds a route from start_pose to end_pose using RRT.
       Returns a list of poses in Uri's reference frame.
    """
    # Define search space bounds based approximately on reachable workspace
    # Format: [(x_m, x_M), (y_m, y_M), (z_m, z_M), (rx_m, rx_M), ...]
    bounds = [
        (-0.9, 0.9),   # x
        (-0.9, 0.9),   # y
        (0.0, 0.9),    # z
        (-3.14, 3.14), # rx
        (-3.14, 3.14), # ry
        (-3.14, 3.14)  # rz
    ]

    class Node:
        def __init__(self, pose):
            self.pose = pose
            self.parent = None

    start_node = Node(start_pose)
    tree = [start_node]

    # Verify start and end are valid
    if not is_valid_dual_pose(start_pose, threshold, uri_robot, ayal_robot, ayal_in_uri_pose):
        print("Start pose is invalid or below threshold.")
        return []
    if not is_valid_dual_pose(end_pose, threshold, uri_robot, ayal_robot, ayal_in_uri_pose):
        print("End pose is invalid or below threshold.")
        return []

    for i in range(max_iter):
        # 10% of the time, sample the end pose directly to bias the search
        if random.random() < 0.1:
            rand_pose = end_pose
        else:
            rand_pose = generate_random_pose(bounds)

        # Find nearest node in tree
        nearest_node = tree[0]
        min_dist = _distance(nearest_node.pose, rand_pose)
        for node in tree[1:]:
            d = _distance(node.pose, rand_pose)
            if d < min_dist:
                min_dist = d
                nearest_node = node

        # Steer towards rand_pose
        new_pose = _steer(nearest_node.pose, rand_pose, step_size)

        # Check validity
        if is_valid_dual_pose(new_pose, threshold, uri_robot, ayal_robot, ayal_in_uri_pose):
            new_node = Node(new_pose)
            new_node.parent = nearest_node
            tree.append(new_node)

            # Check if we reached the goal
            if _distance(new_pose, end_pose) <= step_size:
                # Add final goal node
                goal_node = Node(end_pose)
                goal_node.parent = new_node
                
                # Check goal connection validity
                if is_valid_dual_pose(end_pose, threshold, uri_robot, ayal_robot, ayal_in_uri_pose):
                    tree.append(goal_node)
                    
                    # Backtrack to build path
                    path = []
                    curr = goal_node
                    while curr is not None:
                        path.append(curr.pose)
                        curr = curr.parent
                    
                    return path[::-1] # Reverse the path

    print("RRT failed to find a path within the max iterations.")
    return []

if __name__ == "__main__":
    import uri_if
    
    print("Dual Robot RRT Planner Example Usage")
    
    URI_HOST = "192.168.56.101"
    AYAL_HOST = "192.168.57.101" # commonly used for Ayal
    CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), '../../uris_gui/calibration.json')
    USE_SINGLE_CALIBRATION = True

    # Load calibration to get ayal_in_uri_pose
    print("Loading optimal calibration pose...")
    optimal_pose, _, _ = gemini.calculate_optimal_calibration(
        CALIBRATION_FILE,
        single_calibration=USE_SINGLE_CALIBRATION,
    )
    if optimal_pose is None:
        print("Failed to load calibration. Exiting.")
        sys.exit(1)
        
    ayal_in_uri_pose = list(optimal_pose)

    print("Connecting to robots...")
    uri_robot = uri_if.RMPLAB_Uri(URI_HOST)
    ayal_robot = uri_if.RMPLAB_Uri(AYAL_HOST)
    
    try:
        uri_robot.connect(False)
        ayal_robot.connect(False)
    except Exception as e:
        print(f"Error connecting to robots: {e}")
        sys.exit(1)

    try:
        start_pose = list(uri_robot.recieve.getActualTCPPose())
        # Example end_pose, translated 10cm along X from start
        end_pose = start_pose.copy()
        end_pose[0] += 0.1

        threshold = 0.0035
        step_size = 0.05
        max_iter = 1000

        print(f"Start Pose: {start_pose}")
        print(f"End Pose:   {end_pose}")
        print("Planning path...")

        path = rrt_path_planner(
            start_pose=start_pose, 
            end_pose=end_pose, 
            threshold=threshold, 
            step_size=step_size, 
            uri_robot=uri_robot, 
            ayal_robot=ayal_robot, 
            ayal_in_uri_pose=ayal_in_uri_pose,
            max_iter=max_iter
        )

        if path:
            print(f"Path found with {len(path)} waypoints:")
            for i, p in enumerate(path):
                print(f"  {i}: {p}")
        else:
            print("No path could be found.")
            
    finally:
        uri_robot.disconnect()
        ayal_robot.disconnect()
        print("Disconnected.")
