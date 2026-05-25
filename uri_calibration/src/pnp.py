"""
Dual-robot apple pick-and-place scenario:
- Robot URI and Robot AYAL work sequentially to move an apple between positions
- No camera: apple position is known a priori
- Sequence: URI pick → place at 0.1,0.1 → AYAL pick → place at -0.1,-0.1 → URI pick
"""

import uri_if
import numpy as np
from uri_gui.src.config import *
from uri_gui.src.control import *

# region Configuration Constants
# Transformation: T_uri_base_to_ayal_base (translation only for now)
# Format: [dx, dy, dz, rx, ry, rz] in shared/world frame
P_URI_TO_AYAL = np.array([0.49314396, -0.96987269,  0.26870739, -0.00682097,  0.01358239, -0.00351815])
# Auto mode flag (if True, no user prompts)
GLOBAL_AUTO_MODE = False

# Reference frame: table surface
TABLE_Z = 0.0

# Apple geometry
APPLE_HEIGHT = 0.06
APPLE_GRASP_Z = TABLE_Z + APPLE_HEIGHT / 2  # z=0.03, middle of apple
CARRY_Z = 0.08  # intermediate carry height

# Gripper
GRIPPER_ACCURACY = 15  # position accuracy
GRIPPER_HOLD_POS = 167  # closed position
GRIPPER_OPEN_POS = GRIPPER_HOLD_POS - GRIPPER_ACCURACY  # open position
GRIPPER_FULLY_OPEN_POS = 0  # fully open
GRIPPER_SPEED = 255
GRIPPER_FORCE = 10

# Tool orientation (always same)
global TOOL_RX, TOOL_RY, TOOL_RZ
TOOL_RX, TOOL_RY, TOOL_RZ = 0.0, np.pi, 0.0

# Waypoints in shared frame
INITIAL_X, INITIAL_Y = 0.083927, -0.545546  # initial apple position
OFFSET = 0.05  # 5cm offset for pick-and-place
WAYPOINT_INITIAL = np.array([INITIAL_X, INITIAL_Y, APPLE_GRASP_Z]) # initial apple position
WAYPOINT_1 = np.array([INITIAL_X+OFFSET, INITIAL_Y+OFFSET, APPLE_GRASP_Z]) # moving 5cm in x and y
WAYPOINT_2 = np.array([INITIAL_X-OFFSET, INITIAL_Y-OFFSET, APPLE_GRASP_Z]) # moving 5cm in x and y

# Base joint positions
BASE_JOINTS = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
URI_BASE_JOINTS_DEG = [-123.672094, -72.410748, -128.335102, -69.282289, 90.042589, 16.829668]
AYAL_BASE_JOINTS_DEG = [-25.390153, -59.609876, 128.950027, 21.265393, 89.605819, 25.480758]
URI_BASE_JOINTS = [angle * np.pi / 180 for angle in URI_BASE_JOINTS_DEG]
AYAL_BASE_JOINTS = [angle * np.pi / 180 for angle in AYAL_BASE_JOINTS_DEG]

# endregion

# region Helper Functions
def transform_point_uri_to_ayal(P_in_uri_frame):
    """
    Transform a 3D point from URI base frame to AYAL base frame.
    Assumes T_URI_TO_AYAL is [dx, dy, dz, rx, ry, rz] with optional rotation.
    For now, only translation is applied.
    """
    T_uri_to_ayal = utils.pose_to_T(P_URI_TO_AYAL)
    T_in_uri_frame = utils.pose_to_T(P_in_uri_frame)
    T_in_ayal_frame = np.linalg.inv(T_uri_to_ayal) @ T_in_uri_frame
    P_in_ayal_frame = utils.T_to_pose(T_in_ayal_frame)
    return P_in_ayal_frame

def move_to_tcp_pose(uri, x, y, z, rx, ry, rz, description=""):
    """
    Move robot to a TCP pose using moveJ_IK (joint-space inverse kinematics).
    """
    print(f"  {description} → TCP({x:.4f}, {y:.4f}, {z:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f})")
    tcp_movej(uri, x, y, z, rx, ry, rz)
    # time.sleep(0.5)  # brief pause to ensure arrival

def approach_point(uri, x, y, z, rx, ry, rz, description=""):
    """
    Move to approach position (above the target point).
    z: height offset (0 for URI, P_URI_TO_AYAL[2] for AYAL)
    """
    z_approach = z + CARRY_Z
    move_to_tcp_pose(uri, x, y, z_approach, rx, ry, rz, 
                     description=f"Approach {description}")

def rand_tool_orientation(uri):
    """
    Randomize wrist3
    """
    q_pose = uri.recieve.getActualQ()
    base, shoulder, elbow, wrist1, wrist2, wrist3 = q_pose
    wrist3 = np.random.uniform(-np.pi, np.pi)
    q_movej(uri, base, shoulder, elbow, wrist1, wrist2, wrist3)
    x, y, z, rx, ry, rz = get_tcp_pose(uri) 
    print(f"    Randomized wrist3 to {wrist3:.4f} rad")
    print(f"    Randomized rx, ry, rz to {rx:.4f}, {ry:.4f}, {rz:.4f} rad")
    return rx, ry, rz

def grasp_and_lift(uri, x, y, z, rx, ry, rz, description=""):
    """
    Descend to grasp point, close gripper, and lift to carry height.
    z: height offset (0 for URI, P_URI_TO_AYAL[2] for AYAL)
    """
    # Descend to grasp
    move_to_tcp_pose(uri, x, y, z, rx, ry, rz,
                     description=f"Descend to {description}")
    # time.sleep(0.3)
    
    # Close gripper
    print(f"    Closing gripper...")
    close_gripper(uri)
    # time.sleep(0.5)
    
    # Lift to carry height
    move_to_tcp_pose(uri, x, y, CARRY_Z+z, rx, ry, rz,
                     description=f"Lift from {description}")

def move_and_place(uri, x_to, y_to, z, rx, ry, rz, description_from="", description_to=""):
    """
    Move from one horizontal position to another at carry height, then descend and release.
    """
    # Move at carry height to destination
    move_to_tcp_pose(uri, x_to, y_to, CARRY_Z+z, rx, ry, rz,
                     description=f"Carry from {description_from} to {description_to}")
    
    # Descend to place
    move_to_tcp_pose(uri, x_to, y_to, z, rx, ry, rz,
                     description=f"Place at {description_to}")
    # time.sleep(0.3)
    
    # Open gripper
    print(f"    Opening gripper...")
    move_gripper(uri, GRIPPER_OPEN_POS, GRIPPER_SPEED, GRIPPER_FORCE)  # Ensure gripper is open
    # time.sleep(0.5)
    
    # Retreat to approach height
    move_to_tcp_pose(uri, x_to, y_to, CARRY_Z + z, rx, ry, rz,
                     description=f"Retreat from {description_to}")

def return_to_base(uri, base_joints, description=""):
    """
    Move robot back to base joint position.
    """
    print(f"  Moving {description} to base position...")
    q_movej(uri, *base_joints)
    # time.sleep(1.0)

def wait_for_user(prompt="Press Enter to continue, 's' to skip, any other key to abort: "):
    """
    Prompt user for input.
    Returns: "continue" | "skip" | "abort".
    """
    user_input = input(prompt).strip().lower()
    if user_input == "":
        return "continue"
    if user_input == "s":
        return "skip"
    print("❌ User aborted scenario.")
    return "abort"

def gate(step_name: str):
    """
    Helper to handle user input actions for each step.
    Returns True if step should run, False if it should be skipped.
    Raises SystemExit-style return to abort upstream.
    """
    if GLOBAL_AUTO_MODE:
        print(f"⏩ Auto mode: proceeding with {step_name} ...")
        return True
    action = wait_for_user()
    if action == "abort":
        raise SystemExit("Aborted by user")
    if action == "skip":
        print(f"⏭️ Skipping {step_name} ...")
        return False
    return True

def close_gripper(uri):
    """
    PLACEHOLDER: User to implement actual gripper close logic.
    Example:
        close_gripper(uri, GRIPPER_SPEED, GRIPPER_FORCE)
    """
    uri.gripper.close(GRIPPER_SPEED, GRIPPER_FORCE)

def open_gripper(uri):
    """
    PLACEHOLDER: User to implement actual gripper open logic.
    Example:
        open_gripper(uri, GRIPPER_SPEED, GRIPPER_FORCE)
        # or move_gripper(uri, GRIPPER_OPEN_POS, GRIPPER_SPEED, GRIPPER_FORCE)
    """
    uri.gripper.open(GRIPPER_SPEED, GRIPPER_FORCE)
# endregion

def run_dual_robot_pnp(uri_robot, ayal_robot):
    """
    Execute the full 7-step pick-and-place scenario.
    
    Steps:
    0. Both arms at base.
    1. URI: pick apple at (0, 0).
    2. URI: place apple at (0.1, 0.1).
    3. URI: return to base.
    4. AYAL: pick apple at (0.1, 0.1).
    5. AYAL: place apple at (-0.1, -0.1).
    6. AYAL: return to base.
    7. URI: pick apple at (-0.1, -0.1).
    """
    
    try:
        print("\n" + "="*70)
        print("DUAL-ROBOT SCENARIO: Pick-and-Place with URI and AYAL")
        print("="*70)
        
        # region Step 0: Both to base
        print("\nStep 0: Moving both robots to base position...")
        if gate("Step 0"):
            return_to_base(uri_robot, URI_BASE_JOINTS, "URI")
            return_to_base(ayal_robot, AYAL_BASE_JOINTS, "AYAL")
            close_gripper(uri_robot)
            close_gripper(ayal_robot)
            # time.sleep(1.0)
        # endregion
        
        # region Step 1: URI prepares to pick apple at WAYPOINT_INITIAL
        TOOL_RX, TOOL_RY, TOOL_RZ = 0.0, np.pi, 0.0
        waypoint_initial_full = np.concatenate([WAYPOINT_INITIAL.copy(), [TOOL_RX, TOOL_RY, TOOL_RZ]])
        print("\nStep 1: URI approaching apple at", waypoint_initial_full, "...")
        if gate("Step 1"):
            approach_point(uri_robot, *waypoint_initial_full, description="apple at origin")
            open_gripper(uri_robot)  # Ensure gripper is open
        # endregion

        # region Step 2: URI picks apple at WAYPOINT_INITIAL
        print("\nStep 2: URI picking apple at", waypoint_initial_full, "...")
        if gate("Step 2"):
            grasp_and_lift(uri_robot, *waypoint_initial_full, description="origin")
        # endregion
        
        # region Step 3: URI places apple at WAYPOINT_1
        TOOL_RX, TOOL_RY, TOOL_RZ =rand_tool_orientation(uri_robot)          
        waypoint_1_full = np.concatenate([WAYPOINT_1.copy(), [TOOL_RX, TOOL_RY, TOOL_RZ]])
        print("\nStep 3: URI placing apple at", waypoint_1_full, "...")
        if gate("Step 3"):
            move_and_place(uri_robot, *waypoint_1_full,
                           description_from="origin", description_to="URI target")
        # endregion
        
        # region Step 4: URI returns to base
        print("\nStep 4: URI returning to base...")
        if gate("Step 4"):
            return_to_base(uri_robot, URI_BASE_JOINTS, "URI")
            # time.sleep(1.0)
        # endregion
        
        # region Step 5: AYAL approaches apple at WAYPOINT_1
        waypoint_1_ayal_full = transform_point_uri_to_ayal(waypoint_1_full)
        print("\nStep 5: AYAL approaching apple at", waypoint_1_full, "[URI frame]...")
        if gate("Step 5"):
            print(f"  Transformed URI ({waypoint_1_full[0]:.4f}, {waypoint_1_full[1]:.4f}, {waypoint_1_full[2]:.4f}) → "
                f"AYAL ({waypoint_1_ayal_full[0]:.4f}, {waypoint_1_ayal_full[1]:.4f}, {waypoint_1_ayal_full[2]:.4f})")
            
            approach_point(ayal_robot, *waypoint_1_ayal_full, description="apple (in AYAL frame)")
            move_gripper(ayal_robot, GRIPPER_OPEN_POS, GRIPPER_SPEED, GRIPPER_FORCE)  # Ensure gripper is open
        # endregion

        # region Step 6: AYAL grasps apple at (0.183927, -0.645546)
        print("\nStep 6: AYAL picking apple at", waypoint_1_ayal_full, "[AYAL frame]...")
        if gate("Step 6"):
            grasp_and_lift( ayal_robot, *waypoint_1_ayal_full, description="apple (in AYAL frame)")
        # endregion

        # region Step 7: AYAL places apple at WAYPOINT_2
        # rand_tool_rz()  # randomize tool RZ before placing
        waypoint_2_full = np.concatenate([WAYPOINT_2.copy(), [TOOL_RX, TOOL_RY, TOOL_RZ]])
        waypoint_2_ayal_full = transform_point_uri_to_ayal(waypoint_2_full)
        print("\nStep 7: AYAL placing apple at", waypoint_2_ayal_full, "[AYAL frame]...")
        if gate("Step 7"):
            move_and_place(ayal_robot,
                           *waypoint_2_ayal_full,
                            description_from="pick point", description_to="AYAL target")
                        #    P_URI_TO_AYAL[2], description_from="pick point", description_to="AYAL target")
        # endregion
        
        # region Step 8: AYAL returns to base
        print("\nStep 8: AYAL returning to base...")
        if gate("Step 8"):
            return_to_base(ayal_robot, AYAL_BASE_JOINTS, "AYAL")
            # time.sleep(1.0)
        # endregion
        
        # region Step 9: URI picks apple at WAYPOINT_2
        # Transform back to URI frame
        print("\nStep 9: URI picking apple at", waypoint_2_ayal_full, "[AYAL frame]...")
        if gate("Step 9"):            
            approach_point(uri_robot, *waypoint_2_full, description="apple (in URI frame)")
            grasp_and_lift(uri_robot, *waypoint_2_full, description="apple (in URI frame)")
        # endregion

        # region Step 10: URI places apple at WAYPOINT_INITIAL
        print("\nStep 10: URI placing apple back at", WAYPOINT_INITIAL, "...")
        if gate("Step 10"):
            move_and_place( uri_robot,
                            *waypoint_initial_full,
                            description_from="origin", description_to="URI target")
        # endregion

        # region Step 11: URI returns to base
        print("\nStep 11: URI returning to base...")
        if gate("Step 11"):
            return_to_base(uri_robot, URI_BASE_JOINTS, "URI")
            # time.sleep(1.0)
        # endregion

        print("\n" + "="*70)
        print("SCENARIO COMPLETE!")
        print("="*70 + "\n")
        
    except SystemExit as e:
        print(f"\n❌ Scenario aborted: {e}")
        return
    except Exception as e:
        print(f"\n❌ ERROR during scenario: {e}")
        print("Attempting to return both robots to base as safety measure...")
        try:
            return_to_base(uri_robot, URI_BASE_JOINTS, "URI")
            return_to_base(ayal_robot, AYAL_BASE_JOINTS, "AYAL")
        except:
            pass

if __name__ == "__main__":
    # host IPs
    URI_HOST = "192.168.56.101"
    AYAL_HOST = "192.168.57.101"

    uri = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)

    uri.connect(gripper_calibrate=False)
    ayal.connect(gripper_calibrate=False)

    # RUN SCENARIO
    run_dual_robot_pnp(uri, ayal)
    # x, y, z = uri.recieve.getActualTCPPose()[:3]
    # move_to_tcp_pose(uri, x, y, z, TOOL_RX, TOOL_RY, 2.6553, description=f"Approach final position")