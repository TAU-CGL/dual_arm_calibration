import math
import time

import numpy as np
import uri_if
import uri_calibration.src.utils as gemini

# x-axis is towards the operator.
# y-axis is to the right of the operator.
# z-axis is upwards.
# tcp length is 176/178 mm in z direction.

# Parameters
P_ayal_base_in_uri_MES = [0.495, -0.965, 0.265] # meters, measured position of AYAL base in URI frame. +-100mm

# Transformation Matrices
T_ayal_to_uri = np.array([# transformation from AYAL to URI (from gemini_calc_v2.py)
    [ 0.999943234, -0.000815055, -0.010623765,  0.500962858],
    [ 0.001113048,  0.99960523,   0.028073933, -0.974606816],
    [ 0.010596689, -0.028084164,  0.999549393,  0.290508966],
    [ 0.,           0.,           0.,           1.         ]
], dtype=float)
T_uri_to_ayal = np.linalg.inv(T_ayal_to_uri)
P_ayal_base_in_uri_CALC = gemini.T_to_pose(T_ayal_to_uri)[:3] # calculated position of AYAL base in URI frame.

#tcp Poses at calibration time
P_uri_when_calib = [-0.050598, -0.494921, 0.400037, -0.033123, -2.16515, 2.21521]
P_ayal_when_calib = [-0.549835, 0.47687, 0.128805, -1.56539, 0.057976, -0.015374]

# New target poses 
# 1) starting from AYAL pose in AYAL frame
P_ayal_new_in_ayal_BASEframe = [[-0.623225, 0.287135, 0.340592, 0.160499, 2.30297, 1.692002]] # target pose for AYAL in AYAL frame (from gui)
# 2) translating in z-axis to avoid collision
P_uri_new_in_ayal_TCPframe = [[0.0, 0.0, 0.010, 0.0, 0.0, 0.0]] # moving 100mm in z direction in URI TCP frame

T_ayal_new_in_ayal_BASEframe = gemini.pose_to_T(P_ayal_new_in_ayal_BASEframe[0])
T_uri_new_in_ayal_TCPframe = gemini.pose_to_T(P_uri_new_in_ayal_TCPframe[0])
# 3) flipping TCP to create the "kiss"
F = gemini.flip_matrix(axis="y") # TCP flip (180° about Y)

T_uri_new_in_ayal_BASEframe = T_ayal_new_in_ayal_BASEframe @ T_uri_new_in_ayal_TCPframe @ F
# 4) Final transformation to URI base frame
T_uri_new_in_uri_BASEframe = T_ayal_to_uri @ T_uri_new_in_ayal_BASEframe

# host IPs
URI_HOST = "192.168.56.101"
AYAL_HOST = "192.168.57.101"

if __name__ == "__main__":
    print("T_uri_new_in_uri_BASEframe:", gemini.T_to_pose(T_uri_new_in_uri_BASEframe).round(9))
    print("P_ayal_base_in_uri_CALC:", P_ayal_base_in_uri_CALC)
    
    # connect to robots
    uri_robot = uri_if.RMPLAB_Uri(URI_HOST)
    ayal_robot = uri_if.RMPLAB_Uri(AYAL_HOST)
    uri_robot.connect(gripper_calibrate=False)
    ayal_robot.connect(gripper_calibrate=False)

    pose_ayal_in_uri = gemini.calculate_ayal_in_uri(ayal_robot.recieve.getActualTCPPose(), uri_robot.recieve.getActualTCPPose())
    print(pose_ayal_in_uri)