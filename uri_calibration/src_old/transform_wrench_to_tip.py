import numpy as np

def transform_wrench_to_tcp(wrench_flange, tcp_offset):
    """
    Transforms a 6D wrench (forces and torques) from the UR5e flange sensor to the TCP.
    
    Parameters:
    wrench_flange: [Fx, Fy, Fz, Tx, Ty, Tz] raw reading from the sensor (forces in N, torques in Nm).
    tcp_offset:    [x, y, z] distance from the sensor face to the peg tip, in meters.
    
    Returns:
    np.array:      [Fx, Fy, Fz, Tx, Ty, Tz] true forces and torques acting at the TCP.
    """
    # 1. Split the raw 6D wrench into Force (3D) and Torque (3D) vectors
    f_flange = np.array(wrench_flange[:3])
    t_flange = np.array(wrench_flange[3:])
    
    # 2. Extract the tool offset distances
    px, py, pz = tcp_offset
    
    # 3. Create the Skew-Symmetric matrix for the offset vector
    # This is the matrix equivalent of setting up a cross product
    p_skew = np.array([
        [  0, -pz,  py],
        [ pz,   0, -px],
        [-py,  px,   0]
    ])
    
    # 4. Calculate the real TCP Torque
    # We subtract the "lever arm" effect that the physical tool created at the wrist
    t_tcp = t_flange - (p_skew @ f_flange)
    
    # 5. The forces remain identical (a push at the tip is a push at the wrist)
    f_tcp = f_flange
    
    # 6. Recombine into a clean 6D array
    return np.concatenate((f_tcp, t_tcp))

# ==========================================
# --- Example: Removing a Phantom Torque ---
# ==========================================

if __name__ == "__main__":
    # Scenario: The robot pushes the peg slightly sideways against the hole.
    # The sensor feels 10N of force in X. 
    # Because the peg is 150mm (0.15m) long, the sensor also feels a "phantom" torque of 1.5 Nm in Y.
    raw_sensor_reading = [10.0, 0.0, -20.0, 0.0, 1.5, 0.0]  
    
    # The distance from the sensor face to the tip of the peg (e.g., straight down Z)
    tool_offset = [0.0, 0.0, 0.15] 
    
    # Process the data
    clean_tcp_wrench = transform_wrench_to_tcp(raw_sensor_reading, tool_offset)
    
    print("Raw Sensor Wrench: ", [round(num, 2) for num in raw_sensor_reading])
    print("Clean TCP Wrench:  ", [round(num, 2) for num in clean_tcp_wrench])