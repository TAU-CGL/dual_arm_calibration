import time

import uri_if

# host IPs
URI_HOST = "192.168.56.101"
AYAL_HOST = "192.168.57.101"

def run_force_mode(robot, duration_s: float = 2.0) -> None:
    task_frame = robot.recieve.getActualTCPPose()
    selection_vector = [0, 0, 1, 0, 0, 0]
    wrench = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]
    force_type = 2
    limits = [0.1, 0.1, 0.15, 0.1, 0.1, 0.1]

    robot.control.forceMode(task_frame, selection_vector, wrench, force_type, limits)
    time.sleep(duration_s)
    if hasattr(robot.control, "forceModeStop"):
        robot.control.forceModeStop()
    else:
        robot.control.stopScript()

def run_force_mode_with_xy_motion(
    robot,
    duration_s: float = 4.0,
    vx: float = 0.02,
    vy: float = 0.0,
) -> None:
    task_frame = robot.recieve.getActualTCPPose()
    selection_vector = [0, 0, 1, 0, 0, 0]
    wrench = [0.0, 0.0, 4.0, 0.0, 0.0, 0.0]
    force_type = 2
    limits = [0.1, 0.1, 0.15, 0.1, 0.1, 0.1]

    robot.control.forceMode(task_frame, selection_vector, wrench, force_type, limits)

    start = time.time()
    if hasattr(robot.control, "speedL"):
        dt = 0.05
        while time.time() - start < duration_s:
            robot.control.speedL([vx, vy, 0.0, 0.0, 0.0, 0.0], 0.2, dt)
            time.sleep(dt)
        if hasattr(robot.control, "stopL"):
            robot.control.stopL(0.2)
    else:
        dt = 0.2
        base_pose = robot.recieve.getActualTCPPose()
        while time.time() - start < duration_s:
            elapsed = time.time() - start
            target = base_pose.copy()
            target[0] += vx * elapsed
            target[1] += vy * elapsed
            robot.control.moveL(target, 0.1, 0.1, False)
            time.sleep(dt)

    if hasattr(robot.control, "forceModeStop"):
        robot.control.forceModeStop()
    else:
        robot.control.stopScript()

if __name__ == "__main__":
    # Connect to robots
    uri = uri_if.RMPLAB_Uri(URI_HOST)
    uri.connect(False)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    ayal.connect(False)

    try:
        run_force_mode_with_xy_motion(uri, duration_s=4.0, vx=0.02, vy=0.0)
    finally:
        uri.disconnect()
        ayal.disconnect()


