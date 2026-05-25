"""
Robot backend initialization and command handling for GUI.

Provides a unified RobotBackend interface with SimBackend and RealBackend
implementations, hiding concurrency details (process vs thread) from the GUI.
"""

import os
import sys
import time
import threading
import multiprocessing
import numpy as np
from abc import ABC, abstractmethod
from queue import Empty

_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _RMP_LAB_ROOT in sys.path:
    sys.path.remove(_RMP_LAB_ROOT)
sys.path.insert(0, _RMP_LAB_ROOT)

try:
    from uri_if import RMPLAB_Uri, HOST_AYAL, HOST_URI
except ImportError:
    print("Warning: rmplab_uri not available. Real robot backend disabled.")
    RMPLAB_Uri = None

try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "alpha_puzzle"))
    import dismantle_puzzle
    PUZZLE_AVAILABLE = True
except ImportError:
    print("Warning: alpha_puzzle not available. Puzzle commands disabled.")
    PUZZLE_AVAILABLE = False

ROBOTFLOW_DIR = "uri_sim/pybullet_ur5_robotiq-robotflow"


# =============================================================================
# Abstract Backend
# =============================================================================
class RobotBackend(ABC):
    """Unified interface for sim and real robot backends."""

    @abstractmethod
    def send_command(self, cmd: dict) -> None:
        """Send a command dict to the backend."""
        ...

    @abstractmethod
    def poll_status(self) -> dict | None:
        """Non-blocking poll for a status/response dict. Returns None if empty."""
        ...

    @abstractmethod
    def shutdown(self) -> None:
        """Gracefully shut down the backend."""
        ...

# =============================================================================
# Puzzle Action Handler (for real robots)
# =============================================================================
def handler__puzzle_action(action: str, robots: dict, cmd: dict) -> None:
    """
    TODO move loading trajectories to init or something - no need to load them every time
    Handle complex puzzle actions (prepare, assemble, dismantle, finish).
    Runs in a separate thread context.
    """
    alpha_puzzle_dir = os.path.join(os.path.dirname(__file__), "..", "alpha_puzzle")
    sys.path.insert(0, alpha_puzzle_dir)

    try:
        import dismantle_puzzle  # pyright: ignore[reportMissingImports]
    except ImportError:
        print("Warning: alpha_puzzle not available. Puzzle commands disabled.")
        print(f"[puzzle] {action} not available - puzzle module not loaded")
        return
    
    try:
        ayal = robots.get("ayal")
        uri = robots.get("uri")
        
        if not (ayal and uri):
            print("[puzzle] Both robots required for puzzle actions")
            return
        
        # Common parameters
        cycles = cmd.get("cycles")
        
        dynamic_path = os.path.join(alpha_puzzle_dir, "paths/alphaZ/improved_dynamic_ik_trajectory.csv")
        static_path = os.path.join(alpha_puzzle_dir, "paths/alphaZ/improved_static_ik_trajectory.csv")
        
        match action:
            case "prepare_puzzle":
                print("[puzzle] Preparing puzzle...")
                dismantle_puzzle.prepare_robots(uri, ayal)
                print("[puzzle] Puzzle preparation complete")
            
            case "assemble_puzzle":
                print(f"[puzzle] Assembling puzzle ({cycles} cycles)...")
                # Load trajectories
                dynamic_traj = dismantle_puzzle.csv_to_arm_path(dynamic_path)
                static_traj = dismantle_puzzle.csv_to_arm_path(static_path)
                for cycle in range(cycles):
                    print(f"[puzzle] Cycle {cycle + 1}/{cycles}")
                    dismantle_puzzle.forward_trajectory(uri, ayal, dynamic_traj, static_traj)
                print("[puzzle] Puzzle assembly complete")
            
            case "dismantle_puzzle":
                print(f"[puzzle] Dismantling puzzle ({cycles} cycles)...")
                # Load trajectories
                dynamic_traj = dismantle_puzzle.csv_to_arm_path(dynamic_path)
                static_traj = dismantle_puzzle.csv_to_arm_path(static_path)
                for cycle in range(cycles):
                    print(f"[puzzle] Cycle {cycle + 1}/{cycles}")
                    dismantle_puzzle.reversed_trajectory(uri, ayal, dynamic_traj, static_traj)
                print("[puzzle] Puzzle dismantle complete")
            
            case "finish_puzzle":
                print("[puzzle] Finishing puzzle...")
                dismantle_puzzle.finish_robots(uri, ayal)
                print("[puzzle] Puzzle finish complete")
    
    except Exception as e:
        print(f"[puzzle] Error in {action}: {e}")
        import traceback
        traceback.print_exc()

# =============================================================================
# Simulation Backend (multiprocessing)
# =============================================================================
def _sim_run_process(command_queue: multiprocessing.Queue, status_queue: multiprocessing.Queue, config: dict) -> None:
    """
    TODO should add signal for when sim is ready to receive commands - that way i can avoid sending commands before the sim is ready and having them get lost
    TODO should consider using a more robust IPC mechanism if the command volume increases - that way i can ensure reliable communication between the GUI and sim even under heavy load
    Runs in a separate process. Creates PyBullet sim and steps it, reading
    commands from the queue. Exits when it receives {"action": "quit"}.
    """
    import pybullet as p  # pyright: ignore[reportMissingImports]
    import uri_gui_pyqt5.src.sim as s
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
    uri_sim_dir = os.path.join(repo_root, "uri_sim")
    robotflow_dir = os.path.join(repo_root, ROBOTFLOW_DIR)
    os.chdir(uri_sim_dir)
    sys.path.insert(0, robotflow_dir)

    sim = s.multiRobotsSim(config)
    sim_render_res = config["config_data"]["sim"]["sim_render_res"]
    try:
        while sim.is_connected():
            try:
                cmd = command_queue.get_nowait()
                if cmd["action"] == "quit":
                    status_queue.put({"action": "quit", "status": "ok"})
                    break
                response = sim.mapCommand(cmd)
                if status_queue is not None:
                    status_queue.put(response)
            except Empty:
                pass
            sim.step()
            time.sleep(sim_render_res)
    finally:
        if hasattr(p, "disconnect") and p.isConnected():
            p.disconnect()


class SimBackend(RobotBackend):
    """Simulation backend — runs PyBullet in a separate process."""

    def __init__(self, config: dict):
        print("[backends] Starting simulation backend...")
        self.cmd_queue = multiprocessing.Queue()
        self.status_queue = multiprocessing.Queue()
        self.process = multiprocessing.Process(
            target=_sim_run_process,
            args=(self.cmd_queue, self.status_queue, config),
        )
        self.process.start()

    def send_command(self, cmd: dict) -> None:
        self.cmd_queue.put(cmd)

    def poll_status(self) -> dict | None:
        try:
            return self.status_queue.get_nowait()
        except Empty:
            return None

    def shutdown(self) -> None:
        self.cmd_queue.put({"action": "quit"})
        self.process.join(timeout=2.0)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=1.0)


class InProcessSimBackend(RobotBackend):
    """In-process PyBullet sim — no subprocess, no queue.

    Used by script/headless mode so a user script can drive the sim robots
    directly through ``uri_if.RMPLAB_Uri`` (same API as the real robots).
    The sim's ``register_robot`` calls populate the in-process registry
    that ``RMPLAB_Uri.connect()`` reads, so the same script body works in
    both sim and real modes.
    """

    def __init__(self, config: dict):
        print("[backends] Starting in-process simulation backend...")
        import uri_gui_pyqt5.src.sim as s
        self._sim = s.multiRobotsSim(config)
        self._last_status: dict | None = None

    def send_command(self, cmd: dict) -> None:
        self._last_status = self._sim.mapCommand(cmd)

    def poll_status(self) -> dict | None:
        result, self._last_status = self._last_status, None
        return result

    def shutdown(self) -> None:
        import pybullet as p
        if p.isConnected():
            p.disconnect()

# =============================================================================
# Real Robot Backend (threading + RMPLAB_Uri)
# =============================================================================
def _real_send_action(command_queue: multiprocessing.Queue, status_queue: multiprocessing.Queue, robots: dict) -> None:
    """
    Runs in a separate thread. Reads commands from queue and executes on real robots.
    Exits when it receives {"action": "quit"}.
    """
    try:
        while True:
            try:
                cmd = command_queue.get_nowait()
                action = cmd["action"]
                robot_id = cmd.get("id")
                robot_name = "ayal" if robot_id == 1 else "uri"
                robot = robots[robot_name]
                                
                match action:
                    case "quit":
                        print("[real_robots] Quit signal received")
                        break
                    
                    case "movej":
                        values = cmd["values"]
                        robot.control.moveJ(values)
                        status_queue.put({"action": action, "status": "ok"})
                        print(f"[real_robots] {robot_name}: moveJ({values})")
                    
                    case "movel":
                        values = cmd["values"]
                        robot.control.moveL(values)
                        status_queue.put({"action": action, "status": "ok"})
                        print(f"[real_robots] {robot_name}: moveL({values})")
                    
                    case "get_tcp_pose":
                        pose = robot.recieve.getActualTCPPose()
                        status_queue.put({"action": action, "status": "ok", "tcp_pose": pose})
                        print(f"[real_robots] {robot_name}: TCP pose = {pose}")
                    
                    case "get_q_pose":
                        q = np.array(robot.recieve.getActualQ()) * 180 / np.pi
                        status_queue.put({"action": action, "status": "ok", "q_pose": q.tolist()})
                        print(f"[real_robots] {robot_name}: Joint angles = {q}")
                    
                    case "gripper_move":
                        pos = cmd["pos"]
                        speed = cmd["speed"]
                        force = cmd["force"]
                        robot.gripper.move_and_wait_for_pos(pos, speed, force)
                        status_queue.put({"action": action, "status": "ok"})
                        print(f"[real_robots] {robot_name}: gripper_move(pos={pos}, speed={speed}, force={force})")
                    
                    case "gripper_close":
                        speed = cmd["speed"]
                        force = cmd["force"]
                        robot.gripper.close(speed, force)
                        status_queue.put({"action": action, "status": "ok"})
                        print(f"[real_robots] {robot_name}: gripper_close(speed={speed}, force={force})")
                    
                    case "gripper_open":
                        speed = cmd["speed"]
                        force = cmd["force"]
                        robot.gripper.open(speed, force)
                        status_queue.put({"action": action, "status": "ok"})
                        print(f"[real_robots] {robot_name}: gripper_open(speed={speed}, force={force})")
                    
                    case "teachmode_toggle":
                        if robot.teachmode:
                            robot.control.endTeachMode()
                            robot.teachmode = False
                            status_queue.put({"action": action, "status": "ok", "teachmode": False})
                            print(f"[real_robots] {robot_name}: teachmode OFF")
                        else:
                            robot.control.teachMode()
                            robot.teachmode = True
                            status_queue.put({"action": action, "status": "ok", "teachmode": True})
                            print(f"[real_robots] {robot_name}: teachmode ON")
                    
                    case "prepare_puzzle" | "assemble_puzzle" | "dismantle_puzzle" | "finish_puzzle":
                        # Run complex actions in separate thread to avoid blocking queue
                        puzzle_thread = threading.Thread(
                            target=handler__puzzle_action,
                            args=(action, robots, cmd),
                            daemon=True
                        )
                        puzzle_thread.start()
                    
                    case _:
                        print(f"[real_robots] Unknown command: {action}")
                        status_queue.put({"action": action, "status": "error", "message": f"Unknown: {action}"})
            
            except Empty:
                time.sleep(0.01)
    
    except Exception as e:
        print(f"[real_robots] Error: {e}")
        import traceback
        traceback.print_exc()


def _real_init_robots() -> dict | None:
    """Initialize real robots. Returns dict or None on failure."""
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

    try:
        import uri_if as sim_uri_pkg
    except ImportError:
        print("Warning: sim_uri not available. Real robot backend disabled.")
        return None

    RMPLAB_Uri = sim_uri_pkg.RMPLAB_Uri
    HOST_AYAL = sim_uri_pkg.HOST_AYAL
    HOST_URI = sim_uri_pkg.HOST_URI
    
    print(f"[backends] Attempting to connect to real robots...")
    
    ayal = RMPLAB_Uri(host=HOST_AYAL, backend="real")
    uri = RMPLAB_Uri(host=HOST_URI, backend="real")
    
    try:
        ayal.connect(calibrate=False)
        uri.connect(calibrate=False)
        
        print(f"[backends] Connected to real robots: AYAL={HOST_AYAL}, URI={HOST_URI}")
        return {"ayal": ayal, "uri": uri}
    except Exception as e:
        print(f"[backends] Error connecting to real robots: {e}")
        import traceback
        traceback.print_exc()
        return None


class RealBackend(RobotBackend):
    """Real robot backend — commands executed via threading."""

    def __init__(self):
        print("[backends] Starting real robot backend...")
        self.robots = _real_init_robots()
        if not self.robots:
            raise RuntimeError("Failed to connect to real robots")
        self.cmd_queue = multiprocessing.Queue()
        self.status_queue = multiprocessing.Queue()
        self.thread = threading.Thread(
            target=_real_send_action,
            args=(self.cmd_queue, self.status_queue, self.robots),
            daemon=True,
        )
        self.thread.start()

    def send_command(self, cmd: dict) -> None:
        self.cmd_queue.put(cmd)

    def poll_status(self) -> dict | None:
        try:
            return self.status_queue.get_nowait()
        except Empty:
            return None

    def shutdown(self) -> None:
        self.cmd_queue.put({"action": "quit"})
        for robot_name, robot in self.robots.items():
            try:
                robot.disconnect()
                print(f"[backends] Disconnected {robot_name}")
            except Exception as e:
                print(f"[backends] Error disconnecting {robot_name}: {e}")
