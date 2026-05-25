# region imports
import os
from re import match
import sys
import time
import pybullet as p
import pybullet_data
from tqdm import tqdm


_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_ROBOTIQ_ROOT = os.path.join(_RMP_LAB_ROOT, "uri_sim", "pybullet_ur5_robotiq-robotflow")
if _RMP_LAB_ROOT in sys.path:
    sys.path.remove(_RMP_LAB_ROOT)
sys.path.insert(0, _RMP_LAB_ROOT)
if _ROBOTIQ_ROOT in sys.path:
    sys.path.remove(_ROBOTIQ_ROOT)
sys.path.insert(0, _ROBOTIQ_ROOT)
import uri_if
from uri_gui_pyqt5.src.utils_1 import load_sim_config
from robot import UR5eRobotiq85
# Must match alpha_puzzle/dismantle_puzzle.py (used by sim_uri.register_robot)
#endregion

URI_HOST = "192.168.56.101"
AYAL_HOST = "192.168.57.101"

class _SimEnv:
    """Minimal env for rmplab_uri: exposes step_simulation() -> PyBullet step."""

    def step_simulation(self):
        p.stepSimulation()

_config = load_sim_config()
_sim_config = _config["sim"]

"""
The PyBullet sim
"""
class multiRobotsSim:
    def __init__(self, config):
        self.config = config
        self.mode = self.config["mode"]
        p.connect(p.DIRECT if self.mode == "headless" else p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) # toggle back in the sim using 'g'
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        self.robots = self.initRobots()
        self._sim_env = _SimEnv()
        uri_if.register_robot(URI_HOST, self.robots[0], self._sim_env)
        uri_if.register_robot(AYAL_HOST, self.robots[1], self._sim_env)

        self.camera = self.initCamera()
        self.planes = self.initPlanes()
        self.stands = self.initStands()
        self._reach_grid_ids = []

        if self.config["mode"] == "script":
            self.p_bar = tqdm(ncols=0, disable=False)  # TODO understand

    def step(self):
        p.stepSimulation()
        if not self.mode == "headless":
            time.sleep(_sim_config["sim_render_res"])
    
    def is_connected(self):
        return p.isConnected()
    
    def initRobots(self):
        uri = self.initRobot(_sim_config["base_uri"][:3], _sim_config["base_uri"][3:], tuple(_sim_config["default_joints_uri"]))
        ayal = self.initRobot(_sim_config["base_ayal"][:3], _sim_config["base_ayal"][3:], tuple(_sim_config["default_joints_ayal"]))
        return [uri, ayal]

    def initRobot(self, pos, ori, joints):
        robot = UR5eRobotiq85(pos=pos, ori=ori)
        robot.step_simulation = lambda: p.stepSimulation()
        robot.load()
        robot.move_ee(joints,control_method='joint')
        robot.move_gripper(0.04)
        return robot

    def initCamera(self):
        pass
    
    def initPlanes(self):
        plane = p.loadURDF("plane.urdf")
        return [plane]

    def initStands(self):
        pass

    def mapCommand(self, command):
        if "action" not in command:
            print("ERROR, command must have action")
            print(f"This command: {command}")
            return {"status": "error", "message": "no action"}

        action = command.get("action")
        result = {"action": action, "status": "error", "message": None}

        match action:
            case "movej":
                if "id" not in command:
                    msg = "ERROR, movej requires id"
                    print(msg)
                    return {"action": action, "status": "error", "message": msg}
                robot_id = command.get("id")
                robot = self.robots[robot_id]
                joints = command.get("values")
                robot.move_ee(tuple(joints), control_method="joint")
                result.update({"status": "ok"})

            case "movel":
                if "id" not in command:
                    msg = "ERROR, movel requires id"
                    print(msg)
                    return {"action": action, "status": "error", "message": msg}
                robot_id = command.get("id")
                robot = self.robots[robot_id]
                pose = command.get("values")
                robot.move_ee(tuple(pose), control_method="end")
                result.update({"status": "ok"})

            case "get_tcp_pose":
                if "id" not in command:
                    msg = "ERROR, get_tcp_pose requires id"
                    print(msg)
                    return {"action": action, "status": "error", "message": msg}
                robot_id = command.get("id")
                robot = self.robots[robot_id]
                info = robot.get_joint_obs()
                result.update({"status": "ok", "tcp_pose": info.get("ee_pos")})
                print(f"[sim] robot[{robot_id}] TCP pose = {info.get('ee_pos')}")

            case "get_q_pose":
                if "id" not in command:
                    msg = "ERROR, get_q_pose requires id"
                    print(msg)
                    return {"action": action, "status": "error", "message": msg}
                robot_id = command.get("id")
                robot = self.robots[robot_id]
                info = robot.get_joint_obs()
                result.update({"status": "ok", "q_pose": info.get("positions")})
                print(f"[sim] robot[{robot_id}] joint angles = {info.get('positions')}")

            case "gripper_move":
                if "id" not in command:
                    msg = "ERROR, gripper_move requires id"
                    print(msg)
                    return {"action": action, "status": "error", "message": msg}
                robot_id = command.get("id")
                robot = self.robots[robot_id]
                pos = command.get("pos")
                if pos is None:
                    msg = "ERROR, gripper_move requires pos"
                    print(msg)
                    return {"action": action, "status": "error", "message": msg}
                robot.move_gripper(pos)
                result.update({"status": "ok", "gripper_pos": pos})
                print(f"[sim] robot[{robot_id}] gripper_move(pos={pos})")

            case "gripper_close":
                if "id" not in command:
                    msg = "ERROR, gripper_close requires id"
                    print(msg)
                    return {"action": action, "status": "error", "message": msg}
                robot_id = command.get("id")
                robot = self.robots[robot_id]
                robot.close_gripper()
                result.update({"status": "ok"})
                print(f"[sim] robot[{robot_id}] gripper_close")

            case "gripper_open":
                if "id" not in command:
                    msg = "ERROR, gripper_open requires id"
                    print(msg)
                    return {"action": action, "status": "error", "message": msg}
                robot_id = command.get("id")
                robot = self.robots[robot_id]
                robot.open_gripper()
                result.update({"status": "ok"})
                print(f"[sim] robot[{robot_id}] gripper_open")

            case "teachmode_toggle":
                result.update({"status": "unsupported", "message": "teachmode not available in sim"})
                print("[sim] teachmode_toggle not supported in simulation")

            case "prepare_puzzle":
                res = self._run_prepare_puzzle()
                result.update(res)

            case "assemble_puzzle":
                res = self._run_assemble_puzzle()
                result.update(res)

            case "dismantle_puzzle":
                res = self._run_dismantle_puzzle()
                result.update(res)

            case "separate_puzzle" | "finish_puzzle":
                res = self._run_separate_puzzle()
                result.update(res)

            case "show_reach_grid":
                self._clear_reach_grid()
                res = self._show_reach_grid(command.get("npz_path"),
                                            point_size=command.get("point_size", 4.0),
                                            show_unreachable=command.get("show_unreachable", False))
                result.update(res)

            case "clear_reach_grid":
                self._clear_reach_grid()
                result.update({"status": "ok"})

            case _:
                msg = f"Unknown action: {action!r}"
                print(msg)
                result.update({"status": "error", "message": msg})

        return result

    def _show_reach_grid(self, npz_path, point_size=4.0, show_unreachable=False):
        """Overlay reachability grid points (in Uri's base frame) into the sim.

        +1 (clean)        -> green
         0 (singularity)  -> yellow
        -1 (unreachable)  -> red, drawn only if show_unreachable=True
        """
        import numpy as np
        if not npz_path or not os.path.isfile(npz_path):
            return {"status": "error", "message": f"npz not found: {npz_path}"}

        data = np.load(npz_path)
        grid = data["grid"]
        xs, ys, zs = data["xs"], data["ys"], data["zs"]

        base_uri = list(_sim_config["base_uri"][:3])  # Uri base in world frame

        positions, colors = [], []
        for ix, x in enumerate(xs):
            for iy, y in enumerate(ys):
                for iz, z in enumerate(zs):
                    v = int(grid[ix, iy, iz])
                    if v == 1:
                        c = (0.0, 1.0, 0.0)
                    elif v == 0:
                        c = (1.0, 1.0, 0.0)
                    else:
                        if not show_unreachable:
                            continue
                        c = (1.0, 0.0, 0.0)
                    positions.append([base_uri[0] + float(x),
                                      base_uri[1] + float(y),
                                      base_uri[2] + float(z)])
                    colors.append(c)

        # PyBullet's addUserDebugPoints chokes on huge batches; chunk it.
        CHUNK = 500
        for i in range(0, len(positions), CHUNK):
            item_id = p.addUserDebugPoints(positions[i:i + CHUNK],
                                           colors[i:i + CHUNK],
                                           pointSize=point_size)
            self._reach_grid_ids.append(item_id)
        print(f"[sim] reach grid: drew {len(positions)} points "
              f"(+1: {int((grid == 1).sum())}, 0: {int((grid == 0).sum())}, "
              f"-1: {int((grid == -1).sum())}, unreachable {'shown' if show_unreachable else 'hidden'})")
        return {"status": "ok", "points": len(positions)}

    def _clear_reach_grid(self):
        for item_id in self._reach_grid_ids:
            try:
                p.removeUserDebugItem(item_id)
            except Exception:
                pass
        self._reach_grid_ids = []

    def _get_puzzle_paths(self):
        alpha_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "alpha_puzzle"))
        if alpha_dir in sys.path:
            sys.path.remove(alpha_dir)
        sys.path.insert(0, alpha_dir)

        dyn_path = os.path.join(alpha_dir, "paths", "alphaZ", "improved_dynamic_ik_trajectory.csv")
        static_path = os.path.join(alpha_dir, "paths", "alphaZ", "improved_static_ik_trajectory.csv")

        if not os.path.isfile(dyn_path) or not os.path.isfile(static_path):
            print(f"[puzzle] Missing trajectory CSV under {alpha_dir!r} (expected improved_*_ik_trajectory.csv).")
            return None, None, None

        return alpha_dir, dyn_path, static_path

    def _load_puzzle_trajectories(self):
        paths = self._get_puzzle_paths()
        if paths[0] is None:
            return None, None

        _, dyn_path, static_path = paths
        from dismantle_puzzle import csv_to_arm_path

        return csv_to_arm_path(dyn_path), csv_to_arm_path(static_path)

    def _open_puzzle_hosts(self):
        uri = uri_if.RMPLAB_Uri(URI_HOST)
        ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
        uri.connect(False)
        ayal.connect(False)

        if not uri.is_connected() or not ayal.is_connected():
            print("[puzzle] rmplab_uri sim not connected (register_robot / hosts).")
            return None, None

        return uri, ayal

    def _close_puzzle_hosts(self, uri, ayal):
        try:
            if uri is not None:
                uri.disconnect()
            if ayal is not None:
                ayal.disconnect()
        except Exception as e:
            print(f"[puzzle] Error closing hosts: {e}")

    def _run_prepare_puzzle(self):
        from dismantle_puzzle import prepare_robots

        uri, ayal = self._open_puzzle_hosts()
        if uri is None or ayal is None:
            return {"status": "error", "message": "host connect failed"}

        try:
            prepare_robots(uri, ayal)
            return {"status": "ok"}
        except Exception as e:
            msg = f"[prepare_puzzle] {type(e).__name__}: {e}"
            print(msg)
            return {"status": "error", "message": str(e)}
        finally:
            self._close_puzzle_hosts(uri, ayal)

    def _run_assemble_puzzle(self):
        from dismantle_puzzle import reversed_trajectory

        paths = self._load_puzzle_trajectories()
        if paths[0] is None:
            return {"status": "error", "message": "trajectory files missing"}
        dynamic_ik_trajectory, static_ik_trajectory = paths

        uri, ayal = self._open_puzzle_hosts()
        if uri is None or ayal is None:
            return {"status": "error", "message": "host connect failed"}

        try:
            reversed_trajectory(uri, ayal, dynamic_ik_trajectory, static_ik_trajectory)
            return {"status": "ok"}
        except Exception as e:
            msg = f"[assemble_puzzle] {type(e).__name__}: {e}"
            print(msg)
            return {"status": "error", "message": str(e)}
        finally:
            self._close_puzzle_hosts(uri, ayal)

    def _run_dismantle_puzzle(self):
        from dismantle_puzzle import forward_trajectory

        paths = self._load_puzzle_trajectories()
        if paths[0] is None:
            return {"status": "error", "message": "trajectory files missing"}
        dynamic_ik_trajectory, static_ik_trajectory = paths

        uri, ayal = self._open_puzzle_hosts()
        if uri is None or ayal is None:
            return {"status": "error", "message": "host connect failed"}

        try:
            forward_trajectory(uri, ayal, dynamic_ik_trajectory, static_ik_trajectory)
            return {"status": "ok"}
        except Exception as e:
            msg = f"[dismantle_puzzle] {type(e).__name__}: {e}"
            print(msg)
            return {"status": "error", "message": str(e)}
        finally:
            self._close_puzzle_hosts(uri, ayal)

    def _run_separate_puzzle(self):
        from dismantle_puzzle import finish_robots

        uri, ayal = self._open_puzzle_hosts()
        if uri is None or ayal is None:
            return {"status": "error", "message": "host connect failed"}

        try:
            finish_robots(uri, ayal)
            return {"status": "ok"}
        except Exception as e:
            msg = f"[separate_puzzle] {type(e).__name__}: {e}"
            print(msg)
            return {"status": "error", "message": str(e)}
        finally:
            self._close_puzzle_hosts(uri, ayal)

