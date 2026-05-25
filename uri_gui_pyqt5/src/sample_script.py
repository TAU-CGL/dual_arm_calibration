"""
Sample script: drive the simulation without a GUI.
Usage: python uris_gui_pyqt5/main.py script uris_gui_pyqt5/sample_script.py
"""
import time


def run(backend):
    time.sleep(1.0)

    backend.send_command({
        "action": "movej",
        "id": 0,
        "values": (0.5, -1.0, 1.5, -1.0, -1.57, 0.0),
    })
    time.sleep(2.0)

    backend.send_command({"action": "get_tcp_pose", "id": 0})
    time.sleep(0.5)

    backend.send_command({
        "action": "movej",
        "id": 1,
        "values": (-0.5, -1.2, 1.0, -0.8, 1.57, 0.0),
    })
    time.sleep(2.0)

    backend.send_command({"action": "gripper_open", "id": 0})
    time.sleep(1.0)

    while True:
        status = backend.poll_status()
        if status is None:
            break
        print(f"[script] response: {status}")

    print("[script] Waiting 3s before shutdown...")
    time.sleep(3.0)
