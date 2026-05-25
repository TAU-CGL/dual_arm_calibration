
import os, sys

_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _RMP_LAB_ROOT not in sys.path:
    sys.path.insert(0, _RMP_LAB_ROOT)

import uri_if
import numpy as np
import time
import threading

# 1. setup
def jitter_setup(host: str):
    uri = uri_if.RMPLAB_Uri(host)
    uri.connect(False)
    base_pose = list(uri.recieve.getActualTCPPose())
    return uri, base_pose

def compliant_setup(host: str):
    uri = uri_if.RMPLAB_Uri(host)
    uri.connect(False)
    uri.control.teachMode()
    return uri

# 2. jitter
def jitter(uri: uri_if.RMPLAB_Uri, base_pose: list[float], delay: float,
           stop_event: threading.Event | None = None):
    min_value_translation = [x - 0.01 for x in base_pose[:3]]
    max_value_translation = [x + 0.01 for x in base_pose[:3]]
    min_value_rotation = [x - 0.05 for x in base_pose[3:6]]
    max_value_rotation = [x + 0.05 for x in base_pose[3:6]]
    def stop() -> bool:
        return stop_event is not None and stop_event.is_set()
    while not stop():
        for _ in range(10):
            if stop():
                return
            jittered_pose = base_pose.copy()
            random_noise = (np.random.rand(6) - 0.5) * 0.0005 # ±0.5 mm noise
            new_pose = [jittered_pose[i] + random_noise[i] for i in range(6)]

            jittered_pose[:3] = [
                max(min(new_pose[k], max_value_translation[k]), min_value_translation[k])
                for k in range(3)
            ]
            jittered_pose[3:6] = [
                max(min(new_pose[k], max_value_rotation[k-3]), min_value_rotation[k-3])
                for k in range(3, 6)
            ]
            uri.control.moveJ_IK(jittered_pose, 1, 1, False)
            time.sleep(delay)
            uri.control.moveJ_IK(base_pose, 1, 1, False)
        time.sleep(0.5)

# 3. finish
def finish(uri: uri_if.RMPLAB_Uri, ayal: uri_if.RMPLAB_Uri):
    ayal.control.endTeachMode()
    ayal.disconnect()
    uri.disconnect()

def main():
    URI_HOST        = "192.168.56.101"
    AYAL_HOST       = "192.168.57.101"
    delay          = 0.01 # seconds between jittered moveL commands

    ayal = compliant_setup(AYAL_HOST)
    time.sleep(1) # ensure ayal is in teach mode before starting jitter
    input("Press Enter to start jittering...")
    uri, base_pose = jitter_setup(URI_HOST)
    jitter(uri, base_pose, delay)
    finish(uri, ayal)

if __name__ == "__main__":
    main()