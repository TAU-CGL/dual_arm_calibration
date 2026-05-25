#!/usr/bin/env python3
import os
import sys
import time
from pathlib import Path

_RMP_LAB_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _RMP_LAB_ROOT in sys.path:
    sys.path.remove(_RMP_LAB_ROOT)
sys.path.insert(0, _RMP_LAB_ROOT)
import uri_if

URI_HOST = "192.168.56.101"
AYAL_HOST = "192.168.57.101"

STEP_TRANS = 0.0005
STEP_ROT = 0.01
SPEED = 0.01
ACCEL = 0.1
SETTLE_S = 1
N_SAMPLES = 50
SAMPLE_DELAY_S = 0.002
LOG_FILE = Path(__file__).parent / "probe_axes.log"
USER_CONFIRM = "--user" in sys.argv


def wait(label):
    if USER_CONFIRM:
        input(f"{label}? ")

AXES = [
    # ("+x", 0, +1), ("-x", 0, -1),
    # ("+y", 1, +1), ("-y", 1, -1),
    # ("+z", 2, +1), ("-z", 2, -1),
    # ("+Rx", 3, +1), ("-Rx", 3, -1),
    ("+Ry", 4, +1), ("-Ry", 4, -1),
    ("+Rz", 5, +1), ("-Rz", 5, -1),
]


def fmt(v):
    return "[" + ", ".join(f"{x:.4f}" for x in v) + "]"


def fmt_deriv(w_after, w_init, step, idx):
    result = []
    for j in range(6):
        dw = w_after[j] - w_init[j]
        if idx < 3:
            denom = step if j < 3 else step * 10
        else:
            denom = step / 10 if j < 3 else step
        result.append(dw / denom)
    return fmt(result)


def avg_wrench(robot):
    samples = []
    for i in range(N_SAMPLES):
        if i:
            time.sleep(SAMPLE_DELAY_S)
        samples.append(list(robot.recieve.getActualTCPForce())[:6])
    return [sum(s[j] for s in samples) / len(samples) for j in range(6)]


def main():
    uri = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    uri.connect(False)
    ayal.connect(False)

    log = open(LOG_FILE, "w")

    def emit(line):
        print(line)
        log.write(line + "\n")
        log.flush()

    try:
        for i in range(5):
            step_rot = STEP_ROT * (i + 1)
            emit(f"STEP_TRANS:   {STEP_TRANS}")
            emit(f"STEP_ROT:     {step_rot}")
            start_uri = list(uri.recieve.getActualTCPPose())
            emit(f"init tcp_uri:     {fmt(start_uri)}")
            emit(f"init tcp_ayal:    {fmt(ayal.recieve.getActualTCPPose())}")

            ayal.control.zeroFtSensor()
            time.sleep(SETTLE_S)

            init_wrench = avg_wrench(ayal)
            emit(f"init wrench_ayal: {fmt(init_wrench)}")
            
            for label, idx, sign in AXES:
                step = STEP_TRANS if idx < 3 else step_rot
                target = start_uri[:]
                target[idx] += sign * step
                wait(label)
                uri.control.moveL(target, SPEED, ACCEL, False)
                time.sleep(SETTLE_S)
                # emit(f"{label} tcp_uri:     {fmt(uri.recieve.getActualTCPPose())}")
                # emit(f"{label} tcp_ayal:    {fmt(ayal.recieve.getActualTCPPose())}")
                w_after = avg_wrench(ayal)
                emit(f"{label} dwrench_ayal: {fmt_deriv(w_after, init_wrench, step, idx)}")
                wait(f"{label} back")
                uri.control.moveL(start_uri, SPEED, ACCEL, False)
                time.sleep(SETTLE_S)
        
    finally:
        log.close()
        uri.disconnect()
        ayal.disconnect()


if __name__ == "__main__":
    main()
