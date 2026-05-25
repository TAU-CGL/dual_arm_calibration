"""
FSM that wraps a single robot's movement and gates every move on a safety check
of BOTH the robot and its peer (via DashboardHandler).

States:
    DISCONNECTED -> IDLE -> MOVING -> IDLE
    any active state can transition to a fault state on a failed safety check:
        PROTECTIVE_STOP, EMERGENCY_STOP, FAULTED, PEER_FAULT

Usage:
    self_dash = DashboardHandler(URI_HOST)
    peer_dash = DashboardHandler(AYAL_HOST)
    fsm = RobotFSM(uri, self_dash, peer_dash, name="uri")
    fsm.connect()
    fsm.move(lambda: uri.control.moveL(target, speed, accel))

On a failed pre-move check the FSM either raises FaultError (default) or
prompts the user for a manual decision, depending on `on_fault`.
"""
from __future__ import annotations

import math
import os
import sys
import time
import numpy as np
from enum import Enum
from typing import Callable, Optional
from .dashboard_handler import DashboardHandler

class RobotState(Enum):
    DISCONNECTED = "disconnected"
    IDLE = "idle"
    MOVING = "moving"
    PROTECTIVE_STOP = "protective_stop"
    EMERGENCY_STOP = "emergency_stop"
    PEER_FAULT = "peer_fault"
    FAULTED = "faulted"

FAULT_STATES = {
    RobotState.PROTECTIVE_STOP,
    RobotState.EMERGENCY_STOP,
    RobotState.PEER_FAULT,
    RobotState.FAULTED,
}

class OnFault(Enum):
    RAISE = "raise"      # default: raise FaultError
    PROMPT = "prompt"    # ask the user via input() what to do

class FaultError(RuntimeError):
    def __init__(self, who: str, state: RobotState, detail: str = ""):
        super().__init__(f"{who} fault: {state.value} {detail}".strip())
        self.who = who
        self.state = state
        self.detail = detail

def _classify(dash: DashboardHandler) -> RobotState:
    """Map a dashboard's current safety status to a RobotState."""
    if not dash.is_robot_connected():
        return RobotState.DISCONNECTED
    if dash.is_robot_emergency_stopped():
        return RobotState.EMERGENCY_STOP
    if dash.is_robot_protective_stopped():
        return RobotState.PROTECTIVE_STOP
    if dash.is_robot_in_error():
        return RobotState.FAULTED
    if not dash.is_powered_on():
        return RobotState.DISCONNECTED
    return RobotState.IDLE

class RobotFSM:
    def __init__(
        self,
        robot,
        self_dash: DashboardHandler,
        peer_dash: DashboardHandler,
        name: str = "robot",
        peer_name: str = "peer",
        on_fault: OnFault = OnFault.RAISE,
        auto_clamp_recover: bool = False,
        wrist3_jog_deg: float = 10.0,
        max_clamp_retries: int = 1,
        post_move_settle_s: float = 2.0,
    ):
        self.robot = robot
        self.self_dash = self_dash
        self.peer_dash = peer_dash
        self.name = name
        self.peer_name = peer_name
        self.on_fault = on_fault
        self.auto_clamp_recover = auto_clamp_recover
        self.wrist3_jog_deg = wrist3_jog_deg
        self.max_clamp_retries = max_clamp_retries
        self.post_move_settle_s = post_move_settle_s
        self._state = RobotState.DISCONNECTED

    @property
    def state(self) -> RobotState:
        return self._state

    def connect(self) -> None:
        self._state = _classify(self.self_dash)

    def safety_check(self) -> None:
        """Inspect self + peer; raise / prompt if either is faulted."""
        self_state = _classify(self.self_dash)
        peer_state = _classify(self.peer_dash)

        if self_state in FAULT_STATES:
            self._state = self_state
            self._handle_fault(self.name, self_state)
            return

        if peer_state in FAULT_STATES:
            self._state = RobotState.PEER_FAULT
            self._handle_fault(self.peer_name, peer_state)
            return

        if self_state == RobotState.DISCONNECTED:
            self._state = RobotState.DISCONNECTED
            self._handle_fault(self.name, self_state, detail="not powered/connected")
            return

        # both ok
        if self._state in FAULT_STATES or self._state == RobotState.DISCONNECTED:
            self._state = RobotState.IDLE

    def move(self, action: Callable[[], None]) -> None:
        """Run a pre-check, execute `action`, run a post-check.

        `action` should be a no-arg callable that issues the actual move
        (e.g. lambda: robot.control.moveL(target, speed, accel)).

        If `auto_clamp_recover` is enabled and the move trips a protective
        stop, this method probes wrist3 by ±`wrist3_jog_deg` from the
        post-stop pose: if exactly one side stays clear, it commits to that
        un-clamping side and retries the original action; otherwise it
        re-raises.
        """
        for attempt in range(self.max_clamp_retries + 1):
            try:
                self._move_once(action)
                return
            except FaultError as e:
                time.sleep(1)
                if (
                    self.auto_clamp_recover
                    and e.state == RobotState.PROTECTIVE_STOP
                    and attempt < self.max_clamp_retries
                    and self._diagnose_and_recover_clamp()
                ):
                    print(f"[{self.name} fsm] clamp recovery succeeded; retrying original move")
                    continue
                raise

    def _move_once(self, action: Callable[[], None]) -> None:
        self.safety_check()
        if self._state in FAULT_STATES:
            return  # _handle_fault already decided whether to raise

        self._state = RobotState.MOVING
        action_exc: Optional[BaseException] = None
        try:
            action()
        except BaseException as e:
            action_exc = e
        # The dashboard's safetymode() can lag the controller, so a protective
        # stop triggered during/just after the move may not show up on a
        # single post-check. Poll briefly so we don't silently mark it IDLE.
        self._post_move_safety_wait(timeout=self.post_move_settle_s)
        if self._state == RobotState.MOVING:
            self._state = RobotState.IDLE
        if action_exc is not None:
            raise action_exc

    def _post_move_safety_wait(self, timeout: float) -> None:
        """Poll dashboard up to `timeout`s; first fault observed raises via safety_check."""
        deadline = time.time() + max(0.0, timeout)
        while True:
            self.safety_check()  # raises FaultError if a fault is now visible
            if time.time() >= deadline:
                return
            time.sleep(0.1)

    # ---- fault handling -----------------------------------------------------
    def _handle_fault(self, who: str, state: RobotState, detail: str = "") -> None:
        if self.on_fault == OnFault.RAISE:
            raise FaultError(who, state, detail)
        # PROMPT
        msg = f"[{self.name} fsm] {who} is in {state.value} {detail}".strip()
        print(msg)
        choice = input("  [r]etry / [a]bort / [u]nlock-protective-stop: ").strip().lower()
        if choice == "r":
            return
        if choice == "u":
            target = self.self_dash if who == self.name else self.peer_dash
            target.unlock_protective_stop()
            return
        raise FaultError(who, state, detail)

    # ---- clamp auto-recovery -----------------------------------------------
    def _diagnose_and_recover_clamp(self) -> bool:
        """Probe wrist3 ±wrist3_jog_deg from the post-stop pose.

        Procedure:
          1. Unlock + close popup.
          2. Capture q_anchor (the joint config the controller stopped at).
          3. moveJ to q_anchor with q[5] += +delta. After settle, is the
             robot in protective stop? -> plus_ok.
          4. If plus_ok: moveJ back to q_anchor (so we test - from the same
             baseline). If THAT fails, abort.
          5. moveJ to q_anchor with q[5] += -delta. -> minus_ok.

        Decision:
          - plus_ok and not minus_ok -> '-' is the clamping side; commit to '+'.
          - minus_ok and not plus_ok -> '+' is the clamping side; we're already at '-'.
          - both ok                   -> not a one-sided clamp; return to anchor and report False.
          - both faulted              -> can't diagnose as a clamp; return False.

        Returns True iff we resolved a one-sided clamp and the robot now
        sits at the un-clamping wrist3 angle, ready for the original move
        to be retried.
        """
        if not self._unlock_and_settle():
            return False

        q_anchor = list(self.robot.recieve.getActualQ())
        delta = math.radians(self.wrist3_jog_deg)
        print(f"[{self.name} fsm] clamp diagnosis: probing wrist3 ±{self.wrist3_jog_deg:.1f}° from q[5]={q_anchor[5]:.4f}")

        plus_ok = self._probe_wrist3(q_anchor, +delta)
        print(f"[{self.name} fsm]   + side: {'OK' if plus_ok else 'FAULT'}")

        # if plus_ok:
        #     ret_ok = self._probe_wrist3(q_anchor, 0.0)
        #     if not ret_ok:
        #         print(f"[{self.name} fsm]   anchor return faulted; aborting diagnosis")
        #         return False

        minus_ok = self._probe_wrist3(q_anchor, -delta*2)
        print(f"[{self.name} fsm]   - side: {'OK' if minus_ok else 'FAULT'}")

        if plus_ok and not minus_ok:
            print(f"[{self.name} fsm] clamp on '-' side; committing to '+'")
            return self._probe_wrist3(q_anchor, +delta)
        if minus_ok and not plus_ok:
            print(f"[{self.name} fsm] clamp on '+' side; staying at '-'")
            return True
        if plus_ok and minus_ok:
            print(f"[{self.name} fsm] both sides clear — not a one-sided clamp")
            self._probe_wrist3(q_anchor, 0.0)
            return False
        print(f"[{self.name} fsm] both sides faulted — cannot recover")
        return False

    def _unlock_and_settle(self) -> bool:
        try:
            self.self_dash.unlock_protective_stop()
        except Exception as e:
            print(f"[{self.name} fsm] unlock failed: {e}")
            return False
        try:
            self.self_dash.close_safety_popup()
        except Exception:
            pass
        ready = self.self_dash.wait_until_ready(timeout=10)
        time.sleep(0.5)
        # After unlock, the URRTDE control script is no longer running on the
        # controller, so subsequent control.moveJ()/moveL() calls return
        # immediately without moving the robot. Reupload puts it back.
        try:
            self.robot.control.reuploadScript()
            time.sleep(0.5)
        except Exception as e:
            print(f"[{self.name} fsm] reuploadScript failed: {e}")
            return False
        return bool(ready)

    def _probe_wrist3(self, q_anchor, delta_rad: float) -> bool:
        """Move wrist3 to q_anchor[5] + delta_rad. Return True iff no protective
        stop after the move. On fault, unlock so the next probe can run."""
        target = list(q_anchor)
        target[4] += delta_rad
        try:
            self.robot.control.moveL_FK(target, 0.5, 0.5)
            time.sleep(1)
        except Exception as e:
            print(f"[{self.name} fsm]   moveJ raised: {e}")
        time.sleep(self.post_move_settle_s)
        if self.self_dash.is_robot_protective_stopped():
            self._unlock_and_settle()
            return False
        return True

# ---- demo / smoke test ------------------------------------------------------
URI_HOST = "192.168.56.101"
AYAL_HOST = "192.168.57.101"

def _kick_protective_stop(host: str) -> None:
    """Send `protective_stop()` over the URScript secondary port (30002).

    Out-of-band w.r.t. RTDE — works while a moveL is running on the same
    RTDEControlInterface.
    """
    import socket
    with socket.create_connection((host, 30002), timeout=2.0) as s:
        s.sendall(b"protective_stop()\n")

def _demo_protective_stop_mid_move():
    """Start a slow uri move, inject a protective stop ~1s in, observe FSM."""
    import threading
    import time

    import uri_if  # noqa: E402

    uri = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    uri.connect(False)
    ayal.connect(False)

    uri_dash = DashboardHandler(URI_HOST)
    ayal_dash = DashboardHandler(AYAL_HOST)

    fsm = RobotFSM(uri, uri_dash, ayal_dash, name="uri", peer_name="ayal")
    fsm.connect()
    print(f"[demo] initial fsm state: {fsm.state.value}")

    pose = list(uri.recieve.getActualTCPPose())
    target = list(pose)
    target[2] -= 0.05  # 5cm down in base frame, slow speed -> ~5s of motion

    move_error = {}

    def do_move():
        try:
            fsm.move(lambda: uri.control.moveL(target, 0.01, 0.1))
        except FaultError as e:
            move_error["err"] = e

    t = threading.Thread(target=do_move)
    t.start()

    time.sleep(2.0)
    print(f"[demo] mid-move fsm state: {fsm.state.value} — triggering protective stop")
    _kick_protective_stop(URI_HOST)

    t.join(timeout=10.0)
    print(f"[demo] post-move fsm state: {fsm.state.value}")
    if "err" in move_error:
        print(f"[demo] fsm raised: {move_error['err']}")

    # leave the cell in a clean state
    print("[demo] unlocking protective stop on uri")
    time.sleep(2.0)
    uri_dash.unlock_protective_stop()

    uri.disconnect()
    ayal.disconnect()
    uri_dash.disconnect()
    ayal_dash.disconnect()

def _demo_peer_fault():
    """Move uri while ayal is in protective stop -> uri fsm raises PEER_FAULT."""
    import time
    import uri_if  # noqa: E402

    uri = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    uri.connect(False)
    ayal.connect(False)

    uri_dash = DashboardHandler(URI_HOST)
    ayal_dash = DashboardHandler(AYAL_HOST)

    fsm = RobotFSM(uri, uri_dash, ayal_dash, name="uri", peer_name="ayal")
    fsm.connect()
    print(f"[demo] initial uri fsm state: {fsm.state.value}")

    print("[demo] putting ayal into protective stop")
    _kick_protective_stop(AYAL_HOST)
    time.sleep(1.5)  # let dashboard reflect the new safety state

    pose = list(uri.recieve.getActualTCPPose())
    target = list(pose)
    target[2] -= 0.01

    try:
        fsm.move(lambda: uri.control.moveL(target, 0.01, 0.1))
        print("[demo] move returned without raising — unexpected")
    except FaultError as e:
        print(f"[demo] uri fsm refused to move: {e}")

    print(f"[demo] uri fsm state after refusal: {fsm.state.value}")

    print("[demo] unlocking ayal protective stop")
    ayal_dash.unlock_protective_stop()

    uri.disconnect()
    ayal.disconnect()
    uri_dash.disconnect()
    ayal_dash.disconnect()

def _demo_prompt_recovery():
    """Interactive: trigger uri protective stop, see PROMPT, pick 'u' to unlock+retry."""
    import time
    import uri_if  # noqa: E402

    uri = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    uri.connect(False)
    ayal.connect(False)

    uri_dash = DashboardHandler(URI_HOST)
    ayal_dash = DashboardHandler(AYAL_HOST)

    fsm = RobotFSM(uri, uri_dash, ayal_dash, name="uri", peer_name="ayal", on_fault=OnFault.PROMPT)
    fsm.connect()

    print("[demo] putting uri into protective stop")
    _kick_protective_stop(URI_HOST)
    time.sleep(1.5)

    pose = list(uri.recieve.getActualTCPPose())
    target = list(pose)
    target[2] -= 0.01

    print("[demo] requesting move — expect prompt; pick 'u' to unlock")
    try:
        fsm.move(lambda: uri.control.moveL(target, 0.01, 0.1))
        print(f"[demo] move completed; final state: {fsm.state.value}")
    except FaultError as e:
        print(f"[demo] aborted: {e}")

    uri.disconnect()
    ayal.disconnect()
    uri_dash.disconnect()
    ayal_dash.disconnect()

def _demo_clamp_auto_recover():
    """Run a moveJ that you expect to trigger a clamping protective stop on AYAL,
    then watch the FSM auto-unlock + jog wrist3 + retry.

    To use:
      1. Fill in TARGET_Q with joint angles (radians) that you've found cause
         a 'danger of clamping' (C403A0) protective stop on AYAL.
      2. Make sure both robots are powered, in remote control, not faulted.
      3. Run:  python dual_arm_peg/src/robot_fsm.py clamp
    """
    import uri_if  # noqa: E402

    # ===== FILL IN: target joint config that triggers a C403A0 clamp stop =====
    # 6 joints in radians. Tip: paste from ayal.recieve.getActualQ() after you
    # manually drive ayal into a near-clamping configuration on the teach pad.
    TARGET_Q = [-72.671645, -47.819777, 132.954321, 52.790473, 250, 164.670083]
    TARGET_Q = [float(x) / 180 * np.pi for x in TARGET_Q]  # copy-paste friendly: add brackets if needed
    # =========================================================================

    JOG_DEG = 10.0
    MAX_RETRIES = 2

    uri = uri_if.RMPLAB_Uri(URI_HOST)
    ayal = uri_if.RMPLAB_Uri(AYAL_HOST)
    uri.connect(False)
    ayal.connect(False)

    uri_dash = DashboardHandler(URI_HOST)
    ayal_dash = DashboardHandler(AYAL_HOST)

    fsm = RobotFSM(
        ayal,
        ayal_dash,
        uri_dash,
        name="ayal",
        peer_name="uri",
        auto_clamp_recover=True,
        wrist3_jog_deg=JOG_DEG,
        max_clamp_retries=MAX_RETRIES,
    )
    fsm.connect()

    print(f"[demo] initial fsm state: {fsm.state.value}")
    print(f"[demo] starting Q: {[round(q, 4) for q in ayal.recieve.getActualQ()]}")
    print(f"[demo] target  Q: {[round(q, 4) for q in TARGET_Q]}")
    print(f"[demo] auto_clamp_recover=True, wrist3_jog={JOG_DEG}°, max_retries={MAX_RETRIES}")

    try:
        fsm.move(lambda: ayal.control.moveJ(TARGET_Q, 0.5, 0.5))
        print(f"[demo] moveJ completed; final state: {fsm.state.value}")
    except FaultError as e:
        print(f"[demo] moveJ failed after retries: {e}")
        print(f"[demo] final fsm state: {fsm.state.value}")
        print(f"[demo] final Q: {[round(q, 4) for q in ayal.recieve.getActualQ()]}")
        print(f"[demo] try a different TARGET_Q or increase wrist3_jog_deg / max_clamp_retries and run again")
              
    uri.disconnect()
    ayal.disconnect()
    uri_dash.disconnect()
    ayal_dash.disconnect()

DEMOS = {
    "stop": _demo_protective_stop_mid_move,
    "peer": _demo_peer_fault,
    "prompt": _demo_prompt_recovery,
    "clamp": _demo_clamp_auto_recover,
}

def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "stop"
    if name not in DEMOS:
        print(f"unknown demo {name!r}; choose one of {list(DEMOS)}")
        sys.exit(1)
    DEMOS[name]()

if __name__ == "__main__":
    main()
