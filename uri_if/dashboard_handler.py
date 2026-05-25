from dashboard_client import DashboardClient
import time
import uri_if

class DashboardHandler:
    def __init__(self, ip):
        self.client = DashboardClient(ip)
        self.connect()

    @staticmethod
    def _norm(value):
        return str(value).strip().upper()

    def _robotmode(self):
        return self._norm(self.client.robotmode())

    def _safetymode(self):
        return self._norm(self.client.safetymode())

    def _safetystatus(self):
        return self._norm(self.client.safetystatus())

    def _program_state(self):
        return self._norm(self.client.programState())

    def stop_robot(self):
        self.client.stop()

    def power_on(self):
        self.client.powerOn()

    def power_off(self):
        self.client.powerOff()

    def is_powered_on(self):
        mode = self._robotmode()
        return "POWER_OFF" not in mode

    def is_robot_stopped(self):
        return not self.client.running()
    
    def is_robot_moving(self):
        return self.client.running()
    
    def is_robot_in_error(self):
        safety = f"{self._safetymode()} {self._safetystatus()}"
        error_tokens = ("FAULT", "VIOLATION", "ERROR")
        return any(token in safety for token in error_tokens)
    
    def is_robot_emergency_stopped(self):
        safety = f"{self._safetymode()} {self._safetystatus()}"
        emergency_tokens = ("EMERGENCY_STOP", "SYSTEM_EMERGENCY_STOP", "ROBOT_EMERGENCY_STOP")
        return any(token in safety for token in emergency_tokens)
    
    def is_robot_protective_stopped(self):
        safety = f"{self._safetymode()} {self._safetystatus()}"
        return "PROTECTIVE_STOP" in safety
    
    def is_robot_program_running(self):
        return self.client.running()
    
    def is_robot_program_paused(self):
        return "PAUSED" in self._program_state()
    
    def is_robot_safeguard_stopped(self):
        safety = f"{self._safetymode()} {self._safetystatus()}"
        return "SAFEGUARD_STOP" in safety
    
    def is_robot_ready(self):
        return (
            self.is_robot_connected()
            and self.is_powered_on()
            and not self.is_robot_emergency_stopped()
            and not self.is_robot_protective_stopped()
            and not self.is_robot_in_error()
        )
    
    def is_robot_connected(self):   
        return self.client.isConnected()
    
    def get_robot_mode(self):
        return self.client.robotmode()
    
    def get_robot_message(self):
        return self.client.receive()
    
    def get_robot_version(self):    
        return self.client.polyscopeVersion()
    
    def get_robot_serial_number(self):
        return self.client.getSerialNumber()
    
    def connect(self):
        self.client.connect()

    def disconnect(self):
        self.client.disconnect()

    def pause_program(self):
        self.client.pause()

    def play_program(self):
        self.client.play()

    def unlock_protective_stop(self):
        self.client.unlockProtectiveStop()

    def close_safety_popup(self):
        self.client.closeSafetyPopup()

    def load_program(self, program_name):
        self.client.loadURP(program_name)

    def get_operational_mode(self):
        return "REMOTE_CONTROL" if self.client.isInRemoteControl() else "LOCAL_CONTROL"

    def get_safety_mode(self):
        return self.client.safetymode()
    
    def get_robot_status(self):
        status = {
            "is_powered_on": self.is_powered_on(),
            "is_robot_stopped": self.is_robot_stopped(),
            "is_robot_moving": self.is_robot_moving(),
            "is_robot_in_error": self.is_robot_in_error(),
            "is_robot_emergency_stopped": self.is_robot_emergency_stopped(),
            "is_robot_protective_stopped": self.is_robot_protective_stopped(),
            "is_robot_program_running": self.is_robot_program_running(),
            "is_robot_program_paused": self.is_robot_program_paused(),
            "is_robot_safeguard_stopped": self.is_robot_safeguard_stopped(),
            "is_robot_ready": self.is_robot_ready(),
            "is_robot_connected": self.is_robot_connected(),
            "robot_mode": self.get_robot_mode(),
            "robot_message": self.get_robot_message(),
            "robot_version": self.get_robot_version(),
            "robot_serial_number": self.get_robot_serial_number(),
            "operational_mode": self.get_operational_mode(),
            "safety_mode": self.get_safety_mode()
        }
        return status
    
    def wait_until_ready(self, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_robot_ready():
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_stopped(self, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_robot_stopped():
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_moving(self, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_robot_moving():
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_error(self, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_robot_in_error():
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_emergency_stopped(self, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_robot_emergency_stopped():
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_protective_stopped(self, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_robot_protective_stopped():
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_program_running(self, timeout=30):   
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_robot_program_running():
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_program_paused(self, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_robot_program_paused():
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_safeguard_stopped(self, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_robot_safeguard_stopped():
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_connected(self, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.is_robot_connected():
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_disconnected(self, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self.is_robot_connected():
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_mode(self, mode, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.get_robot_mode() == mode:
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_message(self, message, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.get_robot_message() == message:
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_version(self, version, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.get_robot_version() == version:
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_serial_number(self, serial_number, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.get_robot_serial_number() == serial_number:
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_operational_mode(self, operational_mode, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.get_operational_mode() == operational_mode:
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_safety_mode(self, safety_mode, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.get_safety_mode() == safety_mode:
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_status(self, status_key, status_value, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_robot_status()
            if status.get(status_key) == status_value:
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_all_status(self, status_dict, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_robot_status()
            if all(status.get(key) == value for key, value in status_dict.items()):
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_any_status(self, status_dict, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_robot_status()
            if any(status.get(key) == value for key, value in status_dict.items()):
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_not_status(self, status_key, status_value, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_robot_status()
            if status.get(status_key) != status_value:
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_not_all_status(self, status_dict, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_robot_status()
            if not all(status.get(key) == value for key, value in status_dict.items()):
                return True
            time.sleep(0.5)
        return False   
    
    def wait_until_not_any_status(self, status_dict, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_robot_status()
            if not any(status.get(key) == value for key, value in status_dict.items()):
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_condition(self, condition_func, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if condition_func(self.get_robot_status()):
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_not_condition(self, condition_func, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not condition_func(self.get_robot_status()):
                return True
            time.sleep(0.5)
        return False
    
    def wait_until(self, condition_func, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if condition_func(self.get_robot_status()):
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_not(self, condition_func, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not condition_func(self.get_robot_status()):
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_all(self, condition_funcs, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if all(condition_func(self.get_robot_status()) for condition_func in condition_funcs):
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_any(self, condition_funcs, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if any(condition_func(self.get_robot_status()) for condition_func in condition_funcs):
                return True
            time.sleep(0.5)
        return False
        
    def wait_until_not_all(self, condition_funcs, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not all(condition_func(self.get_robot_status()) for condition_func in condition_funcs):
                return True
            time.sleep(0.5)
        return False
        
    def wait_until_not_any(self, condition_funcs, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not any(condition_func(self.get_robot_status()) for condition_func in condition_funcs):
                return True
            time.sleep(0.5)
        return False
    
    def wait_until_timeout(self, timeout=30):
        time.sleep(timeout)
        return True
    
    def wait_until_not_timeout(self, timeout=30):
        start_time = time.time()
        while time.time() - start_time < timeout:
            time.sleep(0.5)
        return True
    
def main():
    dashboard = DashboardHandler(uri_if.HOST_AYAL)
    dashboard.unlock_protective_stop()
    # dashboard.close_safety_popup()

if __name__ == "__main__":
    main()