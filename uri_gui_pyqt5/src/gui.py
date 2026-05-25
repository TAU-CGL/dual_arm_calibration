# region imports & parameters (should be moved to json or something)
import sys, time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton, QStackedWidget, QVBoxLayout, QHBoxLayout, QWidget,
    QGridLayout, QComboBox, QLineEdit, QRadioButton, QButtonGroup, QCheckBox
    )
import os
import uri_gui_pyqt5.src.utils_1 as utils_1
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
# picture
LABEL_WIDTH = 100
LABEL_HEIGHT = 100
# grid layout
TOTAL_ROWS = 6
OTHER_COLUMN = 4
SIM_COLUMN = 0
SIM_WIDTH = TOTAL_ROWS
DEFAULT_JOINTS_URI = (0,0,0,0,0,0)
DEFAULT_JOINTS_AYAL = (0,0,0,0,0,0)
BASE_AYAL = (0.5,-0.97,0.25,0,0,0)
BASE_URI = (0,0,0,0,0,0)
DIM = 6
DDL_HANDLE = {
    "panelAlphaPuzzle"      : "panelAlphaPuzzle",
    "panelCalibrate"        : "panelCalibrate",
    "panelExploreForce"     : "panelExploreForce",
    "panelGripper"          : "panelGripper",
    "panelMeetRobots"       : "panelMeetRobots",
    "panelMoveLoop"         : "panelMoveLoop",
    "panelMoveOffset"       : "panelMoveOffset",
    "move joints (rad/deg)" : "panelMoveJoints",
    "move TCP (mm/cm/m)"    : "panelMoveTCP",
    "panelPNP"              : "panelPNP",
    "panelTCPForce"         : "panelTCPForce",
    "panelX"                : "panelX"
}
SIM_RENDER_RES = 1.0 / 240.0
ROBOTS_IDX = {"uri": 0, "ayal": 1}
# endregion

"""
The PyQt5 gui
"""
class MainWindow(QMainWindow):  # main window. includes: status, nav, general_params, sim (optional)
    def __init__(self, backend):
        super().__init__()
        self.backend = backend
        self.setGeometry(700, 300, 500, 500)
        self.setWindowTitle(f"PyQt5 gui - {type(backend).__name__}")
        self.InitUI()

    def closeEvent(self, event):
        """Send quit command and wait for backend to shut down."""
        try:
            self.backend.shutdown()
        except Exception as e:
            print(f"[GUI] Error in closeEvent: {e}")
        event.accept()
        
    def InitUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.initCentralLayout()
        central_widget.setLayout(self.central_layout)

    def initPicture(self):
        pixmap = QPixmap("/Users/yarbiv/Library/Containers/net.whatsapp.WhatsApp/Data/tmp/documents/0C56A919-B398-403F-981E-5E555A46203B.jpg") 
        label = QLabel(self)
        label.setGeometry((self.width()-LABEL_WIDTH)//2,(self.height()-LABEL_HEIGHT)//2,LABEL_WIDTH,LABEL_HEIGHT)
        label.setPixmap(pixmap)
        label.setScaledContents(True)

    def initCentralLayout(self):
        l_simulation        = QLabel("simulation", self)

        l_simulation    .setStyleSheet("background-color: red;")

        self.connection_panel = ConnectionPanel(self.backend)
        self.connection_panel.setStyleSheet("background-color: blue;")

        self.general_params_panel = GeneralParamsPanel(self.backend)
        self.general_params_panel.setStyleSheet("background-color: orange;")

        self.central_layout = QGridLayout()
        self.central_layout.addWidget(l_simulation,0,SIM_COLUMN,TOTAL_ROWS,OTHER_COLUMN)
        self.central_layout.addWidget(self.connection_panel,0,OTHER_COLUMN,1,1)
        self.navigation_widget = NavigationWidget(self.backend, self.connection_panel)
        self.navigation_widget.setStyleSheet("background-color: green;")
        self.central_layout.addWidget(self.navigation_widget,1,OTHER_COLUMN,3,1)
        self.central_layout.addWidget(self.general_params_panel,4,OTHER_COLUMN,2,1)


class GeneralParamsPanel(QWidget):
    """Right-side panel for general toggles. Currently: reachability grid overlay."""

    def __init__(self, backend):
        super().__init__()
        self.backend = backend

        cfg = utils_1.load_config().get("config_data", {}).get("reach_grid", {})
        self._npz_path = cfg.get("npz_path", "")
        self._point_size = float(cfg.get("point_size", 4.0))
        initial_enabled = bool(cfg.get("enabled", False)) and bool(self._npz_path)
        initial_show_unreachable = bool(cfg.get("show_unreachable", False))

        self.cb_grid = QCheckBox("Show reachability grid")
        self.cb_grid.setChecked(initial_enabled)
        self.cb_grid.toggled.connect(self._on_toggle)

        self.cb_unreachable = QCheckBox("Show unreachable cells (-1)")
        self.cb_unreachable.setChecked(initial_show_unreachable)
        self.cb_unreachable.toggled.connect(self._on_toggle)

        layout = QVBoxLayout()
        layout.addWidget(self.cb_grid)
        layout.addWidget(self.cb_unreachable)
        layout.addStretch(1)
        self.setLayout(layout)

    def _on_toggle(self, _checked):
        # Always clear first so the overlay reflects the latest checkbox state.
        self.backend.send_command({"action": "clear_reach_grid"})
        if not self.cb_grid.isChecked():
            return
        if not self._npz_path or not os.path.isfile(self._npz_path):
            print(f"[GUI] reach_grid: npz_path missing or not found: {self._npz_path!r}")
            return
        self.backend.send_command({
            "action": "show_reach_grid",
            "npz_path": self._npz_path,
            "point_size": self._point_size,
            "show_unreachable": self.cb_unreachable.isChecked(),
        })

class InputPanel(QWidget): # general panel in the navigation panel
    def __init__(self, labels, backend, command_id_getter, action="movej", button_text="Move"):
        super().__init__()
        self.labels = labels
        self.backend = backend
        self.command_id_getter = command_id_getter
        self.action = action
        self.button_text = button_text
        self.values = []
        self.button = QPushButton(button_text, self)
        self.layout = QGridLayout()
        self.initLayout()

    def initLayout(self):
        for i in range(DIM):
            label = QLabel(self.labels[i], self)
            self.layout.addWidget(label, i, 0)

        for i in range(DIM):
            value = QLineEdit()
            self.values.append(value)
            self.layout.addWidget(value, i, 1)

        self.button.clicked.connect(self.on_click)
        self.layout.addWidget(self.button, DIM, 0, 1, 2)
        self.setLayout(self.layout)

    def on_click(self):
        values = []
        for i in range(DIM):
            text = self.values[i].text().strip()
            values.append(float(text) if text else 0.0)

        command_id = self.command_id_getter(self.action)
        command = {"action": self.action, "values": tuple(values), "id": command_id}
        print(f"[GUI] sending command: {command}")
        self.backend.send_command(command)


class ActionPanel(QWidget):
    def __init__(self, commands, backend, command_id_getter, button_text="Action"):
        super().__init__()
        self.commands = commands
        self.backend = backend
        self.command_id_getter = command_id_getter
        self.layout = QGridLayout()
        self.initLayout()

    def initLayout(self):
        for i, cmd in enumerate(self.commands):
            button = QPushButton(cmd, self)
            self.layout.addWidget(button, i, 0)
            button.clicked.connect(lambda checked=False, action=cmd: self.on_click(action))
        self.setLayout(self.layout)

    def on_click(self, action):
        command_id = self.command_id_getter(action)
        command = {"action": action, "id": command_id}
        print(f"[GUI] sending command: {command}")
        self.backend.send_command(command)


class ConnectionPanel(QWidget):
    def __init__(self, backend):
        super().__init__()
        self.backend = backend
        self.selected_robot = ROBOTS_IDX["uri"]

        self.layout = QHBoxLayout()
        self.btn_group = QButtonGroup(self)

        self.radio_uri = QRadioButton("URI")
        self.radio_ayal = QRadioButton("AYAL")
        self.radio_uri.setChecked(True)

        self.btn_group.addButton(self.radio_uri, ROBOTS_IDX["uri"])
        self.btn_group.addButton(self.radio_ayal, ROBOTS_IDX["ayal"])
        self.btn_group.idToggled.connect(self._on_toggled)

        self.layout.addWidget(self.radio_uri)
        self.layout.addWidget(self.radio_ayal)
        self.setLayout(self.layout)

    def _on_toggled(self, id, checked):
        if checked:
            self.selected_robot = id


class AlphaPuzzlePanel(QWidget):
    def __init__(self, description, backend, command_id_getter):
        super().__init__()
        self.description = description
        self.backend = backend
        self.command_id_getter = command_id_getter
        self.buttons = []
        self.layout = QGridLayout()
        self.initLayout()

    def initLayout(self):
        for i, label in enumerate(self.description):
            button = QPushButton(label, self)
            self.buttons.append(button)
            self.layout.addWidget(button, i, 0)
            button.clicked.connect(lambda checked=False, action=label: self.on_click(action))
        self.setLayout(self.layout)

    def on_click(self, action):
        command_id = 0  # both robots for puzzle actions
        command = {"action": action, "id": command_id}
        print(f"[GUI] sending command: {command}")
        self.backend.send_command(command)


class NavigationWidget(QWidget): # holds all possible panels
    def __init__(self, backend, connection_panel):
        super().__init__()
        self.backend = backend
        self.connection_panel = connection_panel

        self.initDDL() # Drop Down List
        self.panelAlphaPuzzle()
        self.panelCalibrate()
        self.panelExploreForce()
        self.panelGripper()
        self.panelMeetRobots()
        self.panelMoveLoop()
        self.panelMoveOffset()
        self.panelMoveJoints()
        self.panelMoveTCP()
        self.panelPNP()
        self.panelTCPForce()
        self.panelX()

        self.initStack()
        self.navigation_layout = QGridLayout()
        self.navigation_layout.addWidget(self.ddl, 0, 0, 1, 2)
        self.navigation_layout.addWidget(self.navigation_stack)
        self.setLayout(self.navigation_layout)

    def initDDL(self): # Drop Down List
        self.ddl = QComboBox()
        for desc, panel in DDL_HANDLE.items():
            self.ddl.addItem(desc)
        self.ddl.currentIndexChanged.connect(self.onDDLChanged)

    def initStack(self): # Careful! must be stacked in order, on onDDLChanged we set them by index!
        self.navigation_stack = QStackedWidget()
        self.navigation_stack.addWidget(self.alpha_puzzle_panel)
        self.navigation_stack.addWidget(self.calibrate_panel)
        self.navigation_stack.addWidget(self.explore_force_panel)
        self.navigation_stack.addWidget(self.gripper_panel)
        self.navigation_stack.addWidget(self.meet_robots_panel)
        self.navigation_stack.addWidget(self.move_loop_panel)
        self.navigation_stack.addWidget(self.move_offset_panel)
        self.navigation_stack.addWidget(self.move_joints_panel)
        self.navigation_stack.addWidget(self.move_tcp_panel)
        self.navigation_stack.addWidget(self.pnp_panel)
        self.navigation_stack.addWidget(self.tcp_force_panel)
        self.navigation_stack.addWidget(self.panel_x_widget)

    def onDDLChanged(self, idx):
        self.navigation_stack.setCurrentIndex(idx)

    def get_command_id(self, action):
        # Puzzle actions are both robots
        puzzle_actions = {"prepare_puzzle", "assemble_puzzle", "dismantle_puzzle", "separate_puzzle"}
        if action in puzzle_actions:
            return 0
        if self.connection_panel is not None:
            return self.connection_panel.selected_robot
        return 1

    def panelAlphaPuzzle(self):
        buttons = ["prepare_puzzle", "assemble_puzzle", "dismantle_puzzle", "separate_puzzle"]
        self.alpha_puzzle_panel = AlphaPuzzlePanel(buttons, self.backend, self.get_command_id)

    def panelCalibrate(self):
        labels = ["X", "Y", "Z", "R_x", "R_y", "R_z"]
        self.calibrate_panel = QWidget()
        layout = QVBoxLayout()

        self.calibrate_input = InputPanel(labels, self.backend, self.get_command_id, action="calibrate", button_text="Calibrate")
        layout.addWidget(self.calibrate_input)

        extra_actions = ["print_tcp", "copy_calibration", "teachmode_toggle", "base_both", "add_sample", "guide2sample_toggle"]
        self.calibrate_actions = ActionPanel(extra_actions, self.backend, self.get_command_id)
        layout.addWidget(self.calibrate_actions)

        self.calibrate_panel.setLayout(layout)

    def panelExploreForce(self):
        buttons = ["start_explore_force", "stop_explore_force"]
        self.explore_force_panel = ActionPanel(buttons, self.backend, self.get_command_id)

    def panelGripper(self):
        buttons = ["gripper_open", "gripper_close", "gripper_move"]
        self.gripper_panel = ActionPanel(buttons, self.backend, self.get_command_id)

    def panelMeetRobots(self): # former move_to
        buttons = ["meet_robots", "home_both"]
        self.meet_robots_panel = ActionPanel(buttons, self.backend, self.get_command_id)

    def panelMoveJoints(self): # former actual_q
        joint_labels = ["Base", "Shoulder", "Elbow", "wrist1", "wrist2", "wrist3"]
        self.move_joints_panel = InputPanel(joint_labels, self.backend, self.get_command_id, action="movej", button_text="Move joints")

    def panelMoveLoop(self): # former loop_move
        buttons = ["move_loop_start", "move_loop_stop"]
        self.move_loop_panel = ActionPanel(buttons, self.backend, self.get_command_id)

    def panelMoveOffset(self): # former move_tcp (+ maybe move_both_tcp)
        labels = ["DX", "DY", "DZ", "DRx", "DRy", "DRz"]
        self.move_offset_panel = InputPanel(labels, self.backend, self.get_command_id, action="move_offset", button_text="Move offset")

    def panelMoveTCP(self): # former actual_tcp # TODO add frames - uri,ayal,tcp_uri,tcp_ayal
        tcp_pos_labels = ["X", "Y", "Z", "R_x", "R_y", "R_z"]
        self.move_tcp_panel = InputPanel(tcp_pos_labels, self.backend, self.get_command_id, action="movel", button_text="Move TCP")

    def panelPNP(self):
        buttons = ["pickup", "place"]
        self.pnp_panel = ActionPanel(buttons, self.backend, self.get_command_id)

    def panelTCPForce(self):
        buttons = ["tcp_force_start", "tcp_force_stop"]
        self.tcp_force_panel = ActionPanel(buttons, self.backend, self.get_command_id)

    def panelX(self): # NEW! all the extra
        buttons = ["custom_action_1", "custom_action_2", "custom_action_3"]
        self.panel_x_widget = ActionPanel(buttons, self.backend, self.get_command_id)


class WorkerThread(QThread): # prevents the gui from freezing
    finished = pyqtSignal()

    def run(self):
        time.sleep(3) # TODO replace with robot/sim behavior
        self.finished.emit()

    def initDDL(self): # Drop Down List
        self.ddl = QComboBox()
        for desc, panel in DDL_HANDLE.items():
            self.ddl.addItem(desc)
        self.ddl.currentIndexChanged.connect(self.onDDLChanged)
        
    def initStack(self): # Careful! must be stacked in order, on onDDLChanged we set them by index!
        self.navigation_stack = QStackedWidget()
        self.navigation_stack.addWidget(self.alpha_puzzle_panel)
        self.navigation_stack.addWidget(self.move_joints_panel)
        self.navigation_stack.addWidget(self.move_tcp_panel)

    def onDDLChanged(self, idx):
        self.navigation_stack.setCurrentIndex(idx)

    """
    panels
    """
    def panelAlphaPuzzle(self):
        buttons = ["prepare_puzzle", "assemble_puzzle", "dismantle_puzzle","separate_puzzle"]
        self.alpha_puzzle_panel = AlphaPuzzlePanel(buttons,self.command_q)

    def panelCalibrate(self):
        pass

    def panelConnection(self):
        pass

    def panelExploreForce(self):
        pass
    
    def panelGripper(self):
        pass

    def panelMeetRobots(self): # former move_to
        pass

    def panelMoveJoints(self): # former actual_q
        joint_labels = ["Base", "Shoulder", "Elbow", "wrist1", "wrist2", "wrist3"]
        self.move_joints_panel = InputPanel(joint_labels, self.command_q)

    def panelMoveLoop(self): # former loop_move
        pass

    def panelMoveOffset(self): # former move_tcp (+ maybe move_both_tcp)
        pass

    def panelMoveTCP(self): # former actual_tcp # TODO add frames - uri,ayal,tcp_uri,tcp_ayal
        tcp_pos_labels = ["X", "Y", "Z", "R_x", "R_y", "R_z"]
        self.move_tcp_panel = InputPanel(tcp_pos_labels, self.command_q)

    def panelPNP(self):
        pass

    def panelTCPForce(self):
        pass

    def panelX(self): # NEW! all the extra
        pass

class WorkerThread(QThread): # prevents the gui from freezing
    finished = pyqtSignal()

    def run(self):
        time.sleep(3) # TODO replace with robot/sim behavior
        self.finished.emit()

