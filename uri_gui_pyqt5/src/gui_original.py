# region imports & parameters (should be moved to json or something)
import sys, time
import multiprocessing
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton, QStackedWidget, QVBoxLayout, QHBoxLayout, QWidget, 
    QGridLayout, QComboBox, QLineEdit
    )
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
    "panelConnection"       : "panelConnection",
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
# endregion

"""
The PyQt5 gui
"""
class MainWindow(QMainWindow):  # main window. includes: status, nav, general_params, sim (optional)
    def __init__(self, command_queue: multiprocessing.Queue = None, sim_process: multiprocessing.Process = None, 
                 sim=None, robots=None, backend="sim"):
        super().__init__()
        self.command_q = command_queue
        self.sim_process = sim_process
        self.sim = sim
        self.robots = robots  # {"ayal": RMPLAB_Uri, "uri": RMPLAB_Uri}
        self.backend = backend  # "sim" or "real"
        self.setGeometry(700, 300, 500, 500)
        self.setWindowTitle(f"PyQt5 gui - {backend.upper()}")
        self.InitUI()

    def closeEvent(self, event):
        """Send quit command and wait for sim process to exit."""
        try:
            if self.backend == "sim" and self.command_q is not None:
                self.command_q.put({"action": "quit"})
                if self.sim_process is not None:
                    self.sim_process.join(timeout=2.0)
                    if self.sim_process.is_alive():
                        self.sim_process.terminate()
                        self.sim_process.join(timeout=1.0)
            elif self.backend == "real" and self.robots is not None:
                # Disconnect real robots
                for robot_name, robot in self.robots.items():
                    try:
                        robot.disconnect()
                        print(f"[GUI] Disconnected {robot_name}")
                    except Exception as e:
                        print(f"[GUI] Error disconnecting {robot_name}: {e}")
        except Exception as e:
            print(f"[GUI] Error in closeEvent: {e}")
        event.accept()
        
    def InitUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        self.initCentralLayout()
        central_widget.setLayout(self.central_layout)
       
        navigation_widget = NavigationWidget(self.command_q)        
        self.central_layout.addWidget(navigation_widget, 2,OTHER_COLUMN,2,1)

    def initPicture(self):
        pixmap = QPixmap("/Users/yarbiv/Library/Containers/net.whatsapp.WhatsApp/Data/tmp/documents/0C56A919-B398-403F-981E-5E555A46203B.jpg") 
        label = QLabel(self)
        label.setGeometry((self.width()-LABEL_WIDTH)//2,(self.height()-LABEL_HEIGHT)//2,LABEL_WIDTH,LABEL_HEIGHT)
        label.setPixmap(pixmap)
        label.setScaledContents(True)

    def initCentralLayout(self):
        l_simulation        = QLabel("simulation", self)
        l_status            = QLabel("status", self)
        l_navigation        = QLabel("", self)
        l_general_params    = QLabel("general_params", self)

        l_simulation    .setStyleSheet("background-color: red;")
        l_status        .setStyleSheet("background-color: blue;")
        l_navigation    .setStyleSheet("background-color: green;")
        l_general_params.setStyleSheet("background-color: orange;")\
        
        self.central_layout = QGridLayout()
        self.central_layout.addWidget(l_simulation,0,SIM_COLUMN,TOTAL_ROWS,OTHER_COLUMN)
        self.central_layout.addWidget(l_status,0,OTHER_COLUMN,1,1)
        self.central_layout.addWidget(l_navigation,1,OTHER_COLUMN,3,1)
        self.central_layout.addWidget(l_general_params,4,OTHER_COLUMN,2,1)

class InputPanel(QWidget): # general panel in the navigation panel
    def __init__(self,description, command_q):
        super().__init__()
        self.description = description
        self.command_q = command_q
        self.values = []
        self.button = QPushButton("Move", self)
        self.layout = QGridLayout()
        self.initLayout()

    def initLayout(self):
        for i in range(DIM):
            label = QLabel(self.description[i], self)
            self.layout.addWidget(label,i,0)
        
        for i in range(DIM):
            value = QLineEdit()
            self.values.append(value)
            self.layout.addWidget(value,i,1)
        
        self.button.clicked.connect(self.on_click) 
        self.layout.addWidget(self.button,DIM,0,1,2)
        self.setLayout(self.layout)

    def on_click(self):
        # read/parse values
        values = []
        for i in range(DIM):
            text = self.values[i].text().strip()
            values.append(float(text) if text else 0.0)
        # send values to robots/sim
        joints = tuple(values)
        command = {"action": "movej","values": joints, "id": 1}
        print(f"sending command to thread: {command}")
        self.command_q.put(command)

    def _on_move_done(self):
        self.button.setText("Move")
        self.button.setEnabled(True)

class AlphaPuzzlePanel(QWidget):
    def __init__(self, description, command_q):
        super().__init__()
        self.description = description
        self.command_q = command_q
        self.buttons = []
        self.layout = QGridLayout()
        self.initLayout()
    
    def initLayout(self):
        for i in range(len(self.description)):
            button = QPushButton(self.description[i],self)
            self.buttons.append(button)
            self.layout.addWidget(button,i,0)
            self.buttons[i].clicked.connect(lambda checked=False, idx=i: self.on_click(idx)) 
                
        self.setLayout(self.layout)

    def on_click(self, idx):
        full_command = {"action": self.description[idx], "id": 1}
        print(f"sending command to thread: {full_command}")
        self.command_q.put(full_command)


class NavigationWidget(QWidget): # holds all possible panels
    def __init__(self, command_q):
        super().__init__()
        self.command_q = command_q
        self.initDDL() # Drop Down List
        self.panelMoveJoints()
        self.panelMoveTCP()
        self.panelAlphaPuzzle()
        self.initStack()
        self.navigation_layout = QGridLayout()   
        self.navigation_layout.addWidget(self.ddl,0,0,1,2)   
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
