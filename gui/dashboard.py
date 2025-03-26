#!/usr/bin/env python3
import math
import sys
import threading
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot, QObject
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit,
    QPushButton, QGridLayout, QTabWidget, QVBoxLayout
)
from stream import MjpegStreamThread, MjpegStreamWidget
NUM_PARAMS = 9

class DashboardModel(QObject):
    state_changed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._control_state = 0
        self._speed = 0
        self._yaw = 0
        self._pitch = 0
        self._motor_left = 0
        self._motor_right = 0
        self._params = [0.0]*NUM_PARAMS

    def getControlState(self):
        return self._control_state
    def setControlState(self, value):
        if self._control_state != value:
            self._control_state = value
            self.state_changed.emit()
    control_state = property(getControlState, setControlState)

    def getSpeed(self):
        return self._speed
    def setSpeed(self, value):
        if self._speed != value:
            self._speed = value
            self.state_changed.emit()
    speed = property(getSpeed, setSpeed)

    def getYaw(self):
        return self._yaw
    def setYaw(self, value):
        if self._yaw != value:
            self._yaw = value
            self.state_changed.emit()
    yaw = property(getYaw, setYaw)

    def getMotorLeft(self):
        return self._motor_left
    def setMotorLeft(self, value):
        if self._motor_left != value:
            self._motor_left = value
            self.state_changed.emit()
    motor_left = property(getMotorLeft, setMotorLeft)

    def getMotorRight(self):
        return self._motor_right
    def setMotorRight(self, value):
        if self._motor_right != value:
            self._motor_right = value
            self.state_changed.emit()
    motor_right = property(getMotorRight, setMotorRight)

    def getPitch(self):
        return self._pitch
    def setPitch(self, value):
        if self._pitch != value:
            self._pitch = value
            self.state_changed.emit()
    pitch = property(getPitch, setPitch)

    def getParams(self):
        return self._params
    def setParams(self, value):
        emit_flag = False
        if len(self._params) != len(value):
            self._params = value.copy()
            emit_flag = True
        else:
            if value and self._params:
                for i in range(len(value)):
                    if self._params[i] != value[i]:
                        self._params[i] = value[i]
                        emit_flag = True
        if emit_flag:
            self.state_changed.emit()
    params = property(getParams, setParams)


class DashboardView(QMainWindow):
    arrow_key_pressed = pyqtSignal(str)
    arrow_key_released = pyqtSignal(str)
    toggle_clicked = pyqtSignal()
    params_changed = pyqtSignal()
    bluetooth_toggle_clicked = pyqtSignal()  # new signal for bt toggling

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Dashboard")
        self.setGeometry(100, 100, 800, 1200)
        self.pressed_keys = set()

        # tab1
        self.tabs = QTabWidget(self)
        self.setCentralWidget(self.tabs)

        self.tab_dashboard = QWidget()
        self.tabs.addTab(self.tab_dashboard, "Dashboard")
        dashboard_layout = QVBoxLayout()
        self.tab_dashboard.setLayout(dashboard_layout)

        self.stream_widget = MjpegStreamWidget(self)
        self.stream_widget.setMinimumHeight(400)
        dashboard_layout.addWidget(self.stream_widget)

        telemetrics_widget = QWidget(self)
        grid = QGridLayout()
        telemetrics_widget.setLayout(grid)

        self.label_up = QLabel("W", self)
        self.label_down = QLabel("S", self)
        self.label_left = QLabel("A", self)
        self.label_right = QLabel("D", self)
        for lb in (self.label_up, self.label_down, self.label_left, self.label_right):
            lb.setAlignment(Qt.AlignCenter)
            lb.setFixedSize(80,80)
        grid.addWidget(self.label_up, 0, 1)
        grid.addWidget(self.label_down, 1, 1)
        grid.addWidget(self.label_left, 1, 0)
        grid.addWidget(self.label_right, 1, 2)

        self.label_speed = QLabel("Speed: 0", self)
        self.label_yaw = QLabel("Yaw Angle: 0", self)
        self.label_pitch = QLabel("Received Pitch (deg): 0", self)
        for lb in (self.label_speed, self.label_yaw, self.label_pitch):
            lb.setAlignment(Qt.AlignCenter)
        grid.addWidget(self.label_speed, 2, 0, 1, 3)
        grid.addWidget(self.label_yaw, 3, 0, 1, 3)
        grid.addWidget(self.label_pitch, 4, 0, 1, 3)

        self.toggle_button = QPushButton("Toggle Motor ON", self)
        grid.addWidget(self.toggle_button, 5, 1)
        self.toggle_button.clicked.connect(self.toggle_clicked)

        # bt connect 
        self.bt_button = QPushButton("Connect Bluetooth", self)
        grid.addWidget(self.bt_button, 6, 1)
        self.bt_button.clicked.connect(self.bluetooth_toggle_clicked)

        dashboard_layout.addWidget(telemetrics_widget)

        # tab 2
        self.tab_settings = QWidget()
        self.tabs.addTab(self.tab_settings, "Settings")
        settings_layout = QGridLayout()
        self.tab_settings.setLayout(settings_layout)
        self.paramboxes = []
        for i in range(NUM_PARAMS):
            tbox = QLineEdit(self)
            self.paramboxes.append(tbox)
            settings_layout.addWidget(tbox, i, 0)
        self.param_button = QPushButton("Set Parameters", self)
        settings_layout.addWidget(self.param_button, NUM_PARAMS, 0)
        self.param_button.clicked.connect(self.params_changed)

        self.tabs.setFocusPolicy(Qt.StrongFocus)
        self.tabs.setFocus()

    def updateArrowDisplay(self):
        styles = {
            True: "background-color: black; color: white; font-size: 25px;",
            False: "background-color: grey; color: white; font-size: 25px;"
        }
        self.label_up.setStyleSheet(styles["w" in self.pressed_keys])
        self.label_down.setStyleSheet(styles["s" in self.pressed_keys])
        self.label_left.setStyleSheet(styles["a" in self.pressed_keys])
        self.label_right.setStyleSheet(styles["d" in self.pressed_keys])

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_W and "w" not in self.pressed_keys:
            self.pressed_keys.add("w")
            self.arrow_key_pressed.emit("w")
        elif key == Qt.Key_S and "s" not in self.pressed_keys:
            self.pressed_keys.add("s")
            self.arrow_key_pressed.emit("s")
        elif key == Qt.Key_A and "a" not in self.pressed_keys:
            self.pressed_keys.add("a")
            self.arrow_key_pressed.emit("a")
        elif key == Qt.Key_D and "d" not in self.pressed_keys:
            self.pressed_keys.add("d")
            self.arrow_key_pressed.emit("d")
        self.updateArrowDisplay()
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        key = event.key()
        if key == Qt.Key_W and "w" in self.pressed_keys:
            self.pressed_keys.remove("w")
            self.arrow_key_released.emit("w")
        elif key == Qt.Key_S and "s" in self.pressed_keys:
            self.pressed_keys.remove("s")
            self.arrow_key_released.emit("s")
        elif key == Qt.Key_A and "a" in self.pressed_keys:
            self.pressed_keys.remove("a")
            self.arrow_key_released.emit("a")
        elif key == Qt.Key_D and "d" in self.pressed_keys:
            self.pressed_keys.remove("d")
            self.arrow_key_released.emit("d")
        self.updateArrowDisplay()
        super().keyReleaseEvent(event)

    def updateDisplay(self, model: DashboardModel):
        self.label_speed.setText(f"Speed: {model.speed}")
        self.label_yaw.setText(f"Yaw Angle: {model.yaw:.1f}")
        self.label_pitch.setText(f"Received Pitch: {model.pitch} deg")
        if model.control_state == 0:
            self.toggle_button.setText("Toggle Motor ON")
        else:
            self.toggle_button.setText("Toggle Motor OFF")

    def getParamValues(self):
        paramvals = []
        for box in self.paramboxes:
            try:
                val = float(box.text())
                if val is float('NaN') or val is float('inf'):
                    raise ValueError
            except ValueError:
                val = 0.0
            paramvals.append(val)
        return paramvals

class DashboardController(QObject):
    def __init__(self, model: DashboardModel, view: DashboardView, dashboard):
        super().__init__()
        self.model = model
        self.view = view
        self.dashboard = dashboard  # ref the dashboard to toggle bt
        self.pressed_keys = set()

        self.view.arrow_key_pressed.connect(self.onArrowKeyPressed)
        self.view.arrow_key_released.connect(self.onArrowKeyReleased)
        self.view.toggle_clicked.connect(self.onToggleClicked)
        self.view.params_changed.connect(self.onSetParamClicked)
        self.view.bluetooth_toggle_clicked.connect(self.onBluetoothToggleClicked)
        self.model.state_changed.connect(self.updateView)
        self.updateTextboxes()

    def onArrowKeyPressed(self, direction: str):
        self.pressed_keys.add(direction)
        self.processInput()

    def onArrowKeyReleased(self, direction: str):
        self.pressed_keys.discard(direction)
        self.processInput()

    def processInput(self):
        f = 1 if "w" in self.pressed_keys else 0
        b = 1 if "s" in self.pressed_keys else 0
        l = 1 if "a" in self.pressed_keys else 0
        r = 1 if "d" in self.pressed_keys else 0
        speed = (f - b) * 100
        yaw = math.atan2(l - r, 1) * 180.0 / math.pi
        self.model.speed = speed
        self.model.yaw = yaw

    def onToggleClicked(self):
        self.model.control_state = 1 if self.model.control_state == 0 else 0

    def onSetParamClicked(self):
        paramvals = self.view.getParamValues()
        self.model.params = paramvals
        with open('appcache.txt', 'w') as f:
            for val in paramvals:
                f.write(str(val) + "\n")

    def onBluetoothToggleClicked(self):
        # use dashboard method to connect bt
        self.dashboard.toggleBluetooth()

    def updateView(self):
        self.view.updateDisplay(self.model)

    def updateTextboxes(self):
        params = self.model.getParams()
        for i in range(min(len(params), len(self.view.paramboxes))):
            self.view.paramboxes[i].setText(str(params[i]))

class Dashboard:
    def __init__(self):
        self.app = QApplication([])
        self.app.setStyleSheet(open("style.css").read())
        self.model = DashboardModel()
        try:
            with open('appcache.txt', 'r') as f:
                vals = [float(val) for val in f.readlines()]
                self.model.params = vals.copy()
        except Exception as e:
            self.model.params = [0.0]*NUM_PARAMS
        self.model.state_changed.emit()

        self.view = DashboardView()
        self.view.params_changed.emit()
        # pass self to controller so it can call toggleBluetooth()
        self.controller = DashboardController(self.model, self.view, self)
        self.view.updateArrowDisplay()
        self.view.show()

        # bt: initially no connection
        self.bluetooth = None
        self.bluetooth_thread = None

    def toggleBluetooth(self):
        from bluetooth import Bluetooth 
        if self.bluetooth is None:
            # start bt in new thread
            self.bluetooth = Bluetooth("ROBOT_C4", self)
            self.bluetooth_thread = threading.Thread(target=self.bluetooth.start, daemon=True)
            self.bluetooth_thread.start()
            self.view.bt_button.setText("Disconnect Bluetooth")
        else:
            # stop bt and join thread
            self.bluetooth.stop()
            self.bluetooth_thread.join()
            self.bluetooth = None
            self.view.bt_button.setText("Connect Bluetooth")

    @property
    def speed(self):
        return self.model.speed
    @speed.setter
    def speed(self, value):
        self.model.speed = value

    @property
    def yaw(self):
        return self.model.yaw
    @yaw.setter
    def yaw(self, value):
        self.model.yaw = value

    @property
    def motor_left(self):
        return self.model.motor_left
    @motor_left.setter
    def motor_left(self, value):
        self.model.motor_left = value

    @property
    def motor_right(self):
        return self.model.motor_right
    @motor_right.setter
    def motor_right(self, value):
        self.model.motor_right = value

    @property
    def control_state(self):
        return self.model.control_state
    @control_state.setter
    def control_state(self, value):
        self.model.control_state = value

    @property
    def pitch(self):
        return self.model.pitch
    @pitch.setter
    def pitch(self, value):
        QTimer.singleShot(0, lambda: self.model.setPitch(value))

    @property
    def params(self):
        return self.model.params
    @params.setter
    def params(self, value):
        QTimer.singleShot(0, lambda: self.model.setParams(value))
        self.model.params = value.copy()

    def exec_(self):
        return self.app.exec_()

if __name__ == "__main__":
    dash = Dashboard()
    exit(dash.exec_())
