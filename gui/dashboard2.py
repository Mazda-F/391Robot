#!/usr/bin/env python3
import math
import sys
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot, QObject
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QSlider, 
    QPushButton, QGridLayout
)

class DashboardModel(QObject):
    state_changed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._control_state = 0
        self._speed = 0
        self._yaw = 0
        self._pitch = 0
    
    def getControlState(self):
        return self.control_state
    
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

    def getPitch(self):
        return self._pitch
    def setPitch(self, value):
        if self._pitch != value:
            self._pitch = value
            self.state_changed.emit()
    pitch = property(getPitch, setPitch)

class DashboardView(QMainWindow):
    arrow_key_pressed = pyqtSignal(str)
    arrow_key_released = pyqtSignal(str)
    slider_changed = pyqtSignal()
    toggle_clicked = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Dashboard")
        self.setGeometry(100, 100, 800, 800)
        self.pressed_keys = set()

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        grid = QGridLayout()
        central_widget.setLayout(grid)

        self.label_up = QLabel("UP", self)
        self.label_down = QLabel("DOWN", self)
        self.label_left = QLabel("L", self)
        self.label_right = QLabel("R", self)
        for lb in (self.label_up, self.label_down, self.label_right, self.label_left):
            lb.setAlignment(Qt.AlignCenter)
            lb.setFixedSize(80,80)
        
        grid.addWidget(self.label_up,0,1)
        grid.addWidget(self.label_up,1,0)
        grid.addWidget(self.label_up,1,1)
        grid.addWidget(self.label_up,1,2)
        
        self.label_speed = QLabel("Speed: 0", self)
        self.label_yaw = QLabel("Yaw Angle: 0", self)
        self.label_pitch = QLabel("Received Pitch (deg): 0", self)

        for lb in (self.label_speed, self.label_yaw, self.label_pitch):
            lb.setAlignment(Qt.AlignCenter)
        grid.addWidget(self.label_speed, 2,0,1,3)
        grid.addWidget(self.label_yaw, 3,0,1,3)
        grid.addWidget(self.label_pitch, 4,0,1,3)

        self.toggle_button = QPushButton("Toggle Motor ON", self)
        grid.addWidget(self.toggle_button, 5, 1)

        self.toggle_button.clicked.connect(self.toggle_clicked)

        central_widget.setFocusPolicy(Qt.StrongFocus)
        central_widget.setFocus()

    def updateArrowDisplay(self):
        styles = {
            True: "background-color: black; color: white; font-size: 25px;",
            False: "background-color: grey; color: white; font-size: 25px;"
        }
        self.label_up.setStyleSheet(styles["Up" in self.pressed_keys])
        self.label_down.setStyleSheet(styles["Down" in self.pressed_keys])
        self.label_left.setStyleSheet(styles["Left" in self.pressed_keys])
        self.label_right.setStyleSheet(styles["Right" in self.pressed_keys])

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Up and "Up" not in self.pressed_keys:
            self.pressed_keys.add("Up")
            self.arrowKeyPressed.emit("Up")
        elif key == Qt.Key_Down and "Down" not in self.pressed_keys:
            self.pressed_keys.add("Down")
            self.arrowKeyPressed.emit("Down")
        elif key == Qt.Key_Left and "Left" not in self.pressed_keys:
            self.pressed_keys.add("Left")
            self.arrowKeyPressed.emit("Left")
        elif key == Qt.Key_Right and "Right" not in self.pressed_keys:
            self.pressed_keys.add("Right")
            self.arrowKeyPressed.emit("Right")
        self.updateArrowDisplay()
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        key = event.key()
        if key == Qt.Key_Up and "Up" in self.pressed_keys:
            self.pressed_keys.remove("Up")
            self.arrowKeyReleased.emit("Up")
        elif key == Qt.Key_Down and "Down" in self.pressed_keys:
            self.pressed_keys.remove("Down")
            self.arrowKeyReleased.emit("Down")
        elif key == Qt.Key_Left and "Left" in self.pressed_keys:
            self.pressed_keys.remove("Left")
            self.arrowKeyReleased.emit("Left")
        elif key == Qt.Key_Right and "Right" in self.pressed_keys:
            self.pressed_keys.remove("Right")
            self.arrowKeyReleased.emit("Right")
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

    def getSliderValues(self):
        return self.slider_left.value(), self.slider_right.value()


class DashboardController(QObject):
    def __init__(self, model: DashboardModel, view: DashboardView):
        super().__init__()
        self.model = model
        self.view = view
        self.pressed_keys = set()

        self.view.arrow_key_pressed.connect(self.onArrowKeyPressed)
        self.view.arrow_key_released.connect(self.onArrowKeyReleased)
        self.view.slider_changed.connect(self.onSliderChanged)
        self.view.toggle_clicked.connect(self.onToggleClicked)
        self.model.state_changed.connect(self.updateView)

    def onArrowKeyPressed(self, direction: str):
        self.pressed_keys.add(direction)
        self.processInput()

    def onArrowKeyReleased(self, direction: str):
        self.pressed_keys.discard(direction)
        self.processInput()

    def processInput(self):
        f = 1 if "Up" in self.pressed_keys else 0
        b = 1 if "Down" in self.pressed_keys else 0
        l = 1 if "Left" in self.pressed_keys else 0
        r = 1 if "Right" in self.pressed_keys else 0
        speed = (f - b) * 100
        yaw = math.atan2(l - r, 1) * 180.0 / math.pi
        self.model.speed = speed
        self.model.yaw = yaw

    def onSliderChanged(self):
        left, right = self.view.getSliderValues()
        self.model.motor_left = left
        self.model.motor_right = right

    def onToggleClicked(self):
        # manual = 0,auto = 1
        self.model.control_state = 1 if self.model.control_state == 0 else 0

    def updateView(self):
        self.view.updateDisplay(self.model)

class Dashboard:
    def __init__(self):
        self.app = QApplication([])
        self.app.setStyleSheet(open("style.css").read())
        self.model = DashboardModel()
        self.view = DashboardView()
        self.controller = DashboardController(self.model, self.view)
        self.view.show()

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

    def exec_(self):
        return self.app.exec_()

if __name__ == "__main__":
    dash = Dashboard()
    exit(dash.exec_())
