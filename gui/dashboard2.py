#!/usr/bin/env python3
import math
import sys
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot, QObject
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QSlider, QLineEdit,
    QPushButton, QGridLayout
)

NUM_PARAMS = 7

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
        self._params = [float]*NUM_PARAMS
    
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
    pitch = property(getPitch, lambda self, value: self.setPitch(value))

    def getParams(self):
        return self._params
    def setParams(self, value):
        emit_flag : bool = False
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
    params = property(getParams, lambda self, value : self.setParams(value))

class DashboardView(QMainWindow):
    arrow_key_pressed = pyqtSignal(str)
    arrow_key_released = pyqtSignal(str)
    slider_changed = pyqtSignal()
    toggle_clicked = pyqtSignal()
    params_changed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Dashboard")
        self.setGeometry(100, 100, 800, 1200)
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
        grid.addWidget(self.label_down,1,1)
        grid.addWidget(self.label_left,1,0)
        grid.addWidget(self.label_right,1,2)
        
        self.label_speed = QLabel("Speed: 0", self)
        self.label_yaw = QLabel("Yaw Angle: 0", self)
        self.label_pitch = QLabel("Received Pitch (deg): 0", self)

        for lb in (self.label_speed, self.label_yaw, self.label_pitch):
            lb.setAlignment(Qt.AlignCenter)
        grid.addWidget(self.label_speed, 2,0,1,3)
        grid.addWidget(self.label_yaw, 3,0,1,3)
        grid.addWidget(self.label_pitch, 4,0,1,3)


        self.slider_left = QSlider(Qt.Vertical, self)
        self.slider_left.setRange(-10, 10)
        self.slider_left.setValue(0)
        self.slider_left.setTickInterval(5)
        self.slider_left.setTickPosition(QSlider.TicksBothSides)

        self.slider_right = QSlider(Qt.Vertical, self)
        self.slider_right.setRange(-10, 10)
        self.slider_right.setValue(0)
        self.slider_right.setTickInterval(5)
        self.slider_right.setTickPosition(QSlider.TicksBothSides)

        grid.addWidget(self.slider_left, 5, 0)
        grid.addWidget(self.slider_right, 5, 2)

        self.label_lm = QLabel("L", self)
        self.label_rm = QLabel("R", self)
        self.label_lm.setAlignment(Qt.AlignCenter)
        self.label_rm.setAlignment(Qt.AlignCenter)
        grid.addWidget(self.label_lm, 6, 0)
        grid.addWidget(self.label_rm, 6, 2)


        self.toggle_button = QPushButton("Toggle Motor ON", self)
        grid.addWidget(self.toggle_button, 7, 1)
        self.toggle_button.clicked.connect(self.toggle_clicked)

        self.paramboxes = []*NUM_PARAMS
        for i in range(NUM_PARAMS):
            tbox = QLineEdit(self)
            self.paramboxes.append(tbox)
            tbox.resize(780, 120)
            grid.addWidget(tbox, 8+i, 1)
        
        self.param_button = QPushButton("Set Parameters", self)
        grid.addWidget(self.param_button, 8+NUM_PARAMS, 1)
        self.param_button.clicked.connect(self.params_changed)

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
            self.arrow_key_pressed.emit("Up")
        elif key == Qt.Key_Down and "Down" not in self.pressed_keys:
            self.pressed_keys.add("Down")
            self.arrow_key_pressed.emit("Down")
        elif key == Qt.Key_Left and "Left" not in self.pressed_keys:
            self.pressed_keys.add("Left")
            self.arrow_key_pressed.emit("Left")
        elif key == Qt.Key_Right and "Right" not in self.pressed_keys:
            self.pressed_keys.add("Right")
            self.arrow_key_pressed.emit("Right")
        self.updateArrowDisplay()
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        key = event.key()
        if key == Qt.Key_Up and "Up" in self.pressed_keys:
            self.pressed_keys.remove("Up")
            self.arrow_key_released.emit("Up")
        elif key == Qt.Key_Down and "Down" in self.pressed_keys:
            self.pressed_keys.remove("Down")
            self.arrow_key_released.emit("Down")
        elif key == Qt.Key_Left and "Left" in self.pressed_keys:
            self.pressed_keys.remove("Left")
            self.arrow_key_released.emit("Left")
        elif key == Qt.Key_Right and "Right" in self.pressed_keys:
            self.pressed_keys.remove("Right")
            self.arrow_key_released.emit("Right")
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
    def __init__(self, model: DashboardModel, view: DashboardView):
        super().__init__()
        self.model = model
        self.view = view
        self.pressed_keys = set()

        self.view.arrow_key_pressed.connect(self.onArrowKeyPressed)
        self.view.arrow_key_released.connect(self.onArrowKeyReleased)
        self.view.slider_changed.connect(self.onSliderChanged)
        self.view.toggle_clicked.connect(self.onToggleClicked)
        self.view.params_changed.connect(self.onSetParamClicked)
        self.model.state_changed.connect(self.updateView)
        self.updateTextboxes()

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

    def onSetParamClicked(self):
        paramvals : list = self.view.getParamValues()
        self.model.params = paramvals
        with open('appcache.txt', 'w') as f:
            for val in paramvals:
                f.write(str(val)+"\n")

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
            f = open('appcache.txt', 'r')
            vals = []
            for val in f.readlines():    
                vals.append(float(val))
            self.model.params = vals.copy()
        except Exception as e:
            print(e)
            pass
        self.model.state_changed.emit()

        self.view = DashboardView()
        self.view.params_changed.emit()
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
