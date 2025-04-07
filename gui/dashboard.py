#!/usr/bin/env python3
import customtkinter as ctk
import threading, math, os
from multiprocessing import Process, Queue
from stream import StreamWidget
from config import *
from bluetooth import main_bluetooth_process
import queue
import numpy as np
import time
from config import *

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class DashboardModel:
    def __init__(self):
        self._control_state = 0
        self._speed = 0
        self._yaw = 0
        self._pitch = 0
        self._gear = 1
        self._energy = 0.0
        self._rpm = 0.0
        self._telemetry = [0.0] * NUM_DIN
        self._params = [0.0] * NUM_PARAMS
        self._observers = []
        self._bluetooth_connected = False

    def add_observer(self, callback):
        self._observers.append(callback)

    def notify_state_changed(self):
        for callback in self._observers:
            callback()

    @property
    def bluetooth_connected(self):
        return self._bluetooth_connected
    @bluetooth_connected.setter
    def bluetooth_connected(self, value : bool):
        if self._bluetooth_connected != value:
            self._bluetooth_connected = value
            self.notify_state_changed()

    @property
    def control_state(self):
        return self._control_state

    @control_state.setter
    def control_state(self, value):
        if self._control_state != value:
            self._control_state = value
            self.notify_state_changed()

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        if self._speed != value:
            self._speed = value
            self.notify_state_changed()

    @property
    def gear(self):
        return self._gear
    @gear.setter
    def gear(self, value):
        if self._gear != value:
            self._gear = value
            self.notify_state_changed()

    @property
    def energy(self):
        return self._gear
    @energy.setter
    def energy(self, value):
        if self._energy != value:
            self._energy = value
            self.notify_state_changed()

    @property
    def rpm(self):
        return self._rpm
    @rpm.setter
    def rpm(self, value):
        if self._rpm != value:
            self._rpm = value
            self.notify_state_changed()

    @property
    def yaw(self):
        return self._yaw
    @yaw.setter
    def yaw(self, value):
        if self._yaw != value:
            self._yaw = value
            self.notify_state_changed()

    @property
    def telemetry(self):
        return self._telemetry

    @telemetry.setter
    def telemetry(self, value):
        telemetry_changed = False
        if len(self._telemetry) != len(value):
            self._telemetry = value.copy()
            telemetry_changed = True
        else:
            for i in range(len(value)):
                if self._telemetry[i] != value[i]:
                    self._telemetry[i] = value[i]
                    telemetry_changed = True
        if telemetry_changed:
            self.notify_state_changed()

    @property
    def params(self):
        return self._params

    @params.setter
    def params(self, value):
        changed = False
        if len(self._params) != len(value):
            self._params = value.copy()
            changed = True
        else:
            for i in range(len(value)):
                if self._params[i] != value[i]:
                    self._params[i] = value[i]
                    changed = True
        if changed:
            self.notify_state_changed()


class DashboardView(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("C4 Dashboard")
        self.geometry("1380x820")
        self.pressed_keys = set()
        self.pressed_shift = set()

        self.tabview = ctk.CTkTabview(self, width=1200, height=720)
        self.tabview.pack(fill="both", expand=True)
        self.tabview.add("Dashboard")
        self.tabview.add("Settings")

        self.dashboard_tab = self.tabview.tab("Dashboard")
        self.dashboard_frame = ctk.CTkFrame(self.dashboard_tab)
        self.dashboard_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.stream_widget = StreamWidget(self.dashboard_frame)
        self.stream_widget.grid(row=0, column=1, padx=(10, 10), pady=(10, 10))

        self.controls_frame = ctk.CTkFrame(self.dashboard_frame)
        self.controls_frame.grid(row=1, column=1, pady=(10, 10))

        self.buttons_frame = ctk.CTkFrame(self.dashboard_frame, width=300)
        self.buttons_frame.grid(row=0, column=0, rowspan=2, pady=(10, 10))

        self.telemetrics_frame = ctk.CTkFrame(self.dashboard_frame, width=300)
        self.telemetrics_frame.grid(row=0, column=2, rowspan=2 ,pady=(10, 10))

        # Controls Panel
        self.label_up = ctk.CTkLabel(self.controls_frame, text="W", width=80, height=80, anchor="center")
        self.label_down = ctk.CTkLabel(self.controls_frame, text="S", width=80, height=80, anchor="center")
        self.label_left = ctk.CTkLabel(self.controls_frame, text="A", width=80, height=80, anchor="center")
        self.label_right = ctk.CTkLabel(self.controls_frame, text="D", width=80, height=80, anchor="center")
        self.label_up.grid(row=0, column=1, padx=5, pady=5)
        self.label_left.grid(row=2, column=0, padx=5, pady=5)
        self.label_down.grid(row=2, column=1, padx=5, pady=5)
        self.label_right.grid(row=2, column=2, padx=5, pady=5)
        
        self.label_speed = ctk.CTkLabel(self.controls_frame, text="Sent Speed: 0 m/s")
        self.label_yaw = ctk.CTkLabel(self.controls_frame, text="Sent Yaw Angle: 0 deg")
        self.label_gear = ctk.CTkLabel(self.controls_frame, text="Gear: 1")
        self.label_speed.grid(row=0, column=4, columnspan=4, padx=35, pady=5)
        self.label_yaw.grid(row=1, column=4, columnspan=4, padx=35, pady=5)
        self.label_gear.grid(row=2, column=4, columnspan=4, padx=35, pady=5)

        # Telemetrics Panel
        self.label_pitch = ctk.CTkLabel(self.telemetrics_frame, text="θ:")
        self.label_x = ctk.CTkLabel(self.telemetrics_frame, text="X:")
        self.label_delta = ctk.CTkLabel(self.telemetrics_frame, text="δ:")
        self.label_e_pitch = ctk.CTkLabel(self.telemetrics_frame, text="θ Error:")
        self.label_e_x = ctk.CTkLabel(self.telemetrics_frame, text="X Error:")
        self.label_e_delta = ctk.CTkLabel(self.telemetrics_frame, text="δ Error:")
        self.label_x_d = ctk.CTkLabel(self.telemetrics_frame, text="X Desired:")
        self.label_delta_d = ctk.CTkLabel(self.telemetrics_frame, text="δ Desired:")
        self.label_lwheel_angle = ctk.CTkLabel(self.telemetrics_frame, text="L Wheel °:")
        self.label_rwheel_angle = ctk.CTkLabel(self.telemetrics_frame, text="R Wheel °:")
        self.label_lpwm = ctk.CTkLabel(self.telemetrics_frame, text="L PWM:")
        self.label_rpwm = ctk.CTkLabel(self.telemetrics_frame, text="R PWM:")
        self.label_cycp = ctk.CTkLabel(self.telemetrics_frame, text="Cycle Period:")
        self.label_cycf = ctk.CTkLabel(self.telemetrics_frame, text="Cycle Freq. :")

        self.value_pitch = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_x = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_delta = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_e_pitch = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_e_x = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_e_delta = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_x_d = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_delta_d = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_lwheel_angle = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_rwheel_angle = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_lpwm = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_rpwm = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_cycp = ctk.CTkLabel(self.telemetrics_frame, text="0")
        self.value_cycf = ctk.CTkLabel(self.telemetrics_frame, text="0")

        self.label_pitch.grid(  row=1, column=1, columnspan=2, padx=35, pady=5)
        self.label_x.grid(      row=2, column=1, columnspan=2, padx=35, pady=5)
        self.label_delta.grid(  row=3, column=1, columnspan=2, padx=35, pady=5)
        self.label_e_pitch.grid(row=4, column=1, columnspan=2, padx=35, pady=5)
        self.label_e_x.grid(    row=5, column=1, columnspan=2, padx=35, pady=5)
        self.label_e_delta.grid(row=6, column=1, columnspan=2, padx=35, pady=5)
        self.label_x_d.grid(    row=7, column=1, columnspan=2, padx=35, pady=5)
        self.label_delta_d.grid(row=8, column=1, columnspan=2, padx=35, pady=5)
        self.label_lwheel_angle.grid(row=8, column=1, columnspan=2, padx=35, pady=5)
        self.label_rwheel_angle.grid(row=9, column=1, columnspan=2, padx=35, pady=5)
        self.label_lpwm.grid(row=10, column=1, columnspan=2, padx=35, pady=5)
        self.label_rpwm.grid(row=11, column=1, columnspan=2, padx=35, pady=5)
        self.label_cycp.grid(row=12, column=1, columnspan=2, padx=35, pady=5)
        self.label_cycf.grid(row=13, column=1, columnspan=2, padx=35, pady=5)

        self.value_pitch.grid(  row=1, column=3, columnspan=2, padx=35, pady=5)
        self.value_x.grid(      row=2, column=3, columnspan=2, padx=35, pady=5)
        self.value_delta.grid(  row=3, column=3, columnspan=2, padx=35, pady=5)
        self.value_e_pitch.grid(row=4, column=3, columnspan=2, padx=35, pady=5)
        self.value_e_x.grid(    row=5, column=3, columnspan=2, padx=35, pady=5)
        self.value_e_delta.grid(row=6, column=3, columnspan=2, padx=35, pady=5)
        self.value_x_d.grid(    row=7, column=3, columnspan=2, padx=35, pady=5)
        self.value_delta_d.grid(row=8, column=3, columnspan=2, padx=35, pady=5)
        self.value_lwheel_angle.grid(row=8, column=3, columnspan=2, padx=35, pady=5)
        self.value_rwheel_angle.grid(row=9, column=3, columnspan=2, padx=35, pady=5)
        self.value_lpwm.grid(row=10, column=3, columnspan=2, padx=35, pady=5)
        self.value_rpwm.grid(row=11, column=3, columnspan=2, padx=35, pady=5)
        self.value_cycp.grid(row=12, column=3, columnspan=2, padx=35, pady=5)
        self.value_cycf.grid(row=13, column=3, columnspan=2, padx=35, pady=5)

        # Buttons Panel
        self.toggle_motors = ctk.CTkButton(self.buttons_frame, text="Toggle Motor ON", command=self._toggle_motors_clicked)
        self.toggle_motors.grid(row=1, column=1, padx=5, pady=20)

        self.bt_button = ctk.CTkButton(self.buttons_frame, text="Connect Bluetooth", command=self._bt_button_clicked)
        self.bt_button.grid(row=7, column=1, padx=5, pady=20)

        # SETTINGS TAB
        self.settings_tab = self.tabview.tab("Settings")
        self.settings_frame = ctk.CTkFrame(self.settings_tab)
        self.settings_frame.pack(fill="both", expand=True, padx=10, pady=10)
        self.param_entries = []
        self.param_titles = [
            "θ Kp: ", "θ Ki: ", "θ Kd: ", "θ  N: ",
            "X Kp: ", "X Ki: ", "X Kd: ", "X  N: ",
            "δ Kp: ", "δ Ki: ", "δ Kd: ", "δ  N: ",]
        
        rownum = 0
        for i in range(NUM_PARAMS):
            if (i+1) % 4 != 0:
                param_title = ctk.CTkLabel(self.settings_frame, text=self.param_titles[i], width=80, height=5, anchor="w")
                entry = ctk.CTkEntry(self.settings_frame, width=200)
                param_title.grid(row=rownum, column=0, padx=10, pady=5)
                entry.grid(row=rownum, column=1, padx=10, pady=5)
                self.param_entries.append(entry)
                rownum += 1
            else:
                param_title = ctk.CTkLabel(self.settings_frame, text=self.param_titles[i], width=80, height=5, anchor="w")
                entry = ctk.CTkEntry(self.settings_frame, width=200)
                param_title.grid(row=rownum-2, column=2, padx=10, pady=5)
                entry.grid(row=rownum-2, column=3, padx=10, pady=5)
                self.param_entries.append(entry)
                

        self.param_button = ctk.CTkButton(self.settings_frame, text="Set Parameters", command=self._param_button_clicked)
        self.param_button.grid(row=NUM_PARAMS, column=0, pady=10)

        self.on_arrow_key_pressed = None
        self.on_arrow_key_released = None
        self.on_shift_key_pressed = None
        self.on_shift_key_released = None
        self.on_toggle_motors = None
        self.on_params_changed = None
        self.on_bluetooth_toggle_clicked = None

        # Bind key 
        self.bind("<KeyPress>", self.key_press_event)
        self.bind("<KeyRelease>", self.key_release_event)
        self.focus_set()
   
        def clear_focus(event):
            if event.widget.__class__.__name__ in ["CTkEntry", "Entry"]:
                return
            self.focus_set()

        self.bind_all("<Button-1>", clear_focus)

    def _toggle_motors_clicked(self):
        if self.on_toggle_motors:
            self.on_toggle_motors()

    def _param_button_clicked(self):
        if self.on_params_changed:
            self.on_params_changed()

    def _bt_button_clicked(self):
        if self.on_bluetooth_toggle_clicked:
            self.on_bluetooth_toggle_clicked()

    def key_press_event(self, event):
        key = event.keysym.lower()
        if key in ["w", "a", "s", "d"]:
            if key not in self.pressed_keys:
                self.pressed_keys.add(key)
                if self.on_arrow_key_pressed:
                    self.on_arrow_key_pressed(key)
            self.update_arrow_display()

        if key in ["j", "k"]:
            if key not in self.pressed_shift:
                self.pressed_shift.add(key)
                if self.on_shift_key_pressed:
                    self.on_shift_key_pressed(key)            

    def key_release_event(self, event):
        key = event.keysym.lower()
        if key in ["w", "a", "s", "d"]:
            if key in self.pressed_keys:
                self.pressed_keys.remove(key)
                if self.on_arrow_key_released:
                    self.on_arrow_key_released(key)
            self.update_arrow_display()
        
        if key in ["j", "k"]:
            if key in self.pressed_shift:
                self.pressed_shift.remove(key)
                if self.on_shift_key_released:
                    self.on_shift_key_released(key)

    def update_arrow_display(self):
        pressed_style = {"fg_color": "black", "text_color": "white"}
        unpressed_style = {"fg_color": "transparent", "text_color": "white"}
        self.label_up.configure(**(pressed_style if "w" in self.pressed_keys else unpressed_style))
        self.label_down.configure(**(pressed_style if "s" in self.pressed_keys else unpressed_style))
        self.label_left.configure(**(pressed_style if "a" in self.pressed_keys else unpressed_style))
        self.label_right.configure(**(pressed_style if "d" in self.pressed_keys else unpressed_style))

    def update_display(self, model: DashboardModel):
        self.label_speed.configure(text=f"Sent Speed: {model.speed} m/s")
        self.label_yaw.configure(text=f"Sent Yaw: {(model.yaw*180.0/math.pi):.1f} deg")
        self.label_gear.configure(text=f"Gear: {model.gear}")

        self.value_pitch.configure(text=f"{model.telemetry[0]:.4f}")
        self.value_x.configure(text=f"{model.telemetry[1]:.4f}")
        self.value_delta.configure(text=f"{model.telemetry[2]:.4f}")
        self.value_e_pitch.configure(text=f"{model.telemetry[3]:.4f}")
        self.value_e_x.configure(text=f"{model.telemetry[4]:.4f}")
        self.value_e_delta.configure(text=f"{model.telemetry[5]:.4f}")
        self.value_x_d.configure(text=f"{model.telemetry[6]:.4f}")
        self.value_delta_d.configure(text=f"{model.telemetry[7]:.4f}")
        self.value_lwheel_angle.configure(text=f"{model.telemetry[8]:.4f}")
        self.value_rwheel_angle.configure(text=f"{model.telemetry[9]:.4f}")
        self.value_lpwm.configure(text=f"{model.telemetry[10]:.4f}")
        self.value_rpwm.configure(text=f"{model.telemetry[11]:.4f}")
        self.value_cycp.configure(text=f"{model.telemetry[12]:.4f}")
        self.value_cycf.configure(text=f"{np.float32(1.0)/np.float32(model.telemetry[12]):.4f}")

        if model.bluetooth_connected:
            if model.control_state == 0:
                self.toggle_motors.configure(text="Toggle Motor ON", state="enabled")
            else:
                self.toggle_motors.configure(text="Toggle Motor OFF", state="enabled")
        else:
            model.control_state = 0 
            self.toggle_motors.configure(text="Toggle Motor ON", state="disabled")

    def get_param_values(self):
        paramvals = []
        for entry in self.param_entries:
            try:
                val = float(entry.get())
            except ValueError:
                val = 0.0
            paramvals.append(val)
        return paramvals

    def update_textboxes(self, model: DashboardModel):
        params = model.params
        for i in range(min(len(params), len(self.param_entries))):
            self.param_entries[i].delete(0, "end")
            self.param_entries[i].insert(0, str(params[i]))


class DashboardController:
    def __init__(self, model: DashboardModel, view: DashboardView, dashboard):
        self.model : DashboardModel = model
        self.view : DashboardView = view
        self.dashboard = dashboard
        self.pressed_keys = set()
        self.pressed_shift = set()

        # view call backs
        self.view.on_arrow_key_pressed = self.on_arrow_key_pressed
        self.view.on_arrow_key_released = self.on_arrow_key_released
        self.view.on_shift_key_pressed = self.on_shift_key_pressed
        self.view.on_shift_key_released = self.on_shift_key_released
        self.view.on_toggle_motors = self.on_toggle_motors
        self.view.on_params_changed = self.on_set_param_clicked
        self.view.on_bluetooth_toggle_clicked = self.on_bluetooth_toggle_clicked

        # update view upon model change
        self.model.add_observer(self.update_view)
        self.view.update_textboxes(self.model)

    def on_arrow_key_pressed(self, direction: str):
        self.pressed_keys.add(direction)
        self.process_input()

    def on_arrow_key_released(self, direction: str):
        if direction in self.pressed_keys:
            self.pressed_keys.remove(direction)
        self.process_input()

    def on_shift_key_pressed(self, key: str):
        self.pressed_shift.add(key)
        self.process_shift()
        self.update_view()
    
    def on_shift_key_released(self, key: str):
        if key in self.pressed_shift:
            self.pressed_shift.remove(key)
        self.process_shift()
        self.update_view()

    def process_input(self):
        f = 1 if "w" in self.pressed_keys else 0
        b = 1 if "s" in self.pressed_keys else 0
        l = 1 if "a" in self.pressed_keys else 0
        r = 1 if "d" in self.pressed_keys else 0
        gear = self.model.gear
        speed_scalar = gear / 10.0 + 0.15
        yaw_scalar = max(4.0/gear , 1) 
        speed = (f - b) * speed_scalar
        yaw = math.atan2(r - l, 1) * yaw_scalar
        self.model.speed = speed
        self.model.yaw = yaw
        self.dashboard.send_bluetooth()
    
    def process_shift(self):
        shift_up = 1 if "k" in self.pressed_shift else 0
        shift_down = 1 if "j" in self.pressed_shift else 0

        if shift_up:
            if self.model.gear < MAXGEAR:
                self.model.gear += 1
                self.model.energy *= 0.7

        if shift_down:
            if self.model.gear > 1:
                self.model.gear -= 1
                self.model.energy *= 1.2
           
        
    def on_toggle_motors(self):
        self.model.control_state = 1 if self.model.control_state == 0 else 0
        self.dashboard.send_bluetooth()

    def on_set_param_clicked(self):
        paramvals = self.view.get_param_values()
        self.model.params = paramvals
        try:
            with open("appcache.txt", "w") as f:
                for val in paramvals:
                    f.write(str(val) + "\n")
        except Exception as e:
            print("Error writing to appcache.txt:", e)
        self.dashboard.send_bluetooth()

    def on_bluetooth_toggle_clicked(self):
        self.dashboard.toggle_bluetooth()

    def update_view(self):
        self.view.update_display(self.model)


class Dashboard:
    def __init__(self):
        self.model = DashboardModel()
        try:
            if os.path.exists("appcache.txt"):
                with open("appcache.txt", "r") as f:
                    vals = [float(line.strip()) for line in f.readlines()]
                    self.model.params = vals
            else:
                self.model.params = [0.0] * NUM_PARAMS
        except Exception as e:
            self.model.params = [0.0] * NUM_PARAMS

        self.last_bt_send = 0
        self.view = DashboardView()
        self.controller = DashboardController(self.model, self.view, self)
        self.bluetooth_proc = None
        self.bt_to_queue = None
        self.bt_from_queue = None
        self.view.toggle_motors.configure(state="disabled")
        self.view.protocol("WM_DELETE_WINDOW", self.on_close)

        self.curr_time = time.time()
        self.prev_time = self.curr_time
        self.shift_up_last_pressed = time.time()
        self.shift_down_last_pressed = time.time()
        self.gas_last_pressed = time.time()
        self.shift_up_depressed = False
        self.shift_down_depressed = False
        self.gas_depressed = False
        
        from steering import SteeringThread
        self.steering_wheel_thread = SteeringThread(
            input_callback=self.external_input_callback,
            error_callback=self.external_error_callback
        )
        self.steering_wheel_thread.start()

        from audio import EngineAudio
        self.engine = EngineAudio(initial_rpm=1200.0, initial_gear=1)

    def external_input_callback(self, speed, yaw, shift_up, shift_down):
        self.view.after(0, self.process_external_input, speed, yaw, shift_up, shift_down)

    def process_external_input(self, speed, yaw, shift_up, shift_down):
        if not self.view.pressed_keys:

            gear = self.model.gear
            speed_scalar = gear / 10.0 + 0.15
            speed = speed * speed_scalar
            
            self.model.speed = speed
            self.model.yaw = yaw

            if shift_up:
                if (time.time() - self.shift_up_last_pressed) > 0.2:
                    if self.model.gear < MAXGEAR:
                        self.model.gear += 1
                        self.model.energy *= 0.7
                    self.shift_up_last_pressed = time.time()

            if shift_down:
                if (time.time() - self.shift_down_last_pressed) > 0.2:
                    if self.model.gear > 1:
                        self.model.gear -= 1
                        self.model.energy *= 1.2
                    self.shift_down_last_pressed = time.time()

            self.send_bluetooth()
            self.update_audio()

    def external_error_callback(self, message):
        print("[INFO] External device error:", message)


    def toggle_bluetooth(self):
        if self.bluetooth_proc is None:
            self.bt_to_queue = Queue()
            self.bt_from_queue = Queue()
            self.bluetooth_proc = Process(
                target=main_bluetooth_process,
                args=("ROBOT_C4", self.bt_to_queue, self.bt_from_queue)
            )

            self.bluetooth_proc.start()
            self.view.bt_button.configure(text="Disconnect Bluetooth")
            self.view.toggle_motors.configure(state="enabled")
            self.model.bluetooth_connected = True
            self.poll_bluetooth()  
            self.send_bluetooth()
        else:
            self.bt_to_queue.put({"command": "stop"})
            self.bluetooth_proc.join()
            self.bluetooth_proc = None
            self.view.bt_button.configure(text="Connect Bluetooth")
            self.view.toggle_motors.configure(state="disabled")
            self.model.bluetooth_connected = False

    def update_audio(self):
        self.engine.update_parameters(self.model.energy, self.model.gear)

    def send_bluetooth(self):
        if self.bt_to_queue is not None:
            cmd = {
                "speed": self.model.speed,
                "yaw": self.model.yaw,
                "control_state": self.model.control_state,
                "params": self.model.params
            }
            self.bt_to_queue.put(cmd)
            self.update_audio()
    
    def poll_bluetooth(self):
        if self.bt_from_queue is not None:
            while True:
                try:
                    message = self.bt_from_queue.get_nowait()
                    if isinstance(message, dict) and "status" in message:
                        if message["status"] == "connected":
                            self.model.bluetooth_connected = True
                        elif message["status"] == "disconnected":
                            self.model.bluetooth_connected = False
                            self.model.control_state = 0
                            self.view.update_display(self.model)
                    else:
                        self.model.telemetry = message
                except queue.Empty:
                    break
        # 10 ms 
        self.view.after(10, self.poll_bluetooth)

    def on_close(self):
        self.model.bluetooth_connected = False
        if self.bluetooth_proc is not None:
            self.bt_to_queue.put({"command": "stop"})
            self.bluetooth_proc.join()
            self.bluetooth_proc = None
        self.steering_wheel_thread.stop()
        self.engine.stop_streaming() 
        self.engine.server.stop()
        self.engine.server.shutdown()
        self.view.destroy()


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
    def control_state(self):
        return self.model.control_state

    @control_state.setter
    def control_state(self, value):
        self.model.control_state = value

    @property
    def telemetry(self):
        return self.model.telemetry
    
    @telemetry.setter
    def telemetry(self, value):
        self.model.telemetry = value

    @property
    def params(self):
        return self.model.params

    @params.setter
    def params(self, value):
        self.model.params = value

    def exec_(self):
        self.view.mainloop()

if __name__ == "__main__":
    dashboard = Dashboard()
    dashboard.exec_()
