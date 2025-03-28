#!/usr/bin/env python3
import customtkinter as ctk
import threading, math, os
from stream import StreamWidget
from config import *

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class DashboardModel:
    def __init__(self):
        self._control_state = 0
        self._speed = 0
        self._yaw = 0
        self._pitch = 0
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
        self.geometry("1180x720")
        self.pressed_keys = set()

        self.tabview = ctk.CTkTabview(self, width=1080, height=720)
        self.tabview.pack(fill="both", expand=True)
        self.tabview.add("Dashboard")
        self.tabview.add("Settings")

        self.dashboard_tab = self.tabview.tab("Dashboard")
        self.dashboard_frame = ctk.CTkFrame(self.dashboard_tab)
        self.dashboard_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.stream_widget = StreamWidget(self.dashboard_frame, width=780, height=400)
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
        self.label_left.grid(row=1, column=0, padx=5, pady=5)
        self.label_down.grid(row=1, column=1, padx=5, pady=5)
        self.label_right.grid(row=1, column=2, padx=5, pady=5)
        
        self.label_speed = ctk.CTkLabel(self.controls_frame, text="Sent Speed: 0 m/s")
        self.label_yaw = ctk.CTkLabel(self.controls_frame, text="Sent Yaw Angle: 0 deg")
        self.label_speed.grid(row=1, column=4, columnspan=4, padx=35, pady=5)
        self.label_yaw.grid(row=2, column=4, columnspan=4, padx=35, pady=5)

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

        # Buttons Panel
        self.toggle_motors = ctk.CTkButton(self.buttons_frame, text="Toggle Motor ON", command=self._toggle_motors_clicked)
        self.toggle_motors.grid(row=5, column=1, padx=5, pady=5)

        self.bt_button = ctk.CTkButton(self.buttons_frame, text="Connect Bluetooth", command=self._bt_button_clicked)
        self.bt_button.grid(row=6, column=1, padx=5, pady=5)

        # SETTINGS TAB
        self.settings_tab = self.tabview.tab("Settings")
        self.settings_frame = ctk.CTkFrame(self.settings_tab)
        self.settings_frame.pack(fill="both", expand=True, padx=10, pady=10)
        self.param_entries = []
        for i in range(NUM_PARAMS):
            entry = ctk.CTkEntry(self.settings_frame, width=200)
            entry.grid(row=i, column=0, padx=5, pady=5)
            self.param_entries.append(entry)
        self.param_button = ctk.CTkButton(self.settings_frame, text="Set Parameters", command=self._param_button_clicked)
        self.param_button.grid(row=NUM_PARAMS, column=0, pady=10)

       
        self.on_arrow_key_pressed = None
        self.on_arrow_key_released = None
        self.on_toggle_motors = None
        self.on_params_changed = None
        self.on_bluetooth_toggle_clicked = None

        # Bind key 
        self.bind("<KeyPress>", self.key_press_event)
        self.bind("<KeyRelease>", self.key_release_event)
        self.focus_set()

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

    def key_release_event(self, event):
        key = event.keysym.lower()
        if key in ["w", "a", "s", "d"]:
            if key in self.pressed_keys:
                self.pressed_keys.remove(key)
                if self.on_arrow_key_released:
                    self.on_arrow_key_released(key)
            self.update_arrow_display()

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

        if model.bluetooth_connected:
            if model.control_state == 0:
                self.toggle_motors.configure(text="Toggle Motor ON", state="enabled")
            else:
                self.toggle_motors.configure(text="Toggle Motor OFF", state="enabled")
        else:
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

        # view call backs
        self.view.on_arrow_key_pressed = self.on_arrow_key_pressed
        self.view.on_arrow_key_released = self.on_arrow_key_released
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

    def process_input(self):
        f = 1 if "w" in self.pressed_keys else 0
        b = 1 if "s" in self.pressed_keys else 0
        l = 1 if "a" in self.pressed_keys else 0
        r = 1 if "d" in self.pressed_keys else 0
        speed = (f - b) * 1.0
        yaw = math.atan2(r - l, 1)/2.0
        self.model.speed = speed
        self.model.yaw = yaw

    def on_toggle_motors(self):
        self.model.control_state = 1 if self.model.control_state == 0 else 0

    def on_set_param_clicked(self):
        paramvals = self.view.get_param_values()
        self.model.params = paramvals
        try:
            with open("appcache.txt", "w") as f:
                for val in paramvals:
                    f.write(str(val) + "\n")
        except Exception as e:
            print("Error writing to appcache.txt:", e)

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

        self.view = DashboardView()
        self.controller = DashboardController(self.model, self.view, self)
        self.bluetooth = None
        self.bluetooth_thread = None
        self.view.toggle_motors.configure(state="disabled")

        self.view.protocol("WM_DELETE_WINDOW", self.on_close)

    def toggle_bluetooth(self):
        from bluetooth import Bluetooth
        if self.bluetooth is None:
            self.bluetooth = Bluetooth("ROBOT_C4", self)
            self.bluetooth_thread = threading.Thread(target=self.bluetooth.start, daemon=True)
            self.bluetooth_thread.start()
            self.view.bt_button.configure(text="Disconnect Bluetooth")
            self.view.toggle_motors.configure(state="enabled")
            self.model.bluetooth_connected = True
        else:
            self.bluetooth.stop()
            self.bluetooth_thread.join()
            self.bluetooth = None
            self.view.bt_button.configure(text="Connect Bluetooth")
            self.view.toggle_motors.configure(state="disabled")
            self.model.bluetooth_connected = False

    def on_close(self):
        # auto stop bt
        self.model.bluetooth_connected = False
        if self.bluetooth is not None:
            self.bluetooth.stop()
            if self.bluetooth_thread is not None:
                self.bluetooth_thread.join()
            self.bluetooth = None
        # destroying main view stop the mjpeg streaming thread
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

    # @property
    # def pitch(self):
    #     return self.model.pitch

    # @pitch.setter
    # def pitch(self, value):
    #     self.model.pitch = value

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
