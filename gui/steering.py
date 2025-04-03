#!/usr/bin/env python3
import threading
import time
import serial
import serial.tools.list_ports
from math import pi

class SteeringThread(threading.Thread):
    def __init__(self, input_callback=None, error_callback=None):
        super().__init__()
        self.input_callback = input_callback
        self.error_callback = error_callback
        self._running = True
        self.daemon = True
        self.timeout = 2.0
        self.comport = None
        self.find_device()

    def find_device(self):
        comports = [
            p.device
            for p in serial.tools.list_ports.comports()
            if 'Arduino Uno' in p.description
        ]
        if comports:
            try:
                self.comport = serial.Serial(comports[0], baudrate=9600, timeout=1)
            except Exception as e:
                self.comport = None
                if self.error_callback:
                    self.error_callback("Error opening COM port: " + str(e))
        else:
            self.comport = None

    def run(self):
        while self._running:
            if self.comport is None:
                self.find_device()
                time.sleep(1)
                continue
            try:
                if self.comport.in_waiting > 0:
                    data = self.comport.readline().decode('utf-8').strip()
                    parts = data.split(',')
                    if len(parts) == 2 and len(parts[0]) >= 2:
                        button_states = parts[0]
                        
                        shift_up = 0 if button_states[3] == '1' else 1
                        shift_down = 0 if button_states[2] == '1' else 1
                        fwd = 0 if button_states[1] == '1' else 1
                        bwd = 0 if button_states[0] == '1' else 1
                        
                        speed = (fwd - bwd) 
                        try:
                            steering_angle = -1.0 * float(parts[1])
                        except ValueError:
                            steering_angle = 0.0

                        if -10.0 < steering_angle < 10.0:
                            steering_angle = 0.0
                        elif steering_angle <= -10.0:
                            steering_angle += 10.0
                        else:
                            steering_angle -= 10.0

                        steering_angle *= 0.9

                        yaw = steering_angle * pi/180

                        if self.input_callback:
                            self.input_callback(speed, yaw, shift_up, shift_down)
                else:
                    time.sleep(0.05)
            except Exception as e:
                if self.error_callback:
                    self.error_callback("COMPORT Connection Error: " + str(e))
                self.comport = None
                time.sleep(1)

    def stop(self):
        self._running = False
        if self.comport:
            try:
                self.comport.close()
            except Exception:
                pass
