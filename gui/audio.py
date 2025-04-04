#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt
import pyo
import numpy as np
import time
import threading
import serial
import serial.tools.list_ports
from math import pi
import socket
import sounddevice as sd


class EngineAudio:
    def __init__(self, initial_rpm=1200, initial_gear=1):
        self.server = pyo.Server().boot()
        self.server.start()
        self.rpm = initial_rpm
        self.gear = initial_gear
        self.prev_gear = initial_gear
        self.waves = []
        self._create_sound_objects()
       
        self.start_streaming()

    def _get_engine_pitch(self, rpm) -> float:
        RPM_CUTF = 8850.0
        RPM_DAMP = 18000.0
        rpm_atten = max(0, min(1.0, (RPM_CUTF - rpm + RPM_DAMP) / RPM_DAMP))
        return float(rpm / 25.0 * rpm_atten)

    def _create_sound_objects(self):
        self.engine_pitch = self._get_engine_pitch(self.rpm) 
        self.waves.append(pyo.Sine(freq=self.engine_pitch * 0.1, mul=0.9))
        self.waves.append(pyo.Sine(freq=self.engine_pitch * 0.5, mul=0.8))
        self.waves.append(pyo.Sine(freq=self.engine_pitch * 0.77, mul=0.8))
        self.waves.append(pyo.Sine(freq=self.engine_pitch * 0.9, mul=0.6))
        self.waves.append(pyo.Sine(freq=self.engine_pitch * 1.7, mul=0.3))
        self.waves.append(pyo.Sine(freq=self.engine_pitch * 2.4, mul=0.2))
        self.waves.append(pyo.Sine(freq=self.engine_pitch * 3.2, mul=0.15))
        self.waves.append(pyo.Sine(freq=self.engine_pitch * 3.5, mul=0.09))
        self.waves.append(pyo.Sine(freq=self.engine_pitch * 3.7, mul=0.05))

        self.noise = pyo.Noise(mul=0.2 * (7 - self.gear) / 6)
        self.noise_filt = pyo.ButBP(self.noise, freq=self.engine_pitch * 1.5, q=5)
        self.engine_raw = sum(self.waves) + self.noise_filt
        self.pulsation = pyo.Sine(freq=self.engine_pitch, mul=0.4, add=0.95)
        self.engine_sound = self.engine_raw * self.pulsation
        self.engine_sound.out()

    def update_parameters(self, energy, gear):
        self.gear = gear
        self.rpm = energy * 100 + 1200.0
        self.engine_pitch = self._get_engine_pitch(self.rpm)  
        for i, wave in enumerate(self.waves):
            wave.freq = self.engine_pitch * i * 0.5
        self.trec.stop()
        try:
            self.table.clear()
        except Exception:
            pass
        self.trec.play()

    def start_streaming(self):
        self.table = pyo.NewTable(length=0.042)
        self.trec = pyo.TableRec(self.engine_sound, table=self.table, fadetime=0.015).play()
        
        # Set up the TCP socket.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.TCP_IP = "192.168.4.1"
        self.TCP_PORT = 130
        try:
            self.sock.connect((self.TCP_IP, self.TCP_PORT))
        except Exception as e:
            print("Error connecting to audio TCP server:", e)
            return
        
        self.streaming = True
        self.last_index = 0
        self.last_sample_count = self.server.getCurrentTimeInSamples()
        self.sample_rate = self.server.getSamplingRate()  # e.g., 44100 Hz
        self.table_size = self.table.getSize()  
        self.dtype = 'int16'
        self.rate = 44100
        self.channels = 2
        
        # Accumulation buffer for fixed 1024-byte chunks.
        self.accum_buffer = bytearray()
        self.chunk_bytes = 1024  # Fixed chunk size.
        self.bytes_per_sample = 4 
        self.samples_per_chunk = self.chunk_bytes // self.bytes_per_sample
        
        self.stream_thread = threading.Thread(target=self.stream_audio)

        self.stream_thread.daemon = True
        self.stream_thread.start()

    def stream_audio(self):
        while self.streaming:
            current_sample_count = self.server.getCurrentTimeInSamples()
            delta_samples = current_sample_count - self.last_sample_count
            self.last_sample_count = current_sample_count
            
            new_index = (self.last_index + delta_samples) % self.table_size
            
            buffer_array = np.array(self.table.getBuffer())
            
            if new_index < self.last_index:
                new_samples = np.concatenate((buffer_array[self.last_index:], buffer_array[:new_index]))
            else:
                new_samples = buffer_array[self.last_index:new_index]
            self.last_index = new_index
            
            if new_samples.size > 0:
                new_samples = np.clip(new_samples, -1, 1)
                int_samples = (new_samples * 32767).astype(np.int16)
                new_bytes = int_samples.tobytes()
                self.accum_buffer.extend(new_bytes)
                

                while len(self.accum_buffer) >= self.chunk_bytes:
                    # chunk = self.accum_buffer[:self.chunk_bytes]
                #     try:
                #         self.sock.sendall(chunk)
                #     except Exception as e:
                #         print("TCP send error:", e)
                #         self.streaming = False
                #         break
                    self.accum_buffer = self.accum_buffer[self.chunk_bytes:]
        
            time.sleep(self.samples_per_chunk / self.sample_rate)

    def stop_streaming(self):
        self.streaming = False
        if hasattr(self, 'stream_thread'):
            self.stream_thread.join()
        if hasattr(self, 'sock'):
            self.sock.close()