#!/usr/bin/env python3
import cv2
import numpy as np
import urllib.request
import threading
from PIL import Image, ImageTk
import customtkinter as ctk
import tkinter as tk
import time

class StreamThread(threading.Thread):
    def __init__(self, frame_callback, error_callback, stream_url="http://192.168.4.1:129/stream"):
        super().__init__()
        self.frame_callback = frame_callback
        self.error_callback = error_callback
        self.stream_url = stream_url
        self._running = True
        self.daemon = True
        self.last_frame_time = time.time()
        self.timeout = 2.0

    def run(self):
        try:
            with urllib.request.urlopen(self.stream_url) as stream:
                data = b""
                while self._running:
                    chunk = stream.read(1024)
                    if not chunk:
                        break
                    data += chunk
                    a = data.find(b'\xff\xd8')  # JPEG start
                    b = data.find(b'\xff\xd9')  # JPEG end
                    if (a != -1 and b != -1 and b > a) or len(data) > 8000:
                        jpg = data[a:b+2]
                        data = data[b+2:] if (b+2) < len(data) else b""

                        if time.time() - self.last_frame_time > self.timeout:
                            print("Stalled frame detected, resetting buffer")
                            data = b""
                            self.last_frame_time = time.time()
                            continue

                        img_array = np.frombuffer(jpg, dtype=np.uint8)
                        if img_array.size == 0:
                            continue
                        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                        if img is None:
                            continue
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        pil_image = Image.fromarray(img)
                        self.frame_callback(pil_image)
        except Exception as e:
            self.error_callback("Connection Error")

    def stop(self):
        self._running = False

class StreamWidget(ctk.CTkFrame):
    def __init__(self, parent, width=640, height=480):
        super().__init__(parent, width=width, height=height)
        self.width = width
        self.height = height
        self.grid_propagate(False)

        self.image_label = ctk.CTkLabel(self, text="Loading stream...")
        self.image_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

        self.retry_button = ctk.CTkButton(self, text="Retry", command=self.retry_connection)
        self.retry_button.place(relx=0.5, rely=0.9, anchor=tk.CENTER)
        self.retry_button.configure(state="disabled")

        self.thread = None
        self.current_photo = None  # keep reference
        self.start_stream_thread()

    def start_stream_thread(self):
        self.thread = StreamThread(self.frame_received, self.handle_connection_error)
        self.thread.start()

    def frame_received(self, pil_image):
        self.after(0, self.update_image, pil_image)

    def update_image(self, pil_image):
        self.retry_button.configure(state="disabled")
        self.image_label.configure(text="")
        pil_image = pil_image.resize((self.width, self.height), Image.LANCZOS)
        # pil_image = pil_image.resize((self.width, self.height))
        self.current_photo = ImageTk.PhotoImage(pil_image)
        # self.current_photo = ctk.CTkImage(pil_image)
        self.image_label.configure(image=self.current_photo)

    def handle_connection_error(self, error_msg):
        self.after(0, self.show_error, error_msg)

    def show_error(self, error_msg):
        self.image_label.configure(text=error_msg, image=None)
        self.retry_button.configure(state="normal")

    def retry_connection(self):
        if self.thread and self.thread.is_alive():
            self.thread.stop()
            self.thread.join()
        self.retry_button.configure(state="disabled")
        self.image_label.configure(text="Retrying...", image=None)
        self.start_stream_thread()

    def destroy(self):
        if self.thread and self.thread.is_alive():
            self.thread.stop()
            self.thread.join()
        super().destroy()