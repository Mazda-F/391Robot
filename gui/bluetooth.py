#!/usr/bin/env python3
import asyncio
from bleak import BleakClient, BleakScanner
import threading
import struct
from config import *
from multiprocessing import Queue
import queue


class Bluetooth:
    def __init__(self, device_name, to_queue : Queue, from_queue : Queue):
        self.device_name = device_name
        self.to_queue = to_queue      
        self.from_queue = from_queue 
        self.service_uuid = "449f9707-8365-440d-94c0-25c7663b292f"
        self.uuid = {
            "din":    "b9a7479e-6475-4093-ae2a-6ee19eae177a",
            "dout" : "0d4e68bf-be73-4ddc-847c-ea40afaef5ef",
        }
        self.loop = asyncio.new_event_loop()
        self._running = True  
        self.speed = 0.0
        self.yaw = 0.0
        self.control_state = 0
        self.params = [0.0] * NUM_PARAMS

    async def run(self):
        while self._running:
            device = await BleakScanner.find_device_by_name(self.device_name)
            if not device:
                print(f"[ERROR] Bluetooth: Device not found")
                self.from_queue.put({"status": "disconnected"})
                await asyncio.sleep(1)
                continue
            
            try:
                async with BleakClient(device) as client:
                    print(f"[INFO] Bluetooth: Successfully connected to {device}")
                    self.from_queue.put({"status": "connected"})
                    while self._running:
                        # read from arduino and write to gui
                        try:
                            din_bytes = await client.read_gatt_char(self.uuid["din"])
                            din_data = struct.unpack_from('<' + 'f' * NUM_DIN, din_bytes)
                        except Exception as e:
                            print("[WARNING] Bluetooth: Failed recieving values from arduino:", e)
                            din_data = [0.0]*NUM_DIN
                            
                        if len(din_data) == NUM_DIN:
                            try:
                                self.from_queue.put(list(din_data)) # from bluetooth to gui
                            except Exception as e:
                                print("[WARNING] Bluetooth: Failed writing values to dashboard:", e)
                                continue
                        
                        # read from gui and write to arduino
                        latest_cmd = None
                        # drain the queue
                        while True:
                            try:
                                cmd = self.to_queue.get_nowait()
                                latest_cmd = cmd  # Always updates to the lateste cmd
                            except queue.Empty:
                                break

                        if latest_cmd:
                            if latest_cmd.get("command") == "stop":
                                self._running = False
                                print("Stopped")
                            else:
                                self.speed = latest_cmd.get("speed", self.speed)
                                self.yaw = latest_cmd.get("yaw", self.yaw)
                                self.control_state = latest_cmd.get("control_state", self.control_state)
                                self.params = latest_cmd.get("params", self.params)

                        if not self._running:
                            break
                        
                        # writing
                        try:
                            param_bytes = struct.pack('<' + 'f' * NUM_PARAMS, *self.params)
                            movement = [self.speed, self.yaw]
                            movement_bytes = struct.pack('<' + 'f' * 2, *movement)
                            ctrl_bytes = struct.pack('<' + 'i', self.control_state)
                            dout_bytes = ctrl_bytes + movement_bytes + param_bytes
                            await client.write_gatt_char(self.uuid["dout"], dout_bytes, response=True)  
                        except Exception as e:
                            print("[WARNING] Bluetooth: Error writing values to arduino:", e)
                            self.from_queue.put({"status": "disconnected"})
                            break

                        await asyncio.sleep(0.02)
            except Exception as e:
                print(f"[ERROR] Bluetooth: Connection Failed: {e}")
                self.from_queue.put({"status" : "disconnected"})
            
            if self._running:
                print({"[INFO] Bluetooth: Attempting auto-reconnection..."})
                await asyncio.sleep(1)

    def start(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.run())


def main_bluetooth_process(device_name, to_queue, from_queue):
    bt = Bluetooth(device_name, to_queue, from_queue)
    bt.start()