#!/usr/bin/env python3
import asyncio
from bleak import BleakClient, BleakScanner
import threading
import struct
from config import *
from multiprocessing import Queue


class Bluetooth:
    def __init__(self, device_name, to_queue : Queue, from_queue : Queue):
        self.device_name = device_name
        self.to_queue = to_queue      
        self.from_queue = from_queue 
        self.service_uuid = "180A"
        self.uuid = {
            "din":    "13012F01-F8C3-4F4A-A8F4-15CD926DA146",
            "dout" : "13012F02-F8C3-4F4A-A8F4-15CD926DA146",
        }
        self.loop = asyncio.new_event_loop()
        self._running = True  
        self.speed = 0.0
        self.yaw = 0.0
        self.control_state = 0
        self.params = [0.0] * NUM_PARAMS

    async def run(self):
        device = await BleakScanner.find_device_by_name(self.device_name)
        if not device:
            print(f"[ERROR] Bluetooth: Device not found")
            return
        async with BleakClient(device) as client:
            print(f"[INFO] Bluetooth: Successfully connected to {device}")
            
            while self._running:
                # read from arduino and write to gui
                try:
                    # din_bytes = await client.read_gatt_char(self.uuid["din"])
                    din_bytes = await asyncio.wait_for(client.read_gatt_char(self.uuid["din"]), timeout=1.0)
                    din_data = struct.unpack_from('<' + 'f' * NUM_DIN, din_bytes)
                except Exception as e:
                    print("[ERROR] Bluetooth: Error recieving values from arduino:", e)
                    din_data = [0.0]*NUM_DIN
                    
                if len(din_data) == NUM_DIN:
                    try:
                        self.from_queue.put(list(din_data)) # from bluetooth to gui
                        # self.dashboard.telemetry = list(din_data)[:NUM_DIN].copy()
                    except Exception as e:
                        print("[ERROR] Bluetooth: Error writing values to dashboard:", e)
                        continue
                
                # read from gui and write to arduino
                try:
                    while True:
                        cmd = self.to_queue.get_nowait()
                        if cmd.get("command") == "stop":
                            self._running = False
                            print("Stopped")
                            break
                        if "speed" in cmd:
                            self.speed = cmd["speed"]
                        if "yaw" in cmd:
                            self.yaw = cmd["yaw"]
                        if "control_state" in cmd:
                            self.control_state = cmd["control_state"]
                        if "params" in cmd:
                            self.params = cmd["params"]
                        # self.to_queue.task_done()
                except Exception:
                    pass

                if not self._running:
                    break

                try:
                    param_bytes = struct.pack('<' + 'f' * NUM_PARAMS, *self.params)
                    movement = [self.speed, self.yaw]
                    movement_bytes = struct.pack('<' + 'f' * 2, *movement)
                    ctrl_bytes = struct.pack('<' + 'i', self.control_state)
                    dout_bytes = ctrl_bytes + movement_bytes + param_bytes
                    await client.write_gatt_char(self.uuid["dout"], dout_bytes, response=True)  
                except Exception as e:
                    print("[ERROR] Bluetooth: Error writing values to arduino:", e)

                await asyncio.sleep(0.001)

    def start(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.run())

    # def stop(self):
    #     self._running = False

def main_bluetooth_process(device_name, to_queue, from_queue):
    bt = Bluetooth(device_name, to_queue, from_queue)
    bt.start()