#!/usr/bin/env python3
import asyncio
from bleak import BleakClient, BleakScanner
import threading
import struct
from config import *


class Bluetooth:
    def __init__(self, device_name, dashboard):
        self.device_name = device_name
        self.dashboard = dashboard
        self.service_uuid = "180A"
        self.uuid = {
            "din":    "13012F01-F8C3-4F4A-A8F4-15CD926DA146",
            "dout" : "13012F02-F8C3-4F4A-A8F4-15CD926DA146",
        }
        self.loop = asyncio.new_event_loop()
        self._running = True  

    async def run(self):
        device = await BleakScanner.find_device_by_name(self.device_name)
        if not device:
            print(f"[ERROR] Bluetooth: Device not found")
            return
        async with BleakClient(device) as client:
            print(f"[INFO] Bluetooth: Successfully connected to {device}")
            
            while self._running:
                try:
                    din_bytes = await client.read_gatt_char(self.uuid["din"])
                    # din_bytes = await asyncio.wait_for(client.read_gatt_char(self.uuid["din"]), timeout=1.0)
                    din_data = struct.unpack_from('<' + 'f' * NUM_DIN, din_bytes)
                except Exception as e:
                    print("[ERROR] Bluetooth: Error recieving values from arduino:", e)
                    din_data = [0.0]*NUM_DIN
                    
                if len(din_data) == NUM_DIN:
                    try:
                        self.dashboard.telemetry = list(din_data)[:NUM_DIN].copy()
                    except Exception as e:
                        print("[ERROR] Bluetooth: Error writing values to dashboard:", e)
                        continue

                try:
                    yaw = self.dashboard.yaw
                    speed = self.dashboard.speed
                    control_state = self.dashboard.control_state
                    params = self.dashboard.params
                except Exception as e:
                    print("[ERROR] Bluetooth: Error reading dashboard values:", e)
                    continue

                param_bytes = struct.pack('<' + 'f' * NUM_PARAMS, *params)
                movement = [speed, yaw]
                movement_bytes = struct.pack('<' + 'f' * 2, *movement)
                ctrl_bytes = struct.pack('<' + 'i', control_state)
                dout_bytes = ctrl_bytes + movement_bytes + param_bytes

                await client.write_gatt_char(self.uuid["dout"], dout_bytes, response=True)
                
                await asyncio.sleep(0.001)

    def start(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.run())

    def stop(self):
        self._running = False
