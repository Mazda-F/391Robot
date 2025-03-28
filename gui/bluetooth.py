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
            "control":  "13012F06-F8C3-4F4A-A8F4-15CD926DA146",
            "movement":    "13012F02-F8C3-4F4A-A8F4-15CD926DA146",
            "dout" : "13012F07-F8C3-4F4A-A8F4-15CD926DA146",
        }
        self.loop = asyncio.new_event_loop()
        self._running = True  

    async def run(self):
        device = await BleakScanner.find_device_by_name(self.device_name)
        if not device:
            print(f"[ERROR] Bluetooth: Device not found")
            return
        print(f"[INFO] Bluetooth: Successfully connected to {device}")
        async with BleakClient(device) as client:
            while self._running:
                din_str = await client.read_gatt_char(self.uuid["din"])
                din_substrs = din_str.split(", ")
                din_data = []
                for substr in din_substrs:
                    din_data.append(substr.decode())
                    
                try:
                    self.dashboard.telemetry = din_data[:NUM_DIN]
                except Exception as e:
                    print("[ERROR] Bluetooth: Error writing values to dashboard", e)
                    continue

                try:
                    yaw = self.dashboard.yaw
                    speed = self.dashboard.speed
                    control_mode = self.dashboard.control_state
                    params = self.dashboard.params
                except Exception as e:
                    print("[ERROR] Bluetooth: Error reading dashboard values:", e)
                    continue
                
                dout_bytes = struct.pack('<' + 'f' * NUM_PARAMS, *params)
                movement = [yaw, speed]
                movement_bytes = struct.pack('<' + 'f' * 2, *movement)

                await self.sendCommand(client, control_mode, self.uuid["control"])
                await self.sendRawBytes(client, movement_bytes, self.uuid["movement"])
                await self.sendRawBytes(client, dout_bytes, self.uuid["dout"])
                
                await asyncio.sleep(0.01)

    async def sendRawBytes(self, client, bytes, uuid):
        await client.write_gatt_char(uuid, bytes, response=True)

    async def sendCommand(self, client, val, uuid):
        val_str = str(val)
        val_bytes = bytearray(val_str, encoding="utf-8")
        await client.write_gatt_char(uuid, val_bytes, response=True)

    def start(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.run())

    def stop(self):
        self._running = False
