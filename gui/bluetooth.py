#!/usr/bin/env python3
import asyncio
from bleak import BleakClient, BleakScanner
import threading
import struct

NUM_PARAMS = 9

class Bluetooth:
    def __init__(self, device_name, dashboard):
        self.device_name = device_name
        self.dashboard = dashboard
        self.service_uuid = "180A"
        self.uuid = {
            "pitch":    "13012F01-F8C3-4F4A-A8F4-15CD926DA146",
            "control":  "13012F06-F8C3-4F4A-A8F4-15CD926DA146",
            "speed":    "13012F02-F8C3-4F4A-A8F4-15CD926DA146",
            "yaw":      "13012F03-F8C3-4F4A-A8F4-15CD926DA146",
            "motor_left":   "13012F04-F8C3-4F4A-A8F4-15CD926DA146",
            "motor_right":  "13012F05-F8C3-4F4A-A8F4-15CD926DA146",
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
                try:
                    yaw = self.dashboard.yaw
                    speed = self.dashboard.speed
                    motor_left = self.dashboard.motor_left
                    motor_right = self.dashboard.motor_right
                    control_mode = self.dashboard.control_state
                    params = self.dashboard.params
                except Exception as e:
                    print("[ERROR] Bluetooth: Error reading dashboard values:", e)
                    continue
                
                dout_bytes = bytearray()
                for param in params:
                    dout_bytes += bytearray(str(param), encoding="utf-8")

                await self.sendCommand(client, speed, self.uuid["speed"])
                await self.sendCommand(client, yaw, self.uuid["yaw"])
                await self.sendCommand(client, control_mode, self.uuid["control"])
                await self.sendCommand(client, dout_bytes, self.uuid["dout"])
                
                await asyncio.sleep(0.01)

    async def sendCommand(self, client, val, uuid):
        val_str = str(val)
        val_bytes = bytearray(val_str, encoding="utf-8")
        await client.write_gatt_char(uuid, val_bytes, response=True)

    def start(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.run())

    def stop(self):
        self._running = False
