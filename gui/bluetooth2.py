#!/usr/bin/env python3
import asyncio
from bleak import BleakClient, BleakScanner
import threading
from dashboard2 import Dashboard


class Bluetooth:
    def __init__(self, device_name, dashboard: Dashboard):
        self.device_name = device_name
        self.dashboard = dashboard
        self.uuid = {
            "pitch"         : "13012F01-F8C3-4F4A-A8F4-15CD926DA146",
            "control"       : "13012F06-F8C3-4F4A-A8F4-15CD926DA146",
            "speed"         : "13012F02-F8C3-4F4A-A8F4-15CD926DA146",
            "yaw"           : "13012F03-F8C3-4F4A-A8F4-15CD926DA146",
            "motor_left"    : "13012F04-F8C3-4F4A-A8F4-15CD926DA146",
            "motor_right"   : "13012F05-F8C3-4F4A-A8F4-15CD926DA146",
            1            : "13012F07-F8C3-4F4A-A8F4-15CD926DA146",
            2            : "13012F08-F8C3-4F4A-A8F4-15CD926DA146",
            3            : "13012F09-F8C3-4F4A-A8F4-15CD926DA146",
            4            : "13012F10-F8C3-4F4A-A8F4-15CD926DA146",
            5            : "13012F11-F8C3-4F4A-A8F4-15CD926DA146",
            6            : "13012F12-F8C3-4F4A-A8F4-15CD926DA146",
            7            : "13012F13-F8C3-4F4A-A8F4-15CD926DA146"
        }
        self.loop = asyncio.new_event_loop()

    async def run(self):
        device = await BleakScanner.find_device_by_name(self.device_name)
        if not device:
            print(f"[ERROR] {__class__} Device not found")
            return
        print(f"[INFO] Bluetooth successfully connected to: {device}")
        async with BleakClient(device) as client:
            while True:
                pitch_data = await client.read_gatt_char(self.pitch_angle_uuid)
                try:
                    yaw = self.dashboard.yaw
                    speed = self.dashboard.speed
                    motor_left = self.dashboard.motor_left
                    motor_right = self.dashboard.motor_right
                    control_mode = self.dashboard.control_state
                    params = self.dashboard.params
                except Exception as e:
                    print(f"[ERROR] {__class__} Error reading dashboard values:", e)
                    continue

                try:
                    pitch_value = float(pitch_data.decode('utf-8'))
                    self.dashboard.pitch = pitch_value
                except Exception as e:
                    print(f"[ERROR] {__class__} Error updating pitch:", e)

                await self.sendCommand(client, speed, self.uuid["speed"])
                await self.sendCommand(client, yaw, self.uuid["yaw"])
                await self.sendCommand(client, control_mode, self.uuid["control"])
                # await self.sendCommand(client, motor_left, self.uuid["motor_left"])
                # await self.sendCommand(client, motor_right, self.uuid["motor_right"])

                for i in range(NUM_PARAMS):
                    await self.sendCommand(client, params[i], self.uuid[i+1])
                await asyncio.sleep(0.01)

    async def sendCommand(self, client, val, uuid):
        val_str = str(val)
        val_bytes = bytearray(val_str, encoding="utf-8")
        await client.write_gatt_char(uuid, val_bytes, response=True)

    def start(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.run())


if __name__ == "__main__":
    DEVICE_NAME = "ROBOT_C4"
    NUM_PARAMS = 7
    dash = Dashboard()

    bt = Bluetooth(DEVICE_NAME, dash)
    bt_thread = threading.Thread(target=bt.start, daemon=True)
    bt_thread.start()

    dash.exec_()
