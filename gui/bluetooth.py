import asyncio
from bleak import BleakClient, BleakScanner
import random
import struct
import threading 
import queue
from dashboard import Dashboard
from multiprocessing.connection import Listener
import multiprocessing as mp


class Bluetooth:
    """
    Async Bluetooth connection class with the Arduino BLE:

    Use with asyncio

    Example:

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        bt = Bluetooth(event_queue, device_name="ROBOT_C4")
        asyncio.ensure_future(bt.run())
        loop.run_forever()

    """
    def __init__(self, queue : queue.Queue, device_name, lock : threading.Lock, writelock : threading.Lock, input : Dashboard):
        self.queue = queue
        self.lock = lock
        self.writelock = writelock
        self.input = input
        self.device_name = device_name
        self.service_uuid = "180A"
        self.pitch_angle_uuid = "13012F01-F8C3-4F4A-A8F4-15CD926DA146"
        self.control_uuid = "13012F06-F8C3-4F4A-A8F4-15CD926DA146"
        self.speed_uuid = "13012F02-F8C3-4F4A-A8F4-15CD926DA146"
        self.yaw_uuid = "13012F03-F8C3-4F4A-A8F4-15CD926DA146"

        self.kp_uuid = "13012F07-F8C3-4F4A-A8F4-15CD926DA146"
        self.ki_uuid = "13012F08-F8C3-4F4A-A8F4-15CD926DA146"
        self.kd_uuid = "13012F09-F8C3-4F4A-A8F4-15CD926DA146"
        self.event_loop = asyncio.get_event_loop()

    async def run(self):
        self.device = await BleakScanner.find_device_by_name(self.device_name)
        print(self.device)

        async with BleakClient(self.device) as client:
            print(client)
            # services = await client.get_services()
            # for service in services:
            #     print(service)
            #     for char in service.characteristics:
            #         print("  ", char, char.properties)
            while True:
                # pitch = await client.read_gatt_char(self.pitch_angle_uuid)
                
                try:
                    while not self.lock.acquire(blocking=False):
                        asyncio.sleep(0.001)
                        pass
                    yaw = self.input.yaw
                    speed = self.input.speed 
                    # motor_left = self.input.motor_left
                    # motor_right = self.input.motor_right
                    control_mode = self.input.control_state
                    Kp = self.input.Kp
                    Ki = self.input.Ki
                    Kd = self.input.Kd

                    self.lock.release()
                        
                    # with self.lock: 
                    #     yaw = self.input.yaw
                    #     speed = self.input.speed   
                except Exception:
                    pass

                try:
                    with self.writelock: 
                        # self.input.pitch = pitch.decode('utf-8')
                        pass
                except Exception:
                    pass
            

                await self.sendCommand(client, speed, self.speed_uuid)
                await self.sendCommand(client, yaw, self.yaw_uuid)
                await self.sendCommand(client, Kp, self.kp_uuid)
                await self.sendCommand(client, Ki, self.ki_uuid)
                await self.sendCommand(client, Kd, self.kd_uuid)
                await self.sendCommand(client, control_mode, self.control_uuid)

                await asyncio.sleep(0.001)

    async def sendCommand(self, client : BleakClient, val, uuid):
        val_str = str(val)
        val_bytes = bytearray(val_str, encoding="utf-8") 
        await client.write_gatt_char(uuid, val_bytes, response=True)


    def start(self):
        try:
            asyncio.ensure_future(self.run())
            self.event_loop.run_forever()
        except KeyboardInterrupt:
            print('\nReceived Keyboard Interrupt')
        finally:
            print('Program finished')


if __name__ == "__main__":
    DEVICE_NAME = "ROBOT_C4"
    event_queue = queue.Queue(1)
    lock = threading.Lock()
    lock2 = threading.Lock()

    dash = Dashboard(event_queue, lock, lock2)  

    def start_asyncio_loop():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        bt = Bluetooth(event_queue, "ROBOT_C4", lock, lock2, dash)
        asyncio.ensure_future(bt.run())
        loop.run_forever()

    ble_thread = threading.Thread(target=start_asyncio_loop, daemon=True)
    ble_thread.start()

    dash.root.mainloop()

    exit(0) 
