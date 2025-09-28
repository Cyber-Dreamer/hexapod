import asyncio
from gait.controller import GaitController
from sensors.base import AsyncSensor
from web.server import run_web_server

async def gait_task():
    gait = GaitController()
    while True:
        await gait.step()

async def sensor_task():
    sensor = AsyncSensor("dummy")
    while True:
        value = await sensor.read()
        print(f"Sensor value: {value}")

async def main():
    print("Hexapod async main loop starting...")
    tasks = [
        asyncio.create_task(gait_task()),
        asyncio.create_task(sensor_task()),
        asyncio.create_task(run_web_server()),
    ]
    await asyncio.gather(*tasks)
