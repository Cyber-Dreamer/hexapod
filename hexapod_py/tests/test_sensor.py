import pytest
import asyncio
from sensors.base import AsyncSensor

@pytest.mark.asyncio
async def test_sensor_read():
    sensor = AsyncSensor("test")
    value = await sensor.read()
    assert value == 42
