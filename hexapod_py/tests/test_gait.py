import pytest
import asyncio
from gait.controller import GaitController

@pytest.mark.asyncio
async def test_gait_step():
    gait = GaitController()
    await gait.step()
    assert gait.name == "tripod"
