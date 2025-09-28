import asyncio

class GaitController:
    def __init__(self, name="tripod"):
        self.name = name

    async def step(self):
        # Simulate async gait step
        await asyncio.sleep(0.05)
        print(f"Gait {self.name}: step executed")

# More gait algorithms can be added here
