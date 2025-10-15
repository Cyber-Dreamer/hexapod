import asyncio

class AsyncSensor:
    def __init__(self, name):
        self.name = name
        self.value = None

    async def read(self):
        # Simulate async sensor reading
        await asyncio.sleep(0.1)
        self.value = 42  # Dummy value
        return self.value

# Example: add more sensor classes as needed
