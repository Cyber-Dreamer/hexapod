from fastapi import FastAPI
import asyncio

app = FastAPI()

@app.get("/status")
async def status():
    return {"status": "Hexapod running"}

# More endpoints for teleoperation will be added here

async def run_web_server():
    import uvicorn
    config = uvicorn.Config(app, host="0.0.0.0", port=8000, log_level="info")
    server = uvicorn.Server(config)
    await server.serve()
