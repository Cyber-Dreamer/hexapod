# Hexapod Async Python Project

This project is a pure Python implementation of a hexapod robot controller, designed to run asynchronously for high performance and responsiveness. It includes modules for gait control, sensor integration, web-based teleoperation, and inverse kinematics.

## Structure
- `core/`: Main application logic and async event loop
- `gait/`: Gait algorithms and controllers
- `sensors/`: Sensor drivers and async interfaces
- `web/`: Async web server for teleoperation (FastAPI/Starlette)
- `kinematics/`: Inverse kinematics and geometry
- `utils/`: Utility functions and helpers
- `tests/`: Unit and integration tests

## Getting Started
1. Install dependencies: `pip install -r requirements.txt`
2. Run the main application: `python -m hexapod_py`

## Async Design
All modules are designed to use Python's `asyncio` for concurrency.
