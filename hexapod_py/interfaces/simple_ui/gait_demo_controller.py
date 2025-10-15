"""
Gait Demo Controller
====================

This module provides a controller that connects a simple PyBullet UI to a
HexapodPlatform (either a simulator or a real robot). It allows real-time
control over the hexapod's gait and body pose.
"""

import time
import numpy as np
import pybullet as p
import os
import sys

# Add parent directory to path to import hexapod modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.platform.hexapod_platform import HexapodPlatform
from hexapod_py.locomotion.locomotion import HexapodLocomotion
from hexapod_py.platform.simulation.simulator import HexapodSimulator # Used for type checking

class GaitDemoController:
    """
    A controller that uses PyBullet's debug sliders to control a hexapod's gait.

    This class is platform-agnostic. It can control a HexapodSimulator or could
    be adapted to send commands to a physical robot, as long as the UI is
    running within a PyBullet instance.
    """
    def __init__(self, platform: HexapodPlatform, locomotion: HexapodLocomotion):
        self.platform = platform
        self.locomotion = locomotion
        self.debug_param_ids = {}

        # UI controls are specific to the simulator, so we need a reference to it.
        # If the platform is not a simulator, UI controls cannot be created.
        self.sim = self.platform if isinstance(self.platform, HexapodSimulator) else None

    def _add_ui_controls(self):
        """Adds debug sliders to the GUI for real-time control."""
        if not self.sim or not self.sim.gui:
            print("Cannot add UI controls: Not in GUI simulation mode.")
            return
        self.debug_param_ids['vx'] = p.addUserDebugParameter("Vx (fwd/bwd)", -1.0, 1.0, 0)
        self.debug_param_ids['vy'] = p.addUserDebugParameter("Vy (strafe)", -1.0, 1.0, 0)
        self.debug_param_ids['omega'] = p.addUserDebugParameter("Omega (turn)", -1.0, 1.0, 0)
        self.debug_param_ids['body_height'] = p.addUserDebugParameter("Body Height (m)", 0.15, 0.3, 0.20)
        self.debug_param_ids['step_height'] = p.addUserDebugParameter("Step Height (m)", 0.01, 0.08, 0.04)
        self.debug_param_ids['standoff'] = p.addUserDebugParameter("Standoff (m)", 0.2, 0.5, 0.28)
        self.debug_param_ids['roll'] = p.addUserDebugParameter("Roll (rad)", -0.5, 0.5, 0.0)
        self.debug_param_ids['pitch'] = p.addUserDebugParameter("Pitch (rad)", -0.5, 0.5, 0.0)

    def _read_ui_controls(self):
        """
        Reads the current values from the GUI sliders.
        Returns a dictionary of default values if not in GUI mode.
        """
        if not self.sim or not self.sim.gui or not self.debug_param_ids:
            return {'vx': 0.0, 'vy': 0.0, 'omega': 0.0, 'body_height': 0.20, 'pitch': 0.0, 'roll': 0.0, 'step_height': 0.04, 'standoff': 0.28}

        return {key: p.readUserDebugParameter(self.debug_param_ids[key]) for key in self.debug_param_ids}

    def run(self):
        """
        Starts the platform and runs the main control loop.
        """
        try:
            self.platform.start()
            self._add_ui_controls()

            print("Starting UI control loop. Press Ctrl+C in terminal to exit.")
            while True:
                # 1. Read values from UI controls
                controls = self._read_ui_controls()
                vx, vy = controls['vx'], controls['vy']

                # Normalize the direction vector if its magnitude is > 1.0
                dir_vec = np.array([vx, vy])
                if np.linalg.norm(dir_vec) > 1.0:
                    dir_vec_normalized = dir_vec / np.linalg.norm(dir_vec)
                    vx, vy = dir_vec_normalized

                # 2. Update locomotion parameters from UI
                # Convert meters from UI to millimeters for the locomotion module
                self.locomotion.body_height = controls['body_height'] * 1000
                standoff_mm = controls['standoff'] * 1000
                if self.locomotion.standoff_distance != standoff_mm:
                    self.locomotion.recalculate_stance(standoff_distance=standoff_mm)

                # 3. Run the gait logic to get target joint angles
                all_angles_rad = self.locomotion.run_gait(
                    vx=vx,
                    vy=vy,
                    omega=controls['omega'],
                    roll=controls['roll'],
                    pitch=controls['pitch'],
                    step_height=controls['step_height'] * 1000 # m to mm
                )

                # 4. Send joint angles to the platform (simulator or real robot)
                self.platform.set_joint_angles(all_angles_rad)

                # The platform (simulator) now runs its own update loop.
                # We just need to pace the controller's command generation rate.
                time.sleep(1./100.) # Send commands at 100Hz

        except Exception as e:
            print(f"An error occurred in the control loop: {e}")
        finally:
            print("Stopping platform...")
            self.platform.stop()