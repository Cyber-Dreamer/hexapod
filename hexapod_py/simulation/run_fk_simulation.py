"""
Hexapod Forward Kinematics Simulation
=====================================

This script runs a PyBullet simulation to test the forward kinematics of the
hexapod robot. It creates UI sliders to control the joint angles of each leg
independently. This is useful for debugging the URDF model, joint limits,
and the simulator's joint mapping.
"""

import sys
import os
import time
import numpy as np
import pybullet as p

# Add parent directory to path to import hexapod modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.simulation.simulator import HexapodSimulator

def add_fk_controls(sim_gui=True):
    """Adds debug sliders for all 18 joints (3 per leg)."""
    if not sim_gui:
        print("Cannot add UI controls in non-GUI mode.")
        return {}
    debug_param_ids = {}

    # Locomotion Order: 0:RL, 1:ML, 2:FL, 3:FR, 4:MR, 5:RR
    leg_names = ["RL", "ML", "FL", "FR", "MR", "RR"]
    joint_names = ["Coxa", "Femur", "Tibia"]
    
    # Angle limits in degrees for the sliders
    # Coxa: +/- 90, Femur: +/- 110, Tibia: +/- 120
    angle_limits = [90, 110, 120] 

    for leg_idx in range(6):
        for joint_idx in range(3):
            param_name = f"L{leg_idx}_{leg_names[leg_idx]}_{joint_names[joint_idx]}"
            limit = angle_limits[joint_idx]
            # Add slider and store its ID
            debug_param_ids[param_name] = p.addUserDebugParameter(
                param_name, -limit, limit, 0
            )
    return debug_param_ids

def read_fk_controls(debug_param_ids, sim_gui=True):
    """Reads the angle values from all 18 sliders and returns them in radians."""
    all_angles_rad = [[0.0] * 3 for _ in range(6)]
    if not sim_gui or not debug_param_ids:
        return all_angles_rad

    leg_names = ["RL", "ML", "FL", "FR", "MR", "RR"]
    joint_names = ["Coxa", "Femur", "Tibia"]

    for leg_idx in range(6):
        for joint_idx in range(3):
            param_name = f"L{leg_idx}_{leg_names[leg_idx]}_{joint_names[joint_idx]}"
            angle_deg = p.readUserDebugParameter(debug_param_ids[param_name])
            all_angles_rad[leg_idx][joint_idx] = np.deg2rad(angle_deg)
    
    return all_angles_rad

def main():
    """Main function to run the hexapod FK simulation."""
    sim = HexapodSimulator(gui=True)

    try:
        sim.start()
        # Add specific controls for FK and get their IDs
        fk_control_ids = add_fk_controls(sim_gui=sim.gui)

        while True:
            # 1. Read joint angles from UI controls
            all_angles_rad = read_fk_controls(fk_control_ids, sim_gui=sim.gui)

            # 2. Set joint motor targets in PyBullet
            sim.set_joint_angles(all_angles_rad)

            # The simulator now runs its own step loop in the background.
            # We just need to sleep here to prevent this loop from running too fast.
            time.sleep(1./100.)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if sim.physics_client is not None:
            sim.stop()

if __name__ == '__main__':
    main()