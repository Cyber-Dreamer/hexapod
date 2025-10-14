"""
Hexapod MuJoCo Gait Simulation
==============================

This script runs a MuJoCo simulation to test the gait locomotion of the
hexapod robot. It uses the new `mujoco.viewer` which provides built-in
UI sliders to control the robot's velocity in real-time.

This serves as an alternative to the PyBullet simulation, showcasing how the
same locomotion and kinematics code can be used with a different physics engine.

Prerequisites:
--------------
You need to have MuJoCo and its Python bindings installed:
`pip install mujoco`

You might also need a compatible EGL library on some systems for headless rendering,
though the interactive viewer is the primary focus here.
"""

import sys
import os
import time
import numpy as np
import mujoco
import mujoco.viewer

# Add parent directory to path to import hexapod modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.locomotion.locomotion import HexapodLocomotion

# --- Constants and Mappings ---

# This mapping translates the locomotion leg order (0-5) to the URDF joint names.
# It must match the order in `simulator.py` and the URDF file structure.
# Locomotion Order: 0:RL, 1:ML, 2:FL, 3:FR, 4:MR, 5:RR
LEG_JOINT_NAMES_LOCO_ORDER = [
    ["hip_6", "ties_6", "foot_6"], # Leg 0: Rear-Left (RL)
    ["hip_1", "ties_1", "foot_1"], # Leg 1: Middle-Left (ML)
    ["hip_2", "ties_2", "foot_2"], # Leg 2: Front-Left (FL)
    ["hip_3", "ties_3", "foot_3"], # Leg 3: Front-Right (FR)
    ["hip_4", "ties_4", "foot_4"], # Leg 4: Middle-Right (MR)
    ["hip_5", "ties_5", "foot_5"], # Leg 5: Rear-Right (RR)
]

def toggle_floor_visibility(viewer):
    """Callback to toggle the visibility of the floor in the MuJoCo viewer."""
    viewer.user_scn.flags[mujoco.mjtVisFlag.mjVIS_FLOOR] ^= 1

def main():
    """Main function to run the hexapod MuJoCo simulation."""

    # --- Model Loading and Compilation ---
    urdf_path = os.path.join(os.path.dirname(__file__), 'robot.urdf')
    try:
        model = mujoco.MjModel.from_xml_path(urdf_path)
    except Exception as e:
        print(f"Error loading URDF for MuJoCo: {e}")
        print("MuJoCo's URDF importer is stricter than PyBullet's. Please ensure the URDF is well-formed.")
        return

    data = mujoco.MjData(model)

    # --- Map Joint Names to MuJoCo Indices ---
    # Create a mapping from joint names in the URDF to their index in the MuJoCo model.
    joint_name_to_id = {model.joint(i).name: i for i in range(model.njnt)}

    # Create a list of lists containing the MuJoCo joint IDs in locomotion order.
    loco_to_mujoco_ids = []
    for leg_joints in LEG_JOINT_NAMES_LOCO_ORDER:
        mujoco_ids = [joint_name_to_id.get(name) for name in leg_joints]
        if any(id is None for id in mujoco_ids):
            print(f"Error: A joint in {leg_joints} was not found in the MuJoCo model.")
            return
        loco_to_mujoco_ids.append(mujoco_ids)

    # --- Initialize Locomotion Logic ---
    locomotion = HexapodLocomotion(gait_type='tripod')

    # --- Launch Interactive Viewer ---
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Add a key callback to toggle the floor visibility (press 'F')
        mujoco.viewer.add_viewer_key_callback(viewer, 'f', toggle_floor_visibility)
        
        # Add sliders for gait control
        viewer.add_control("Vx (fwd/bwd)", "vx", (-1.0, 1.0), 0.0)
        viewer.add_control("Vy (strafe)", "vy", (-1.0, 1.0), 0.0)
        viewer.add_control("Omega (turn)", "omega", (-1.0, 1.0), 0.0)
        viewer.add_control("Body Height (mm)", "body_height", (150, 300), 200)
        viewer.add_control("Step Height (mm)", "step_height", (10, 80), 40)
        viewer.add_control("Standoff (mm)", "standoff", (200, 500), 300)

        # Set initial stance
        standoff_mm = viewer.get_control("standoff")
        locomotion.recalculate_stance(standoff_distance=standoff_mm)

        # --- Simulation Loop ---
        while viewer.is_running():
            step_start = time.time()

            # 1. Read values from UI controls
            vx = viewer.get_control("vx")
            vy = viewer.get_control("vy")
            omega = viewer.get_control("omega")
            body_height_mm = viewer.get_control("body_height")
            step_height_mm = viewer.get_control("step_height")
            standoff_mm = viewer.get_control("standoff")

            # Normalize the direction vector if its magnitude is > 1.0
            dir_vec = np.array([vx, vy])
            if np.linalg.norm(dir_vec) > 1.0:
                dir_vec_normalized = dir_vec / np.linalg.norm(dir_vec)
                vx, vy = dir_vec_normalized

            # Update parameters that might change during simulation
            locomotion.body_height = body_height_mm
            if locomotion.standoff_distance != standoff_mm:
                locomotion.recalculate_stance(standoff_distance=standoff_mm)

            # 2. Run the gait logic to get target joint angles
            all_angles_rad = locomotion.run_gait(
                vx=vx,
                vy=vy,
                omega=omega,
                step_height=step_height_mm
            )

            # 3. Set joint actuator targets in MuJoCo
            if all_angles_rad:
                for leg_idx, leg_angles in enumerate(all_angles_rad):
                    if leg_angles is not None:
                        for joint_idx, angle in enumerate(leg_angles):
                            mujoco_joint_id = loco_to_mujoco_ids[leg_idx][joint_idx]
                            # Set the position target for the actuator
                            data.ctrl[mujoco_joint_id] = angle

            # 4. Step the simulation
            mujoco.mj_step(model, data)

            # 5. Sync viewer and pace the simulation
            viewer.sync()
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == '__main__':
    main()