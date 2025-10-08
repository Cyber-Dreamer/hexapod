import sys
import os
import time

# Add parent directory to path to import hexapod modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.simulation.simulator import HexapodSimulator
from hexapod_py.locomotion.locomotion import HexapodLocomotion

def main():
    """
    Main function to run the hexapod gait simulation using the HexapodSimulator class.
    """
    # --- Initialize Simulator and Locomotion ---
    sim = HexapodSimulator(gui=True)
    locomotion = HexapodLocomotion()

    try:
        # --- Start Simulation and Add Controls ---
        sim.start()
        sim.add_ui_controls()

        # Add a radio button for gait selection
        gait_options = list(locomotion.available_gaits.keys())
        sim.add_gait_selection_ui(gait_options)

        # --- Simulation Loop ---
        while True:
            # 1. Read values from UI controls
            controls = sim.read_ui_controls()
            
            # Read gait selection
            selected_gait_index = sim.read_gait_selection_ui()
            selected_gait = gait_options[selected_gait_index]

            # 2. Update locomotion controller with UI values
            if locomotion.gait_type != selected_gait:
                locomotion.set_gait(selected_gait)
                print(f"Switched to {selected_gait} gait.")

            # Update parameters that might change during simulation
            locomotion.body_height = controls['body_height'] * 1000 # m to mm
            locomotion.recalculate_stance() # Recalculate foot positions based on new height

            # 3. Run the gait logic to get target joint angles
            # Note: The URDF joint names are different from the locomotion leg order.
            # The HexapodSimulator class handles this mapping.
            all_angles_rad = locomotion.run_gait(
                vx=controls['vx'],
                vy=controls['vy'],
                omega=controls['omega'],
                roll=controls['roll'],
                pitch=controls['pitch'],
                speed=controls['speed'],
                step_height=controls['step_height'] * 1000, # m to mm
                step_length=controls['step_length'] * 1000 # m to mm
            )

            # 4. Set joint motor targets in PyBullet
            sim.set_joint_angles(all_angles_rad)

            # 5. Step the simulation
            sim.step()
            
            # Add a small delay to sync with the simulation's timestep
            time.sleep(1./240.)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if sim.physics_client is not None:
            sim.stop()

if __name__ == '__main__':
    main()