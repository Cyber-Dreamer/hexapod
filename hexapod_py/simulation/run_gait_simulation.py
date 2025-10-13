import sys
import os
import time
import numpy as np

from multiprocessing import Process, Queue
# Add parent directory to path to import hexapod modules
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.kinematics.fk import HexapodForwardKinematics
from hexapod_py.simulation.simulator import HexapodSimulator
from hexapod_py.locomotion.locomotion import HexapodLocomotion
from hexapod_py.simulation.live_plotter import LivePlotter

def plotter_process(angle_queue, leg_lengths, hip_positions):
    """
    This function runs in a separate process to handle the live plotting.
    It receives joint angles via a queue and updates the plot.
    """
    try:
        fk_calculator = HexapodForwardKinematics(
            leg_lengths=leg_lengths,
            hip_positions=hip_positions
        )
        plotter = LivePlotter(hip_positions=hip_positions)

        while True:
            # Wait for new angles from the main process
            all_angles_rad = angle_queue.get()
            if all_angles_rad is None: # Sentinel value to exit
                break
            all_leg_points = fk_calculator.body_fk(all_angles_rad)
            plotter.update(all_leg_points)
    except KeyboardInterrupt:
        pass # Allow Ctrl+C to gracefully exit the process

def main():
    """
    Main function to run the hexapod gait simulation using the HexapodSimulator class.
    """
    # --- Initialize Simulator and Locomotion ---
    sim = HexapodSimulator(gui=True)
    locomotion = HexapodLocomotion(gait_type='tripod')

    # --- Setup for multiprocessing plotter ---
    angle_queue = Queue()
    plot_proc = Process(target=plotter_process, args=(
        angle_queue,
        locomotion.kinematics.segment_lengths,
        locomotion.kinematics.hip_positions
    ))
    plot_proc.start()

    try:
        # --- Start Simulation and Add Controls ---
        sim.start()
        sim.add_ui_controls()

        # --- Timing for Plot Updates ---
        # We still throttle how often we send data to the queue to avoid
        # overwhelming the plotter process.
        plot_update_interval = 1.0 / 30.0  # Target 30 FPS for plot data
        last_plot_send_time = time.time()

        # --- Simulation Loop ---
        while True:
            # 1. Read values from UI controls
            controls = sim.read_ui_controls()            
            vx, vy = controls['vx'], controls['vy']

            # Normalize the direction vector if its magnitude is > 1.0
            # This is consistent with the gait_test_ui.py logic.
            dir_vec = np.array([vx, vy])
            if np.linalg.norm(dir_vec) > 1.0:
                dir_vec_normalized = dir_vec / np.linalg.norm(dir_vec)
                vx, vy = dir_vec_normalized

            # Update parameters that might change during simulation
            locomotion.body_height = controls['body_height'] * 1000 # m to mm
            standoff_mm = controls['standoff'] * 1000 # m to mm
            # Only recalculate stance if the standoff distance has changed to avoid unnecessary computation
            if locomotion.standoff_distance != standoff_mm:
                locomotion.recalculate_stance(standoff_distance=standoff_mm)
 

            # 3. Run the gait logic to get target joint angles
            # Note: The URDF joint names are different from the locomotion leg order.
            # The HexapodSimulator class handles this mapping.
            all_angles_rad = locomotion.run_gait(
                vx=vx,
                vy=vy,
                omega=controls['omega'],
                roll=controls['roll'],
                pitch=controls['pitch'],
                step_height=controls['step_height'] * 1000 # m to mm
            )

            # Send joint angles to the plotter process at a throttled rate.
            current_time = time.time()
            if all_angles_rad is not None and (current_time - last_plot_send_time) > plot_update_interval:
                angle_queue.put(all_angles_rad)
                last_plot_send_time = current_time
                
            # 4. Set joint motor targets in PyBullet
            sim.set_joint_angles(all_angles_rad)

            # 5. Step the simulation
            sim.step()
            
            # Pace the simulation to run in real-time
            time.sleep(1./240.)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if sim.physics_client is not None:
            sim.stop()
        # Signal the plotter process to exit and wait for it to finish
        angle_queue.put(None)
        plot_proc.join()
        print("Plotter process terminated.")

if __name__ == '__main__':
    main()