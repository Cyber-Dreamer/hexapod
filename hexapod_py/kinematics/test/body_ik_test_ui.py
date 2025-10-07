import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Add parent directory to path to import hexapod
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.locomotion.locomotion import HexapodLocomotion
from hexapod_py.kinematics.fk import HexapodForwardKinematics

def test_body_ik_interactive():
    """
    An interactive test for full-body inverse kinematics. Sliders control the
    body's translation and rotation, and a 3D plot displays the resulting posture.
    """
    # Use HexapodLocomotion to get a pre-configured robot model
    locomotion = HexapodLocomotion()
    kinematics = locomotion.kinematics
    
    # The FK calculator will be used to draw the legs based on calculated angles
    fk_calculator = HexapodForwardKinematics(
        leg_lengths=kinematics.leg_lengths,
        hip_positions=kinematics.hip_positions
    )

    # --- UI Setup ---
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(left=0.1, right=0.8, bottom=0.45)

    # --- Initial State ---
    initial_translation = np.array([0.0, 0.0, 0.0])
    initial_rotation = np.array([0.0, 0.0, 0.0]) # Roll, Pitch, Yaw

    # Get the default foot positions when standing neutral
    # These are relative to the body center
    locomotion.recalculate_stance()
    default_foot_positions_world = np.array(locomotion.default_foot_positions)

    # --- Plotting Setup ---
    lines = [ax.plot([], [], [], 'o-', markersize=4)[0] for _ in range(6)]
    body_center_dot = ax.scatter([], [], [], c='red', s=50, label='Body Center')

    ax.set_xlabel('World X (mm)')
    ax.set_ylabel('World Y (mm)')
    ax.set_zlabel('World Z (mm)')
    ax.set_title('Hexapod Body Inverse Kinematics')
    
    # Set plot limits to show the whole robot
    ax.set_xlim([-450, 450])
    ax.set_ylim([-450, 450])
    ax.set_zlim([-350, 150])
    ax.view_init(elev=20., azim=-75)
    ax.set_aspect('equal')

    # --- Text for displaying joint angles ---
    angle_text_str = "Joint Angles (deg):\n" + "\n".join([f"Leg {i}: --" for i in range(6)])
    angle_text = fig.text(0.82, 0.95, angle_text_str, verticalalignment='top', fontfamily='monospace')

    # --- Sliders ---
    slider_axes = {
        'tx': plt.axes([0.25, 0.30, 0.5, 0.02]), 'ty': plt.axes([0.25, 0.27, 0.5, 0.02]), 'tz': plt.axes([0.25, 0.24, 0.5, 0.02]),
        'roll': plt.axes([0.25, 0.18, 0.5, 0.02]), 'pitch': plt.axes([0.25, 0.15, 0.5, 0.02]), 'yaw': plt.axes([0.25, 0.12, 0.5, 0.02]),
        'knee': plt.axes([0.25, 0.06, 0.5, 0.02])
    }
    sliders = {
        'tx': Slider(slider_axes['tx'], 'Translate X', -100, 100, valinit=0),
        'ty': Slider(slider_axes['ty'], 'Translate Y', -100, 100, valinit=0),
        'tz': Slider(slider_axes['tz'], 'Translate Z', -50, 50, valinit=0),
        'roll': Slider(slider_axes['roll'], 'Roll', -np.pi/6, np.pi/6, valinit=0),
        'pitch': Slider(slider_axes['pitch'], 'Pitch', -np.pi/6, np.pi/6, valinit=0),
        'yaw': Slider(slider_axes['yaw'], 'Yaw', -np.pi/6, np.pi/6, valinit=0),
        'knee': Slider(slider_axes['knee'], 'Knee Dir', -1, 1, valinit=-1, valstep=[-1, 1])
    }

    def update(val):
        # 1. Read values from sliders
        translation = np.array([sliders['tx'].val, sliders['ty'].val, sliders['tz'].val])
        rotation = np.array([sliders['roll'].val, sliders['pitch'].val, sliders['yaw'].val])
        knee_dir = sliders['knee'].val

        # 2. Calculate Body IK
        # This gives us the new target for each foot tip, relative to the body center.
        new_foot_targets_world = kinematics.body_ik(
            translation,
            rotation,
            kinematics.hip_positions,
            default_foot_positions_world
        )

        all_angles = []
        unreachable_legs = []

        # 3. For each leg, calculate Leg IK to find the joint angles
        for i in range(6):
            # Convert the world-frame foot target to the leg's local coordinate frame
            foot_target_local = new_foot_targets_world[i] - kinematics.hip_positions[i]
            
            angles = kinematics.leg_ik(
                foot_target_local,
                *kinematics.leg_lengths,
                knee_direction=knee_dir
            )
            all_angles.append(angles)
            if angles is None:
                unreachable_legs.append(i)

        # 4. Use Forward Kinematics to draw the robot
        for i in range(6):
            angles = all_angles[i]
            if angles is not None:
                # We use the FK function with the body's transform to get world coordinates
                leg_points = fk_calculator.leg_fk(angles, i, translation, rotation)
                lines[i].set_data(leg_points[:, 0], leg_points[:, 1])
                lines[i].set_3d_properties(leg_points[:, 2])
                lines[i].set_color('C0') # Blue for reachable
            else:
                # If unreachable, just show the hip position
                hip_pos = fk_calculator.leg_fk((0,0,0), i, translation, rotation)[0]
                lines[i].set_data([hip_pos[0]], [hip_pos[1]])
                lines[i].set_3d_properties([hip_pos[2]])
                lines[i].set_color('red')

        # Update body center dot
        body_center_dot._offsets3d = ([translation[0]], [translation[1]], [translation[2]])

        # Update angle text display
        angle_strings = []
        for i, angles in enumerate(all_angles):
            if angles is not None:
                gamma, alpha, beta = np.rad2deg(angles)
                angle_strings.append(f"Leg {i}: {gamma:6.1f}, {alpha:6.1f}, {beta:6.1f}")
            else:
                angle_strings.append(f"Leg {i}: Unreachable")
        angle_text.set_text("Joint Angles (deg):\n" + "\n".join(angle_strings))

        fig.canvas.draw_idle()

    # Attach the update function to all sliders
    for slider in sliders.values():
        slider.on_changed(update)

    update(0) # Initial call to draw the robot
    plt.show()

if __name__ == '__main__':
    test_body_ik_interactive()