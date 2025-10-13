import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Add parent directory to path to import hexapod
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.kinematics.fk import HexapodForwardKinematics
from hexapod_py.kinematics.ik import HexapodKinematics

def test_body_ik_interactive():
    """
    An interactive test for full-body inverse kinematics. Sliders control the
    body's translation and rotation, and a 3D plot displays the resulting posture.
    """
    # Use HexapodLocomotion to get a pre-configured robot model.
    # Define hexapod geometry directly to decouple from the locomotion module
    center_to_hip_joint = 152.024
    leg_lengths = [92.5, 191.8, 284.969] # Coxa, Femur, Tibia
    body_height = 200
    initial_standoff = 380

    # Leg numbering: 0:RL, 1:ML, 2:FL, 3:FR, 4:MR, 5:RR
    hip_angles = np.deg2rad([210, 270, 330, 30, 90, 150])
    hip_positions = np.array([
        (center_to_hip_joint * np.cos(angle), center_to_hip_joint * np.sin(angle), 0)
        for angle in hip_angles
    ])

    # Define joint limits [coxa, femur, tibia] in radians
    joint_limits = np.deg2rad([90, 110, 120])

    kinematics = HexapodKinematics(
        segment_lengths=leg_lengths,
        hip_positions=hip_positions,
        joint_limits=joint_limits
    )
    
    # The FK calculator will be used to draw the legs based on calculated angles
    fk_calculator = HexapodForwardKinematics(
        leg_lengths=kinematics.segment_lengths,
        hip_positions=kinematics.hip_positions
    )    

    # --- UI Setup ---
    fig = plt.figure(figsize=(12, 9))

    # Use a dark theme with a custom dark grey background
    plt.style.use('dark_background')
    dark_grey = '#2E2E2E'
    fig.patch.set_facecolor(dark_grey)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor(dark_grey)
    plt.subplots_adjust(left=0.1, right=0.8, bottom=0.45)

    # --- Initial State ---
    initial_translation = np.array([0.0, 0.0, 0.0])
    initial_rotation = np.array([0.0, 0.0, 0.0]) # Roll, Pitch, Yaw

    # Helper function to calculate default foot positions in the world frame
    def calculate_stance(standoff, height):
        foot_positions = []
        for pos in hip_positions:
            v = np.array([pos[0], pos[1], 0])
            v_norm = v / np.linalg.norm(v) if np.linalg.norm(v) > 0 else v
            default_pos_xy = v + v_norm * standoff
            foot_positions.append(np.array([default_pos_xy[0], default_pos_xy[1], -height]))
        return np.array(foot_positions)

    # Get the initial default foot positions
    default_foot_positions_world = calculate_stance(initial_standoff, body_height)


    # --- Plotting Setup ---
    leg_colors = ['C0', 'C1', 'C2', 'C3', 'C4', 'C5'] # Standard matplotlib colors
    lines = [ax.plot([], [], [], 'o-', markersize=4, color=leg_colors[i])[0] for i in range(6)]
    body_center_dot = ax.scatter([], [], [], c='red', s=50, label='Body Center')
    
    # Add markers for the target foot positions. These will be updated dynamically.
    foot_targets_plot = ax.scatter(default_foot_positions_world[:, 0], 
                                   default_foot_positions_world[:, 1], 
                                   default_foot_positions_world[:, 2],
                                   c=leg_colors, marker='x', s=100, label='Foot Targets (World)')

    # Draw the body outline by connecting the hip positions
    body_line, = ax.plot([], [], [], '-', color='gray', label='Body')


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
    ax.legend()

    # --- Text for displaying joint angles ---
    angle_text_str = "Joint Angles (deg):\n" + "\n".join([f"Leg {i}: --" for i in range(6)])
    angle_text = fig.text(0.82, 0.95, angle_text_str, verticalalignment='top', fontfamily='monospace')

    # --- Sliders ---
    slider_axes = {
        'tx': plt.axes([0.25, 0.30, 0.5, 0.02]), 'ty': plt.axes([0.25, 0.27, 0.5, 0.02]), 'tz': plt.axes([0.25, 0.24, 0.5, 0.02]),
        'roll': plt.axes([0.25, 0.18, 0.5, 0.02]), 'pitch': plt.axes([0.25, 0.15, 0.5, 0.02]), 'yaw': plt.axes([0.25, 0.12, 0.5, 0.02]),
        'standoff': plt.axes([0.25, 0.09, 0.5, 0.02])
    }
    for ax_s in slider_axes.values():
        ax_s.set_facecolor(dark_grey)
    sliders = {
        'tx': Slider(slider_axes['tx'], 'Translate X', -150, 150, valinit=0),
        'ty': Slider(slider_axes['ty'], 'Translate Y', -150, 150, valinit=0),
        'tz': Slider(slider_axes['tz'], 'Translate Z', -100, 100, valinit=0),
        'roll': Slider(slider_axes['roll'], 'Roll', -np.pi/4, np.pi/4, valinit=0),
        'pitch': Slider(slider_axes['pitch'], 'Pitch', -np.pi/4, np.pi/4, valinit=0),
        'yaw': Slider(slider_axes['yaw'], 'Yaw', -np.pi/4, np.pi/4, valinit=0),
        'standoff': Slider(slider_axes['standoff'], 'Standoff', 200, 500, valinit=initial_standoff)
    }

    # Store the last known valid slider values to prevent entering impossible positions
    last_valid_values = {key: slider.valinit for key, slider in sliders.items()}
    
    # Flag to prevent recursive updates when resetting sliders to a valid state.
    _is_programmatic_update = False

    def update(val):
        nonlocal _is_programmatic_update
        if _is_programmatic_update:
            return

        # 1. Read values from sliders
        translation = np.array([sliders['tx'].val, sliders['ty'].val, sliders['tz'].val])
        rotation = np.array([sliders['roll'].val, sliders['pitch'].val, sliders['yaw'].val])
        standoff = sliders['standoff'].val

        # 2. Update stance based on standoff slider and get new default foot positions
        default_foot_positions_world = calculate_stance(standoff, body_height)
 
        # 2. Calculate Body IK
        # This gives us the new target for each foot tip, relative to the body center.
        new_foot_targets_local = kinematics.body_ik(
            translation,
            rotation,
            hip_positions,
            default_foot_positions_world
        )

        all_angles = []
        # unreachable_legs = [] # This variable is not used

        # 3. For each leg, calculate Leg IK to find the joint angles
        for i in range(6):
            # The body_ik function already returns the target in the leg's local frame.
            foot_target_local = new_foot_targets_local[i]
            
            angles = kinematics.leg_ik(foot_target_local, knee_direction=-1)
            all_angles.append(angles)
            if angles is None:
                # unreachable_legs.append(i) # This variable is not used
                pass
        
        # 4. Check if the pose is reachable. If not, revert sliders and stop.
        if any(angles is None for angles in all_angles):
            _is_programmatic_update = True
            # Revert sliders to the last valid position to prevent invalid state
            for key, slider in sliders.items():
                slider.set_val(last_valid_values[key])
            _is_programmatic_update = False
            
            # Find which leg was unreachable for the message
            unreachable_idx = next(i for i, v in enumerate(all_angles) if v is None)
            angle_text.set_text(f"Pose Unreachable!\n(Leg {unreachable_idx} failed)")
            
            # Stop further processing for this invalid state
            fig.canvas.draw_idle()
            return

        # 5. Pose is valid, so update the last known good values
        for key, slider in sliders.items():
            last_valid_values[key] = slider.val

        # 6. Use body_fk to get world coordinates for all legs and update the plot
        all_leg_points = fk_calculator.body_fk(all_angles, translation, rotation)

        for i in range(6):
            leg_points = all_leg_points[i]
            lines[i].set_data(leg_points[:, 0], leg_points[:, 1])
            lines[i].set_3d_properties(leg_points[:, 2])                
            lines[i].set_color(leg_colors[i])

        # Update the body line to reflect the new position and orientation
        R = fk_calculator.body_fk([], body_translation=translation, body_rotation=rotation)
        rotated_hips = np.array([p[0] for p in R])
        body_plot_points = np.vstack([rotated_hips, rotated_hips[0]])
        body_line.set_data(body_plot_points[:, 0], body_plot_points[:, 1])
        body_line.set_3d_properties(body_plot_points[:, 2])

        # Update body center dot
        body_center_dot._offsets3d = ([translation[0]], [translation[1]], [translation[2]])

        # Update foot target markers
        foot_targets_plot._offsets3d = (default_foot_positions_world[:, 0],
                                        default_foot_positions_world[:, 1],
                                        default_foot_positions_world[:, 2])


        # Update angle text display
        angle_strings = [f"Leg {i}: {np.rad2deg(a)[0]:6.1f}, {np.rad2deg(a)[1]:6.1f}, {np.rad2deg(a)[2]:6.1f}" for i, a in enumerate(all_angles)]
        angle_text.set_text("Joint Angles (deg):\n" + "\n".join(angle_strings))

        fig.canvas.draw_idle()

    # Attach the update function to all sliders
    for slider in sliders.values():
        slider.on_changed(update)

    update(0) # Initial call to draw the robot
    plt.show()

if __name__ == '__main__':
    test_body_ik_interactive()