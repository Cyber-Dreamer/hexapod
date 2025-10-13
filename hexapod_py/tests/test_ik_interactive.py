import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Add parent directory to path to import hexapod
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.locomotion.locomotion import HexapodLocomotion
from hexapod_py.kinematics.fk import HexapodForwardKinematics

def test_ik_interactive():
    """
    An interactive test for inverse kinematics. Sliders control the target
    (x, y, z) coordinates for each leg in its local frame.
    """
    # Use HexapodLocomotion to get a pre-configured kinematics object
    locomotion = HexapodLocomotion()
    kinematics = locomotion.kinematics
    
    # The FK calculator will be used to draw the legs based on calculated angles
    fk_calculator = HexapodForwardKinematics(
        leg_lengths=kinematics.segment_lengths,
        hip_positions=kinematics.hip_positions
    )

    # Initial foot positions in each leg's coordinate system
    # A typical neutral position might be extended forward and down.
    l_coxa, l_femur, l_tibia = kinematics.segment_lengths
    initial_foot_positions = np.array([
        [l_coxa + l_femur, 0, -l_tibia/2] for _ in range(6)
    ])

    fig = plt.figure(figsize=(12, 8))
    
    # Use a dark theme with a custom dark grey background
    plt.style.use('dark_background')
    dark_grey = '#2E2E2E'
    fig.patch.set_facecolor(dark_grey)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor(dark_grey)
    plt.subplots_adjust(left=0.1, right=0.8, bottom=0.45)

    # Plot initial state
    leg_colors = ['C0', 'C1', 'C2', 'C3', 'C4', 'C5']
    lines = [ax.plot([], [], [], 'o-', markersize=4, color=leg_colors[i])[0] for i in range(6)]
    
    # Draw the body outline by connecting the hip positions
    body_outline_points = np.array(kinematics.hip_positions)
    body_plot_points = np.vstack([body_outline_points, body_outline_points[0]])
    ax.plot(body_plot_points[:, 0], body_plot_points[:, 1], body_plot_points[:, 2], 'o-', color='gray', label='Body')

    ax.set_xlabel('Body X (mm)')
    ax.set_ylabel('Body Y (mm)')
    ax.set_zlabel('Body Z (mm)')
    ax.set_title('Hexapod Inverse Kinematics')
    
    ax.set_xlim([-450, 450])
    ax.set_ylim([-450, 450])
    ax.set_zlim([-400, 100])
    ax.view_init(elev=20., azim=-75)
    ax.set_aspect('equal')
    ax.legend()

    # --- Text for displaying joint angles ---
    angle_text_str = "Joint Angles (deg):\n" + "\n".join([f"Leg {i}: --" for i in range(6)])
    angle_text = fig.text(0.82, 0.95, angle_text_str, verticalalignment='top', fontfamily='monospace')

    sliders = []
    slider_axes = []

    def update(val):
        target_positions_local = np.zeros((6, 3))
        for i in range(6):
            target_positions_local[i, 0] = sliders[i*3].val
            target_positions_local[i, 1] = sliders[i*3+1].val
            target_positions_local[i, 2] = sliders[i*3+2].val

        all_angles = []
        for i in range(6):
            # Use the correct leg_ik method
            angles = kinematics.leg_ik(target_positions_local[i])
            all_angles.append(angles)

        # Use body_fk to get world coordinates for all legs
        # For this test, body translation and rotation are zero.
        all_leg_points = fk_calculator.body_fk(all_angles)

        # Update the plot
        for i, leg_points in enumerate(all_leg_points):
            if leg_points.shape[0] > 1: # Check if the leg is reachable
                lines[i].set_data(leg_points[:, 0], leg_points[:, 1])
                lines[i].set_3d_properties(leg_points[:, 2])
                lines[i].set_color(leg_colors[i])
            else: # Unreachable, leg_points will just be the hip position
                lines[i].set_data(leg_points[:, 0], leg_points[:, 1])
                lines[i].set_3d_properties(leg_points[:, 2])
                lines[i].set_color('red')

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

    # Create sliders for each leg's x, y, z in their local frame
    slider_labels = ['Local X', 'Local Y', 'Local Z']
    slider_ranges = {
        'Local X': (-100, 400),
        'Local Y': (-200, 200),
        'Local Z': (-350, 50)
    }

    for i in range(6):
        for j in range(3):
            ax_slider = plt.axes([0.20, 0.35 - (i*0.045 + j*0.015), 0.6, 0.01])
            ax_slider.set_facecolor(dark_grey)
            slider_axes.append(ax_slider)
            
            label = f'Leg {i} {slider_labels[j]}'
            s_range = slider_ranges[slider_labels[j]]
            initial_val = initial_foot_positions[i, j]

            slider = Slider(
                ax=ax_slider,
                label=label,
                valmin=s_range[0],
                valmax=s_range[1],
                valinit=initial_val,
            )
            slider.on_changed(update)
            sliders.append(slider)

    update(0) # Initial call to draw the robot
    plt.show()

if __name__ == '__main__':
    test_ik_interactive()