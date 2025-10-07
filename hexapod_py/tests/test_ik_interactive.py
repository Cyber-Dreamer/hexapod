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

def test_ik_interactive():
    """
    An interactive test for inverse kinematics with a UI to control each leg's
    x, y, z coordinates and display a 3D graph with each joint as a dot.
    """
    # Use HexapodLocomotion to get a pre-configured kinematics object
    locomotion = HexapodLocomotion()
    kinematics = locomotion.kinematics
    L1, L2, L3 = kinematics.leg_lengths

    # Initial foot positions in each leg's coordinate system
    # A typical neutral position might be extended forward and down.
    initial_foot_positions = np.array([
        [L1 + L2, 0, -L3/2] for _ in range(6)
    ])

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(left=0.1, right=0.8, bottom=0.45)

    # --- Helper function to get joint positions from angles ---
    def get_joint_positions(leg_idx, angles):
        if angles is None:
            return np.zeros((4, 3))

        # Get the base rotation for this leg around the Z-axis
        hip_angle = locomotion.hip_angles[leg_idx]
        Rz = np.array([
            [np.cos(hip_angle), -np.sin(hip_angle), 0],
            [np.sin(hip_angle),  np.cos(hip_angle), 0],
            [0,                  0,                 1]
        ])

        t1, t2, t3 = angles
        
        # Joint positions in leg's local coordinate frame
        p1_local = np.array([L1 * np.cos(t1), L1 * np.sin(t1), 0])
        p2_local = p1_local + np.array([L2 * np.cos(t2) * np.cos(t1), L2 * np.cos(t2) * np.sin(t1), L2 * np.sin(t2)])
        p3_local = p2_local + np.array([L3 * np.cos(t2 + t3) * np.cos(t1), L3 * np.cos(t2 + t3) * np.sin(t1), L3 * np.sin(t2 + t3)])

        # Transform to body coordinate frame
        hip_pos = np.array(kinematics.hip_positions[leg_idx])
        p1_body = hip_pos + Rz @ p1_local
        p2_body = hip_pos + Rz @ p2_local
        p3_body = hip_pos + Rz @ p3_local

        return np.array([hip_pos, p1_body, p2_body, p3_body])

    # Plot initial state
    lines = []
    dots = []
    for i in range(6):
        angles = kinematics.inverse_kinematics(initial_foot_positions[i], flip_factor=-1)
        leg_points = get_joint_positions(i, angles)
        line, = ax.plot(leg_points[:, 0], leg_points[:, 1], leg_points[:, 2], marker='o')
        dot = ax.scatter(leg_points[:, 0], leg_points[:, 1], leg_points[:, 2])
        lines.append(line)
        dots.append(dot)

    ax.set_xlabel('Body X (mm)')
    ax.set_ylabel('Body Y (mm)')
    ax.set_zlabel('Body Z (mm)')
    ax.set_title('Hexapod Inverse Kinematics')
    
    # Set plot limits to show the whole robot
    ax.set_xlim([-450, 450])
    ax.set_ylim([-450, 450])
    ax.set_zlim([-400, 100])
    ax.view_init(elev=20., azim=-75)
    ax.set_aspect('equal')

    # --- Text for displaying joint angles ---
    angle_text_str = "Joint Angles (deg):\n" + "\n".join([f"Leg {i}: --" for i in range(6)])
    angle_text = fig.text(
        0.82, 0.85, angle_text_str,
        verticalalignment='top',
        fontfamily='monospace'
    )


    sliders = []
    slider_axes = []

    def update(val):
        target_positions_local = np.copy(initial_foot_positions)
        for i in range(6):
            target_positions_local[i, 0] = sliders[i*3].val
            target_positions_local[i, 1] = sliders[i*3+1].val
            target_positions_local[i, 2] = sliders[i*3+2].val
        
        knee_direction = sliders[-1].val

        all_angles = []
        for i in range(6):
            angles = kinematics.inverse_kinematics(target_positions_local[i], flip_factor=knee_direction)
            all_angles.append(angles)

        for i in range(6):
            leg_points = get_joint_positions(i, all_angles[i])
            if all_angles[i] is not None:
                lines[i].set_data(leg_points[:, 0], leg_points[:, 1])
                lines[i].set_3d_properties(leg_points[:, 2])
                dots[i].set_offsets(leg_points[:, :2])
                dots[i].set_3d_properties(leg_points[:, 2], 'z')
                lines[i].set_color('C0') # Blue for reachable
            else:
                lines[i].set_color('red') # Red for unreachable

        # Update angle text display
        angle_strings = []
        for i, angles in enumerate(all_angles):
            if angles is not None:
                t1, t2, t3 = np.rad2deg(angles)
                angle_strings.append(f"Leg {i}: {t1:6.1f}, {t2:6.1f}, {t3:6.1f}")
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

    # Add a slider for knee direction
    ax_knee_slider = plt.axes([0.20, 0.35 - (6*0.045), 0.6, 0.015])
    knee_slider = Slider(
        ax=ax_knee_slider,
        label='Knee Direction',
        valmin=-1,
        valmax=1,
        valinit=-1,
        valstep=[-1, 1]
    )
    knee_slider.on_changed(update)
    sliders.append(knee_slider)

    plt.show()

if __name__ == '__main__':
    test_ik_interactive()