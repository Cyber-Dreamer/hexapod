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

def test_body_fk_interactive():
    """
    An interactive test for full-body forward kinematics. Sliders control each of
    the 18 joint angles, and a 3D plot displays the resulting robot posture.
    """
    # Define hexapod geometry directly to decouple from the locomotion module
    center_to_hip_joint = 152.024
    leg_lengths = [92.5, 191.8, 284.969] # Coxa, Femur, Tibia

    # Leg numbering: 0:RL, 1:ML, 2:FL, 3:FR, 4:MR, 5:RR
    hip_angles = np.deg2rad([210, 270, 330, 30, 90, 150])
    hip_positions = np.array([
        (center_to_hip_joint * np.cos(angle), center_to_hip_joint * np.sin(angle), 0)
        for angle in hip_angles
    ])
    
    # The FK calculator will be used to draw the legs based on slider angles
    fk_calculator = HexapodForwardKinematics(
        leg_lengths=leg_lengths,
        hip_positions=hip_positions
    )

    # --- UI Setup ---
    fig = plt.figure(figsize=(12, 10))

    # Use a dark theme with a custom dark grey background
    plt.style.use('dark_background')
    dark_grey = '#2E2E2E'
    fig.patch.set_facecolor(dark_grey)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor(dark_grey)
    plt.subplots_adjust(left=0.05, right=0.95, bottom=0.4, top=0.95)

    # --- Initial State ---
    # Start with all angles at 0
    initial_angles_deg = np.zeros((6, 3))

    # --- Plotting Setup ---
    leg_colors = ['C0', 'C1', 'C2', 'C3', 'C4', 'C5']
    lines = [ax.plot([], [], [], 'o-', markersize=4, color=leg_colors[i])[0] for i in range(6)]
    
    # Draw the body outline by connecting the hip positions
    body_outline_points = hip_positions
    # Close the loop for drawing
    body_plot_points = np.vstack([body_outline_points, body_outline_points[0]])
    ax.plot(body_plot_points[:, 0], body_plot_points[:, 1], body_plot_points[:, 2], 'o-', color='gray', label='Body')

    ax.set_xlabel('Body X (mm)')
    ax.set_ylabel('Body Y (mm)')
    ax.set_zlabel('Body Z (mm)')
    ax.set_title('Hexapod Body Forward Kinematics')
    
    # Set plot limits to show the whole robot
    ax.set_xlim([-450, 450])
    ax.set_ylim([-450, 450])
    ax.set_zlim([-350, 150])
    ax.view_init(elev=20., azim=-75)
    ax.set_aspect('equal')
    ax.legend()

    # --- Sliders ---
    sliders = []
    slider_axes = []
    angle_labels = ['γ (Coxa)', 'α (Femur)', 'β (Tibia)']
    angle_limits = {
        'γ (Coxa)': (-90, 90),
        'α (Femur)': (-110, 110),
        'β (Tibia)': (-120, 120)
    }

    # Create 18 sliders (3 for each of the 6 legs)
    slider_start_y = 0.3
    slider_height = 0.012
    for i in range(6): # For each leg
        for j in range(3): # For each joint (gamma, alpha, beta)
            # Position sliders in two columns
            x_pos = 0.1 if i < 3 else 0.55
            y_pos = slider_start_y - ((i % 3) * 0.1 + j * 0.025)
            
            ax_slider = plt.axes([x_pos, y_pos, 0.3, slider_height])
            slider_axes.append(ax_slider)
            ax_slider.set_facecolor(dark_grey)
            
            label = f'L{i} {angle_labels[j]}'
            limits = angle_limits[angle_labels[j]]
            initial_val = initial_angles_deg[i, j]

            slider = Slider(
                ax=ax_slider,
                label=label,
                valmin=limits[0],
                valmax=limits[1],
                valinit=initial_val,
            )
            sliders.append(slider)

    def update(val):
        # 1. Read all 18 slider values (in degrees)
        all_angles_deg = np.zeros((6, 3))
        for i in range(6):
            for j in range(3):
                all_angles_deg[i, j] = sliders[i * 3 + j].val
        
        # 2. Convert to radians
        all_angles_rad = np.deg2rad(all_angles_deg).tolist()

        # 3. Use body_fk to get world coordinates for all legs
        # For this test, body translation and rotation are zero.
        all_leg_points = fk_calculator.body_fk(all_angles_rad)

        # 4. Update the plot
        for i in range(6):
            leg_points = all_leg_points[i]
            lines[i].set_data(leg_points[:, 0], leg_points[:, 1])
            lines[i].set_3d_properties(leg_points[:, 2])

        fig.canvas.draw_idle()

    # Attach the update function to all sliders
    for slider in sliders:
        slider.on_changed(update)

    update(0) # Initial call to draw the robot
    plt.show()

if __name__ == '__main__':
    test_body_fk_interactive()