import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Add parent directory to path to import hexapod
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

def test_leg_fk_interactive_2d():
    """
    An interactive test for forward kinematics of a single leg. Sliders control
    the joint angles (gamma, alpha, beta) and the plots display the resulting
    leg posture and foot tip coordinates.
    """
    # Define leg lengths
    L_COXA = 92.5      # hipJoint_to_femurJoint
    L_FEMUR = 191.8    # femurJoint_to_tibiaJoint
    L_TIBIA = 284.969  # tibiaJoint_to_tipFoot
    
    Limit_COXA = 90
    Limit_FEMUR = 110
    Limit_TIBIA = 120


    fig = plt.figure(figsize=(14, 7))

    # Use a dark theme with a custom dark grey background
    plt.style.use('dark_background')
    dark_grey = '#2E2E2E'
    fig.patch.set_facecolor(dark_grey)
    ax_xz = fig.add_subplot(121) # XZ plane view
    ax_xz.set_facecolor(dark_grey)
    ax_xy = fig.add_subplot(122) # XY plane view
    ax_xy.set_facecolor(dark_grey)
    plt.subplots_adjust(left=0.1, right=0.8, bottom=0.35, wspace=0.3)

    # Initial angles (all zero)
    initial_gamma_deg = 0.0
    initial_alpha_deg = 0.0
    initial_beta_deg = 0.0

    # --- Helper function to get joint positions from angles (Forward Kinematics) ---
    def get_joint_positions(angles_rad):
        gamma, alpha, beta = angles_rad

        # 1. Start with the origin (coxa pivot)
        p0 = np.array([0, 0, 0])

        # 2. Calculate the position of the femur joint.
        p1 = np.array([L_COXA * np.cos(gamma), L_COXA * np.sin(gamma), 0])

        # 3. Calculate the position of the tibia joint.
        p2 = p1 + np.array([L_FEMUR * np.cos(alpha) * np.cos(gamma),
                            L_FEMUR * np.cos(alpha) * np.sin(gamma),
                            L_FEMUR * np.sin(alpha)])

        # 4. Calculate the position of the foot tip.
        p3 = p2 + np.array([L_TIBIA * np.cos(alpha + beta) * np.cos(gamma),
                            L_TIBIA * np.cos(alpha + beta) * np.sin(gamma),
                            L_TIBIA * np.sin(alpha + beta)])

        return np.array([p0, p1, p2, p3])

    # --- Plot initial state ---
    initial_angles_rad = np.deg2rad([initial_gamma_deg, initial_alpha_deg, initial_beta_deg])
    leg_points = get_joint_positions(initial_angles_rad)

    # Create plot objects for each segment and view
    def create_leg_plots(ax, label_prefix=""):
        coxa, = ax.plot([], [], 'o-', color='red', label=f'{label_prefix}Coxa')
        femur, = ax.plot([], [], 'o-', color='green', label=f'{label_prefix}Femur')
        tibia, = ax.plot([], [], 'o-', color='blue', label=f'{label_prefix}Tibia')
        return {'coxa': coxa, 'femur': femur, 'tibia': tibia}

    plots_xz = create_leg_plots(ax_xz)
    plots_xy = create_leg_plots(ax_xy)

    # XZ Plane Plot
    ax_xz.set_xlabel('X (mm)')
    ax_xz.set_ylabel('Z (mm) - Height')
    ax_xz.set_title('Side View (Z vs X)')
    ax_xz.set_aspect('equal')
    ax_xz.grid(True)
    ax_xz.set_xlim(-200, 600)
    ax_xz.set_ylim(-500, 500)

    # Use a single legend for both plots
    handles, labels = ax_xz.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right', bbox_to_anchor=(0.99, 0.95))

    # YX Plane Plot (Top-Down)
    ax_xy.set_xlabel('Y (mm) - Depth')
    ax_xy.set_ylabel('X (mm) - Side')
    ax_xy.set_title('Top-Down View (X vs Y)')
    ax_xy.set_aspect('equal')
    ax_xy.grid(True)
    ax_xy.set_xlim(-400, 400)
    ax_xy.set_ylim(-200, 600)
    ax_xy.invert_xaxis()

    # --- Text for displaying foot tip coordinates ---
    coord_text = fig.text(0.82, 0.85, "Coords: --", verticalalignment='top', fontfamily='monospace')

    # --- Sliders for Joint Angles ---
    ax_slider_gamma = plt.axes([0.25, 0.20, 0.5, 0.03])
    ax_slider_alpha = plt.axes([0.25, 0.15, 0.5, 0.03])
    ax_slider_beta = plt.axes([0.25, 0.10, 0.5, 0.03])

    for ax_s in [ax_slider_gamma, ax_slider_alpha, ax_slider_beta]:
        ax_s.set_facecolor(dark_grey)

    slider_gamma = Slider(ax_slider_gamma, 'γ (Coxa)', -90, 90, valinit=initial_gamma_deg)
    slider_alpha = Slider(ax_slider_alpha, 'α (Femur)', -110, 110, valinit=initial_alpha_deg)
    slider_beta = Slider(ax_slider_beta, 'β (Tibia)', -120, 120, valinit=initial_beta_deg)

    def update(val):
        # Read angles from sliders in degrees and convert to radians
        angles_deg = np.array([slider_gamma.val, slider_alpha.val, slider_beta.val])
        angles_rad = np.deg2rad(angles_deg)
        
        # Calculate FK
        leg_points = get_joint_positions(angles_rad)
        p0, p1, p2, p3 = leg_points
        
        # Update XZ plot (Side View)
        plots_xz['coxa'].set_data([p0[0], p1[0]], [p0[2], p1[2]])
        plots_xz['femur'].set_data([p1[0], p2[0]], [p1[2], p2[2]])
        plots_xz['tibia'].set_data([p2[0], p3[0]], [p2[2], p3[2]])

        # Update YX plot (Top-Down View)
        plots_xy['coxa'].set_data([p0[1], p1[1]], [p0[0], p1[0]])
        plots_xy['femur'].set_data([p1[1], p2[1]], [p1[0], p2[0]])
        plots_xy['tibia'].set_data([p2[1], p3[1]], [p2[0], p3[0]])

        # Update coordinate text display
        foot_x, foot_y, foot_z = p3
        coord_text.set_text(f"Coords (mm):\nX: {foot_x:6.1f}\nY: {foot_y:6.1f}\nZ: {foot_z:6.1f}")

        fig.canvas.draw_idle()

    slider_gamma.on_changed(update)
    slider_alpha.on_changed(update)
    slider_beta.on_changed(update)

    update(0) # Initial call to draw the leg
    plt.show()

if __name__ == '__main__':
    test_leg_fk_interactive_2d()