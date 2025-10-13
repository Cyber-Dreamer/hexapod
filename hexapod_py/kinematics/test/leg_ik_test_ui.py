import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Add parent directory to path to import hexapod
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.kinematics.ik import HexapodKinematics # Assuming this is where your updated class is

def test_leg_ik_interactive_2d():
    """
    An interactive test for inverse kinematics of a single leg,
    displaying its 2D projection in the XZ and XY planes.
    """
    # Define leg lengths (example values, adjust as needed)
    L_COXA = 92.5      # hipJoint_to_femurJoint
    L_FEMUR = 191.8    # femurJoint_to_tibiaJoint
    L_TIBIA = 284.969  # tibiaJoint_to_tipFoot
    
    Limit_COXA = 90
    Limit_FEMUR = 110
    Limit_TIBIA = 120
    
    segment_lengths = [L_COXA, L_FEMUR, L_TIBIA]
    
    joint_limits_rad = [np.deg2rad(Limit_COXA), np.deg2rad(Limit_FEMUR), np.deg2rad(Limit_TIBIA)]


    # Dummy hip position (not used for single leg IK, but required by HexapodKinematics)
    hip_positions = np.array([[0, 0, 0]] * 6)
    kinematics = HexapodKinematics(segment_lengths=segment_lengths, hip_positions=hip_positions, joint_limits=joint_limits_rad)

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

    # Start with the leg straight out (0, 0, 0 angles)
    initial_x = L_COXA + L_FEMUR + L_TIBIA
    initial_y = 0.0
    initial_z = 0.0
    initial_target_point = np.array([initial_x, initial_y, initial_z])

    # --- Helper function to get joint positions from angles ---
    def get_joint_positions(angles):

        gamma = angles[0]
        alpha = angles[1]
        beta = angles[2]

        # --- Forward Kinematics Calculation ---
        # This logic calculates the 3D position of each joint based on the angles.
        # It should mirror the logic in the HexapodForwardKinematics class.
        
        # p0: Coxa joint at the origin of the leg's frame
        p0 = np.array([0, 0, 0])

        # p1: Femur joint position. This is a simple 2D rotation on the XY plane.
        p1 = np.array([L_COXA * np.cos(gamma), L_COXA * np.sin(gamma), 0])

        # To calculate p2 and p3, we first find their positions in the leg's 2D plane
        # (as if gamma were 0), and then rotate them by gamma around the Z-axis.
        
        # Vector from femur joint to tibia joint in the leg's 2D plane
        femur_vec_local = np.array([L_FEMUR * np.cos(alpha), 0, L_FEMUR * np.sin(alpha)])
        # Vector from tibia joint to foot tip in the leg's 2D plane
        tibia_vec_local = np.array([L_TIBIA * np.cos(alpha + beta), 0, L_TIBIA * np.sin(alpha + beta)])

        # p2: Tibia joint position.
        p2 = p1 + np.array([femur_vec_local[0] * np.cos(gamma), femur_vec_local[0] * np.sin(gamma), femur_vec_local[2]])
        # p3: Foot tip position.
        p3 = p2 + np.array([tibia_vec_local[0] * np.cos(gamma), tibia_vec_local[0] * np.sin(gamma), tibia_vec_local[2]])

        return np.array([p0, p1, p2, p3])

    def calculate_workspace_limits(l_coxa, l_femur, l_tibia):
        """
        Calculates the absolute min/max for X, Y, and Z coordinates based on
        joint lengths and limits. This defines the bounding box of the workspace.
        """
        # Use the joint limits defined inside the IK function
        gamma_limit = np.deg2rad(90)
        alpha_limit = np.deg2rad(110)
        beta_limit = np.deg2rad(120)

        # Generate a grid of angles at their limits
        gamma_angles = np.linspace(-gamma_limit, gamma_limit, 10)
        alpha_angles = np.linspace(-alpha_limit, alpha_limit, 20)
        beta_angles = np.linspace(-beta_limit, beta_limit, 20)

        # Create a meshgrid of all angle combinations
        g, a, b = np.meshgrid(gamma_angles, alpha_angles, beta_angles, indexing='ij')

        # Calculate foot tip positions for all these angle combinations using FK
        x_coords = (l_coxa * np.cos(g) +
                    l_femur * np.cos(a) * np.cos(g) +
                    l_tibia * np.cos(a + b) * np.cos(g))

        y_coords = (l_coxa * np.sin(g) +
                    l_femur * np.cos(a) * np.sin(g) +
                    l_tibia * np.cos(a + b) * np.sin(g))

        z_coords = (l_femur * np.sin(a) +
                    l_tibia * np.sin(a + b))

        # Find the min and max of the valid points
        # The IK function has a reachability check, so we filter out points that are impossible
        # even if the angles are within limits (e.g., fully folded back on itself).
        l_horizontal = np.sqrt(x_coords**2 + y_coords**2) - l_coxa
        l_hyp = np.sqrt(l_horizontal**2 + z_coords**2) 
        # Use np.isclose for the upper bound to handle floating point inaccuracies at full extension.
        # This ensures the "straight leg" position is included in the workspace.
        valid_mask = (l_hyp <= (l_femur + l_tibia) + 1e-6) & (l_hyp >= abs(l_femur - l_tibia) - 1e-6)

        # The theoretical maximum X reach is the sum of all segment lengths.
        x_max_theoretical = l_coxa + l_femur + l_tibia

        return {
            'x': (np.min(x_coords[valid_mask]), x_max_theoretical),
            'y': (np.min(y_coords[valid_mask]), np.max(y_coords[valid_mask]) + 1), # Add 1mm buffer
            'z': (np.min(z_coords[valid_mask]), np.max(z_coords[valid_mask])),
        }

    # --- Plot initial state ---
    angles = kinematics.leg_ik(initial_target_point)
    leg_points = get_joint_positions(angles)

    # Create plot objects for each segment and view
    def create_leg_plots(ax, label_prefix=""):
        coxa, = ax.plot([], [], 'o-', color='red', label=f'{label_prefix}Coxa')
        femur, = ax.plot([], [], 'o-', color='green', label=f'{label_prefix}Femur')
        tibia, = ax.plot([], [], 'o-', color='blue', label=f'{label_prefix}Tibia')
        target, = ax.plot([], [], 'rx', markersize=10, label=f'{label_prefix}Target')
        return {'coxa': coxa, 'femur': femur, 'tibia': tibia, 'target': target}

    plots_xz = create_leg_plots(ax_xz)
    plots_xy = create_leg_plots(ax_xy)

    # XZ Plane Plot
    ax_xz.set_xlabel('X (mm)')
    ax_xz.set_ylabel('Z (mm) - Height')
    ax_xz.set_title('Side View (Z vs X)')
    ax_xz.set_aspect('equal')
    ax_xz.grid(True)
    # Set fixed plot limits
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
    # Set fixed plot limits
    ax_xy.set_xlim(-400, 400)
    ax_xy.set_ylim(-200, 600)
    ax_xy.invert_xaxis() # Makes positive Y (depth) go "into" the screen

    # --- Text for displaying joint angles ---
    angle_text = fig.text(0.82, 0.85, "Angles: --", verticalalignment='top', fontfamily='monospace')

    # --- Sliders ---
    ax_slider_x = plt.axes([0.25, 0.20, 0.5, 0.03])
    ax_slider_y = plt.axes([0.25, 0.15, 0.5, 0.03])
    ax_slider_z = plt.axes([0.25, 0.10, 0.5, 0.03])

    for ax_s in [ax_slider_x, ax_slider_y, ax_slider_z]:
        ax_s.set_facecolor(dark_grey)

    # Calculate the workspace and set slider limits accordingly
    workspace_limits = calculate_workspace_limits(L_COXA, L_FEMUR, L_TIBIA)

    slider_x = Slider(ax_slider_x, 'Target X', workspace_limits['x'][0], workspace_limits['x'][1], valinit=initial_x, valstep=1)
    slider_y = Slider(ax_slider_y, 'Target Y', workspace_limits['y'][0], workspace_limits['y'][1], valinit=initial_y, valstep=1)
    slider_z = Slider(ax_slider_z, 'Target Z', workspace_limits['z'][0], workspace_limits['z'][1], valinit=initial_z, valstep=1)

    # Store the last known valid slider values
    last_valid_values = {'x': initial_x, 'y': initial_y, 'z': initial_z}

    # Flag to prevent recursive updates when resetting sliders to a valid state.
    # This is a workaround for older matplotlib versions that don't support
    # the 'sendevent=False' argument in slider.set_val().
    _is_programmatic_update = False

    def update(val):
        nonlocal _is_programmatic_update
        if _is_programmatic_update:
            return

        target_point = np.array([slider_x.val, slider_y.val, slider_z.val])
        
        # Calculate IK
        angles = kinematics.leg_ik(target_point)
        
        if angles is not None:
            leg_points = get_joint_positions(angles)
            # Current position is valid, so we update our stored last valid values
            last_valid_values['x'] = slider_x.val
            last_valid_values['y'] = slider_y.val
            last_valid_values['z'] = slider_z.val
            p0, p1, p2, p3 = leg_points
            
            # Update XZ plot (Side View)
            plots_xz['coxa'].set_data([p0[0], p1[0]], [p0[2], p1[2]])
            plots_xz['femur'].set_data([p1[0], p2[0]], [p1[2], p2[2]])
            plots_xz['tibia'].set_data([p2[0], p3[0]], [p2[2], p3[2]])
            plots_xz['target'].set_data([target_point[0]], [target_point[2]])

            # Update YX plot (Top-Down View)
            plots_xy['coxa'].set_data([p0[1], p1[1]], [p0[0], p1[0]])
            plots_xy['femur'].set_data([p1[1], p2[1]], [p1[0], p2[0]])
            plots_xy['tibia'].set_data([p2[1], p3[1]], [p2[0], p3[0]])
            plots_xy['target'].set_data([target_point[1]], [target_point[0]])

            # Update angle text
            gamma, alpha, beta = np.rad2deg(angles)
            angle_text.set_text(f"Angles (deg):\nγ: {gamma:6.1f}\nα: {alpha:6.1f}\nβ: {beta:6.1f}")
        else:
            # Target is unreachable, revert the sliders to the last valid position
            # Set a flag to prevent this block from re-triggering the update function.
            _is_programmatic_update = True
            slider_x.set_val(last_valid_values['x'])
            slider_y.set_val(last_valid_values['y'])
            slider_z.set_val(last_valid_values['z'])
            _is_programmatic_update = False # Reset the flag

            angle_text.set_text("Angles: Unreachable")
            # The plot doesn't need to be cleared, as it will remain in the last valid state.

        fig.canvas.draw_idle()

    slider_x.on_changed(update)
    slider_y.on_changed(update)
    slider_z.on_changed(update)

    update(0) # Initial call to set limits
    plt.show()

if __name__ == '__main__':
    test_leg_ik_interactive_2d()