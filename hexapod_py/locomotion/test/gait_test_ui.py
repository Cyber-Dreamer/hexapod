import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons, Button
from matplotlib.animation import FuncAnimation

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hexapod_py.locomotion.locomotion import HexapodLocomotion
from hexapod_py.kinematics.fk import HexapodForwardKinematics

def test_gait_interactive():
    """
    An interactive test for hexapod gaits. Sliders control the robot's
    velocity and rotation, and an animated 3D plot displays the walking motion.
    """
    # --- Initialization ---
    locomotion = HexapodLocomotion(gait_type='tripod', body_height=200)
    kinematics = locomotion.kinematics
    
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
    plt.subplots_adjust(left=0.1, right=0.8, bottom=0.35)

    # --- Plotting Setup ---
    leg_colors = ['C0', 'C1', 'C2', 'C3', 'C4', 'C5']
    lines = [ax.plot([], [], [], 'o-', markersize=4, color=leg_colors[i])[0] for i in range(6)]
    
    # Add text annotations for leg numbers
    leg_texts = [ax.text(0, 0, 0, str(i), color=leg_colors[i], fontsize=10, ha='center', va='center') for i in range(6)]

    ax.set_xlabel('World X (mm)')
    ax.set_ylabel('World Y (mm)')
    ax.set_zlabel('World Z (mm)')
    ax.set_title('Hexapod Gait Test')
    
    ax.set_xlim([-500, 500])
    ax.set_ylim([-500, 500])
    ax.set_zlim([-350, 150])
    ax.view_init(elev=20., azim=-75)
    ax.set_aspect('equal')

    # Draw the body outline
    body_line, = ax.plot([], [], [], '-', color='gray', label='Body')

    # --- Text for displaying gait info ---
    info_text = fig.text(0.82, 0.95, "", verticalalignment='top', fontfamily='monospace')
    # --- Text for displaying joint angles ---
    angle_text_str = "Joint Angles (deg):\n" + "\n".join([f"Leg {i}: --" for i in range(6)])
    angle_text = fig.text(0.82, 0.55, angle_text_str, verticalalignment='top', fontfamily='monospace')

    # --- Sliders ---
    slider_axes = {
        'vx': plt.axes([0.25, 0.30, 0.5, 0.02]),
        'vy': plt.axes([0.25, 0.27, 0.5, 0.02]),
        'omega': plt.axes([0.25, 0.24, 0.5, 0.02]),
        'roll': plt.axes([0.25, 0.21, 0.5, 0.02]),
        'pitch': plt.axes([0.25, 0.18, 0.5, 0.02]),        'step_h': plt.axes([0.25, 0.15, 0.5, 0.02]),
        'standoff': plt.axes([0.25, 0.12, 0.5, 0.02]),
        'body_h': plt.axes([0.25, 0.09, 0.5, 0.02])
    }
    for ax_s in slider_axes.values():
        ax_s.set_facecolor(dark_grey)
        
    sliders = {
        'vx': Slider(slider_axes['vx'], 'Vx (fwd/bwd)', -1.0, 1.0, valinit=0),
        'vy': Slider(slider_axes['vy'], 'Vy (strafe)', -1.0, 1.0, valinit=0),
        'omega': Slider(slider_axes['omega'], 'Omega (turn)', -1.0, 1.0, valinit=0),
        'roll': Slider(slider_axes['roll'], 'Roll', -np.pi/8, np.pi/8, valinit=0),
        'pitch': Slider(slider_axes['pitch'], 'Pitch', -np.pi/8, np.pi/8, valinit=0),
        'step_h': Slider(slider_axes['step_h'], 'Step Height', 10, 80, valinit=40),
        'standoff': Slider(slider_axes['standoff'], 'Standoff', 200, 500, valinit=locomotion.standoff_distance),
        'body_h': Slider(slider_axes['body_h'], 'Body Height', 100, 300, valinit=locomotion.body_height),
    }

    # --- Radio Buttons for Gait Selection ---
    ax_radio = plt.axes([0.05, 0.05, 0.15, 0.2], facecolor=dark_grey)
    gait_options = list(locomotion.available_gaits.keys())
    radio_buttons = RadioButtons(ax_radio, gait_options, active=gait_options.index(locomotion.gait_type))

    def select_gait(label):
        """Callback function to switch the gait."""
        locomotion.set_gait(label)
        fig.canvas.draw_idle()

    radio_buttons.on_clicked(select_gait)

    # --- Stance/Height Update Callback ---
    def update_stance_params(val):
        """Callback to update stance-related parameters only when their sliders change."""
        standoff = sliders['standoff'].val
        body_height = sliders['body_h'].val
        locomotion.body_height = body_height
        # Only recalculate stance when these specific sliders are adjusted
        locomotion.recalculate_stance(standoff_distance=standoff)

    # --- Reset Button ---
    ax_reset = plt.axes([0.05, 0.02, 0.15, 0.04], facecolor=dark_grey)
    reset_button = Button(ax_reset, 'Reset Sliders', color=dark_grey, hovercolor='0.5')

    def reset_sliders(event):
        """Callback to reset all sliders to their initial values."""
        for slider in sliders.values():
            slider.set_val(slider.valinit)
        fig.canvas.draw_idle()

    reset_button.on_clicked(reset_sliders)

    # --- Pause Button ---
    ax_pause = plt.axes([0.82, 0.02, 0.15, 0.04], facecolor=dark_grey)
    pause_button = Button(ax_pause, 'Pause', color=dark_grey, hovercolor='0.5')
    is_paused = False

    def toggle_pause(event):
        nonlocal is_paused
        if is_paused:
            ani.resume()
            pause_button.label.set_text('Pause')
        else:
            ani.pause()
            pause_button.label.set_text('Resume')
        is_paused = not is_paused
        fig.canvas.draw_idle()

    # --- Radio Buttons for Knee Direction ---
    ax_knee_radio = plt.axes([0.82, 0.6, 0.15, 0.15], facecolor=dark_grey)
    knee_radio = RadioButtons(ax_knee_radio, ('Knee Down (-1)', 'Knee Up (+1)'), active=0)
    knee_direction_map = {'Knee Down (-1)': -1, 'Knee Up (+1)': 1}

    def select_knee_direction(label):
        """Callback to set knee direction."""
        locomotion.knee_direction = knee_direction_map[label]
        # Also update the knee direction on all available gait objects
        for gait in locomotion.available_gaits.values():
            gait.knee_direction = locomotion.knee_direction

    knee_radio.on_clicked(select_knee_direction)
    
    # --- Animation Setup ---
    # The FuncAnimation object needs to be defined before we can connect the pause button.
    # We will define it as None first and then assign it.
    ani = None

    # --- Animation Update Function ---
    def update(frame):
        # 1. Read values from sliders
        vx = sliders['vx'].val
        vy = sliders['vy'].val
        omega = sliders['omega'].val
        roll = sliders['roll'].val
        pitch = sliders['pitch'].val
        step_height = sliders['step_h'].val # Step height can change mid-gait

        # Normalize the direction vector if it's greater than 1
        dir_vec = np.array([vx, vy])
        if np.linalg.norm(dir_vec) > 1.0:
            dir_vec = dir_vec / np.linalg.norm(dir_vec)
            vx, vy = dir_vec

        # 2. Get joint angles by running the locomotion controller.
        # The controller's internal state machine will handle transitions between walking and standing.
        all_angles = locomotion.run_gait(vx, vy, omega, roll=roll, pitch=pitch, step_height=step_height)

        # 3. Use body_fk to get world coordinates for all legs
        # The body rotation is applied to visualize roll and pitch.
        # Body translation is kept at zero as the world frame moves with the robot.
        body_rotation = np.array([roll, pitch, 0.0])
        all_leg_points = fk_calculator.body_fk(all_angles, body_rotation=body_rotation)

        # 4. Update the plot
        for i in range(6):
            leg_points = all_leg_points[i]
            if leg_points.shape[0] > 1:
                lines[i].set_data(leg_points[:, 0], leg_points[:, 1])
                lines[i].set_3d_properties(leg_points[:, 2])
                # Update leg number text position to be near the hip joint
                hip_pos = leg_points[0]
                leg_texts[i].set_position((hip_pos[0], hip_pos[1]))
                leg_texts[i].set_z(hip_pos[2] + 15) # Offset slightly above the joint
            else: # Unreachable
                lines[i].set_data([], [])
                lines[i].set_3d_properties([])

        # Update the body line to reflect the new orientation
        # We get the hip positions from the FK calculation for the body rotation
        rotated_hips_list = fk_calculator.body_fk([], body_rotation=body_rotation)
        rotated_hips = np.array([p[0] for p in rotated_hips_list])
        body_plot_points = np.vstack([rotated_hips[[0,1,2,3,4,5,0]]])
        body_line.set_data(body_plot_points[:, 0], body_plot_points[:, 1])
        body_line.set_3d_properties(body_plot_points[:, 2])
        # Update angle text display
        angle_strings = []
        for i, angles in enumerate(all_angles):
            if angles is not None:
                angle_strings.append(f"L{i}: {np.rad2deg(angles)[0]:6.1f}, {np.rad2deg(angles)[1]:6.1f}, {np.rad2deg(angles)[2]:6.1f}")
            else:
                angle_strings.append(f"L{i}: Unreachable")
        angle_text.set_text("Joint Angles (deg):\n" + "\n".join(angle_strings))


        # Update info text
        info_text.set_text(
            f"Gait: {locomotion.gait_type.capitalize()}\n"
            f"State: {locomotion.locomotion_state}\n"
            f"Vx: {vx:.2f}\n"
            f"Vy: {vy:.2f}\n"
            f"Omega: {omega:.2f}\n"
            f"Roll: {np.rad2deg(roll):.1f}°\n"
            f"Pitch: {np.rad2deg(pitch):.1f}°"
        )

        return lines + leg_texts + [body_line, info_text, angle_text] # Add body_line to returned artists

    # Create and start the animation
    ani = FuncAnimation(fig, update, frames=None, blit=False, interval=30, repeat=True)

    # Now that 'ani' is created, connect the callbacks that need it
    pause_button.on_clicked(toggle_pause)
    sliders['standoff'].on_changed(update_stance_params)
    sliders['body_h'].on_changed(update_stance_params)

    # Initial call to set the stance from slider defaults
    update_stance_params(None)

    plt.show()

if __name__ == '__main__':
    test_gait_interactive()
