import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons
from matplotlib.animation import FuncAnimation

# Add parent directory to path to import hexapod
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
        leg_lengths=kinematics.leg_lengths,
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
    
    # Draw the body outline
    body_outline_points = np.array(kinematics.hip_positions)
    # Connect hips in order to draw the body chassis
    body_plot_points = np.vstack([body_outline_points[[0,1,2,3,4,5,0]]])
    body_line, = ax.plot(body_plot_points[:, 0], body_plot_points[:, 1], body_plot_points[:, 2], '-', color='gray')

    ax.set_xlabel('World X (mm)')
    ax.set_ylabel('World Y (mm)')
    ax.set_zlabel('World Z (mm)')
    ax.set_title('Hexapod Gait Test')
    
    ax.set_xlim([-500, 500])
    ax.set_ylim([-500, 500])
    ax.set_zlim([-350, 150])
    ax.view_init(elev=20., azim=-75)
    ax.set_aspect('equal')

    # --- Text for displaying gait info ---
    info_text = fig.text(0.82, 0.95, "", verticalalignment='top', fontfamily='monospace')

    # --- Sliders ---
    slider_axes = {
        'vx': plt.axes([0.25, 0.30, 0.5, 0.02]),
        'vy': plt.axes([0.25, 0.27, 0.5, 0.02]),
        'omega': plt.axes([0.25, 0.24, 0.5, 0.02]),
        'roll': plt.axes([0.25, 0.21, 0.5, 0.02]),
        'pitch': plt.axes([0.25, 0.18, 0.5, 0.02]),
        'speed': plt.axes([0.25, 0.15, 0.5, 0.02]),
        'step_h': plt.axes([0.25, 0.12, 0.5, 0.02]),
        'standoff': plt.axes([0.25, 0.09, 0.5, 0.02])
    }
    for ax_s in slider_axes.values():
        ax_s.set_facecolor(dark_grey)
        
    sliders = {
        'vx': Slider(slider_axes['vx'], 'Vx (fwd/bwd)', -1.0, 1.0, valinit=0),
        'vy': Slider(slider_axes['vy'], 'Vy (strafe)', -1.0, 1.0, valinit=0),
        'omega': Slider(slider_axes['omega'], 'Omega (turn)', -1.0, 1.0, valinit=0),
        'roll': Slider(slider_axes['roll'], 'Roll', -np.pi/8, np.pi/8, valinit=0),
        'pitch': Slider(slider_axes['pitch'], 'Pitch', -np.pi/8, np.pi/8, valinit=0),
        'speed': Slider(slider_axes['speed'], 'Gait Speed', 0.005, 0.05, valinit=0.006),
        'step_h': Slider(slider_axes['step_h'], 'Step Height', 10, 80, valinit=40),
        'standoff': Slider(slider_axes['standoff'], 'Standoff', 200, 500, valinit=locomotion.standoff_distance),
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

    # --- Animation Update Function ---
    def update(frame):
        # 1. Read values from sliders
        vx = sliders['vx'].val
        vy = sliders['vy'].val
        omega = sliders['omega'].val
        roll = sliders['roll'].val
        pitch = sliders['pitch'].val
        speed = sliders['speed'].val
        step_height = sliders['step_h'].val
        standoff = sliders['standoff'].val

        # Normalize the direction vector if it's greater than 1
        dir_vec = np.array([vx, vy])
        if np.linalg.norm(dir_vec) > 1.0:
            dir_vec = dir_vec / np.linalg.norm(dir_vec)
            vx, vy = dir_vec

        # Update stance based on standoff slider
        locomotion.recalculate_stance(standoff_distance=standoff)

        # 2. Run the gait logic to get joint angles
        all_angles = locomotion.run_gait(vx, vy, omega, roll=roll, pitch=pitch, speed=speed, step_height=step_height)

        # 3. Use body_fk to get world coordinates for all legs
        # For gait, the body itself is considered static at the origin.
        # The legs move relative to this origin to create motion.
        all_leg_points = fk_calculator.body_fk(all_angles)

        # 4. Update the plot
        for i in range(6):
            leg_points = all_leg_points[i]
            if leg_points.shape[0] > 1:
                lines[i].set_data(leg_points[:, 0], leg_points[:, 1])
                lines[i].set_3d_properties(leg_points[:, 2])
            else: # Unreachable
                lines[i].set_data([], [])
                lines[i].set_3d_properties([])

        # Update info text
        info_text.set_text(
            f"Gait: {locomotion.gait_type.capitalize()}\n"
            f"Phase: {locomotion.current_gait.gait_phase:.2f}\n"
            f"Vx: {vx:.2f}\n"
            f"Vy: {vy:.2f}\n"
            f"Omega: {omega:.2f}\n"
            f"Roll: {np.rad2deg(roll):.1f}°\n"
            f"Pitch: {np.rad2deg(pitch):.1f}°"
        )

        return lines + [body_line, info_text]

    # Create and start the animation
    ani = FuncAnimation(fig, update, frames=None, blit=False, interval=30, repeat=True)

    plt.show()

if __name__ == '__main__':
    test_gait_interactive()
