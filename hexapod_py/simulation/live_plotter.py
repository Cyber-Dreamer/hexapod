"""
Live 3D Plotter for Hexapod Simulation
=======================================

This module provides a class to create and update a live 3D plot of the
hexapod's kinematic state using Matplotlib. It's designed to run in a
non-blocking way alongside the main simulation loop.
"""

import matplotlib.pyplot as plt
import numpy as np

class LivePlotter:
    def __init__(self, hip_positions):
        """
        Initializes the live 3D plot.

        :param hip_positions: A list of 6 (x, y, z) tuples for the hip joint locations.
        """
        plt.ion()  # Turn on interactive mode for non-blocking plotting

        self.fig = plt.figure(figsize=(8, 7))
        
        # Use a dark theme
        plt.style.use('dark_background')
        dark_grey = '#2E2E2E'
        self.fig.patch.set_facecolor(dark_grey)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor(dark_grey)

        self.leg_colors = ['C0', 'C1', 'C2', 'C3', 'C4', 'C5']
        # Create line objects for each of the 6 legs
        self.lines = [self.ax.plot([], [], [], 'o-', markersize=3, color=self.leg_colors[i])[0] for i in range(6)]

        # Draw the body outline by connecting the hip positions
        body_outline_points = np.array(hip_positions)
        body_plot_points = np.vstack([body_outline_points[[0, 1, 2, 3, 4, 5, 0]]])
        self.body_line, = self.ax.plot(body_plot_points[:, 0], body_plot_points[:, 1], body_plot_points[:, 2], '-', color='gray')

        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('Kinematic Gait Visualization')

        # Set fixed plot limits to prevent auto-scaling
        self.ax.set_xlim([-500, 500])
        self.ax.set_ylim([-500, 500])
        self.ax.set_zlim([-350, 150])
        self.ax.view_init(elev=20., azim=-75)
        self.ax.set_aspect('equal')

        self.fig.show()

    def update(self, all_leg_points):
        """
        Updates the plot with new leg joint positions.

        :param all_leg_points: A list of 6 arrays, where each array contains the
                               3D coordinates of the joints for one leg.
        """
        for i in range(6):
            leg_points = all_leg_points[i]
            if leg_points is not None and leg_points.shape[0] > 1:
                # Update the data for the line plot of each leg
                self.lines[i].set_data(leg_points[:, 0], leg_points[:, 1])
                self.lines[i].set_3d_properties(leg_points[:, 2])

        # Redraw the canvas
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()