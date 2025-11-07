import numpy as np

class Gait:
    """Abstract base class for hexapod gaits."""
    def __init__(self, kinematics, step_height, knee_direction=-1):
        """Initializes the Gait object."""
        self.knee_direction = knee_direction
        self.kinematics = kinematics
        self.step_height = step_height
        self.gait_phase = 0.0

    def run(self, vx, vy, omega, roll, pitch, speed, default_foot_positions, last_known_angles, body_height, step_height):
        raise NotImplementedError("The 'run' method must be implemented by the gait subclass.")

    def _calculate_leg_ik(self, leg_idx, phase, vx, vy, omega, roll, pitch, default_foot_positions, last_known_angles, body_height, step_height):
        # Calculate max step length based on leg's standoff distance for stability.
        standoff_distance = np.linalg.norm(default_foot_positions[leg_idx][:2])
        step_range_factor = 0.5
        max_dynamic_step = standoff_distance * step_range_factor

        # Normalize velocity inputs to get a direction vector for the step.
        vx_norm = np.clip(vx / 300.0, -1.0, 1.0)
        vy_norm = np.clip(vy / 300.0, -1.0, 1.0)
        omega_norm = np.clip(omega / (np.pi / 2.0), -1.0, 1.0)

        # Combine linear and rotational movement commands into a single step vector.
        linear_dir = np.array([vx_norm, vy_norm, 0])
        hip_pos = self.kinematics.hip_positions[leg_idx]
        hip_radius = np.linalg.norm(hip_pos[:2])
        if hip_radius < 1e-6: hip_radius = 1.0 # Avoid division by zero.
        rotational_dir = np.array([-hip_pos[1], hip_pos[0], 0]) / hip_radius * omega_norm

        # Clip the combined command vector to ensure total step size is not excessive.
        command_vec = linear_dir + rotational_dir
        if np.linalg.norm(command_vec) > 1.0:
            command_vec = command_vec / np.linalg.norm(command_vec)

        total_step = command_vec * max_dynamic_step

        # Swing phase: leg is in the air.
        if phase < 0.5:
            swing_phase = phase * 2
            # Use a parabolic trajectory for a smooth foot lift.
            parabolic_lift = (1 - (2 * swing_phase - 1)**2)
            z_lift = -body_height + (step_height * parabolic_lift)

            # The foot swings from its rearmost point to its foremost point.
            swing_offset = total_step * (swing_phase - 0.5)
            target_pos = default_foot_positions[leg_idx] + swing_offset
            target_pos[2] = z_lift
        # Stance phase: leg is on the ground, pushing the body.
        else:
            stance_phase = (phase - 0.5) * 2
            # The foot pushes from its foremost point to its rearmost point.
            stance_offset = total_step * (0.5 - stance_phase)
            target_pos = default_foot_positions[leg_idx] + stance_offset

        # Apply body roll and pitch by adjusting the target foot Z position.
        if abs(roll) > 0.001 or abs(pitch) > 0.001:
            hip_pos = self.kinematics.hip_positions[leg_idx]
            roll_offset = -hip_pos[1] * np.tan(roll)
            pitch_offset = hip_pos[0] * np.tan(pitch)
            target_pos[2] += roll_offset + pitch_offset

        # Transform the target position from the body frame to the leg's local frame.
        v_foot_body = target_pos - self.kinematics.hip_positions[leg_idx]
        hip_base_angle = self.kinematics.hip_base_angles[leg_idx]
        R_hip_inv = np.array([[ np.cos(hip_base_angle), np.sin(hip_base_angle), 0],
                              [-np.sin(hip_base_angle), np.cos(hip_base_angle), 0],
                              [0,                       0,                      1]])
        v_foot_local = R_hip_inv @ v_foot_body

        angles = self.kinematics.leg_ik(v_foot_local, knee_direction=self.knee_direction)

        if angles is None:
            # If the target is unreachable, use the last known good angles to prevent errors.
            return last_known_angles[leg_idx]

        return angles
