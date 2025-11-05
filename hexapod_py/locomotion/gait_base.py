import numpy as np

class Gait:
    """
    Abstract base class for hexapod gaits.
    """
    def __init__(self, kinematics, step_height, knee_direction=-1):
        """
        Initializes the Gait object.

        :param kinematics: The HexapodKinematics object.
        :param step_height: The height of a step.
        """
        self.knee_direction = knee_direction
        self.kinematics = kinematics
        self.step_height = step_height
        self.gait_phase = 0.0

    def run(self, vx, vy, omega, roll, pitch, speed, default_foot_positions, last_known_angles, body_height, step_height):
        raise NotImplementedError("The 'run' method must be implemented by the gait subclass.")

    def _calculate_leg_ik(self, leg_idx, phase, vx, vy, omega, roll, pitch, default_foot_positions, last_known_angles, max_step_length, body_height, step_height):
        # vx, vy are now target linear velocities (mm/s)
        # omega is now target angular velocity (rad/s)
        
        # --- Dynamic Step Length Calculation ---
        # 1. Determine the maximum possible step length based on geometry.
        # The standoff distance is the radius from the body center to the default foot position.
        # A safe maximum step length is a fraction of this standoff distance to avoid collisions.
        standoff_distance = np.linalg.norm(default_foot_positions[leg_idx][:2])
        
        # Use a factor to keep a safety margin, as real legs have thickness.
        # 0.5 means the step length can be up to 50% of the standoff distance.
        step_range_factor = 0.5 
        max_dynamic_step = standoff_distance * step_range_factor
        
        # 2. Calculate the commanded step based on velocity inputs.
        # The input vx, vy, omega are physical velocities (mm/s, rad/s). We need to
        # normalize them to a [-1, 1] range to determine the step size.
        # We assume a nominal max velocity for this normalization. A better approach
        # would be to pass the max velocities from the locomotion controller.
        # For now, let's assume max_linear_velocity=300 and max_angular_velocity=pi/2.
        vx_norm = np.clip(vx / 300.0, -1.0, 1.0)
        vy_norm = np.clip(vy / 300.0, -1.0, 1.0)
        omega_norm = np.clip(omega / (np.pi / 2.0), -1.0, 1.0) # Assuming max_angular_velocity is pi/2

        # Create normalized direction vectors for linear and rotational movement
        linear_dir = np.array([vx_norm, vy_norm, 0])
        hip_pos = self.kinematics.hip_positions[leg_idx]
        # The rotational vector needs to be normalized by the hip distance to be on the same scale as linear_dir
        hip_radius = np.linalg.norm(hip_pos[:2])
        if hip_radius < 1e-6: hip_radius = 1.0 # Avoid division by zero for a theoretical center leg
        rotational_dir = np.array([-hip_pos[1], hip_pos[0], 0]) / hip_radius * omega_norm
        
        # Combine the vectors and clip the total magnitude to 1.0.
        # This ensures that moving and turning at the same time doesn't create an impossibly large step.
        command_vec = linear_dir + rotational_dir
        if np.linalg.norm(command_vec) > 1.0:
            command_vec = command_vec / np.linalg.norm(command_vec)
        
        total_step = command_vec * max_dynamic_step
        
        # Swing phase (leg is in the air, moving to the start of the next step)
        if phase < 0.5:
            swing_phase = phase * 2
            # Parabolic trajectory for the foot lift (0 -> 1 -> 0)
            parabolic_lift = (1 - (2 * swing_phase - 1)**2)
            z_lift = -body_height + (step_height * parabolic_lift)

            # Foot moves from its rearmost point to its foremost point.
            swing_offset = total_step * (swing_phase - 0.5)

            target_pos = default_foot_positions[leg_idx] + swing_offset
            target_pos[2] = z_lift
        # Stance phase (leg is on the ground, pushing the body)
        else:
            stance_phase = (phase - 0.5) * 2
            # Foot moves from its foremost point to its rearmost point
            stance_offset = total_step * (0.5 - stance_phase)
 
            target_pos = default_foot_positions[leg_idx] + stance_offset

        # Apply body roll and pitch rotation
        # This adjusts the target foot position to induce body tilt.
        if abs(roll) > 0.001 or abs(pitch) > 0.001:
            hip_pos = self.kinematics.hip_positions[leg_idx]
            # Simplified rotation effect on Z height of the foot target
            # Positive roll (right side up) should lower the right legs (y<0) and raise left legs (y>0)
            roll_offset = -hip_pos[1] * np.tan(roll)
            # Positive pitch (nose up) should lower the front legs (x>0) and raise rear legs (x<0)
            pitch_offset = hip_pos[0] * np.tan(pitch)
            target_pos[2] += roll_offset + pitch_offset

        v_foot_body = target_pos - self.kinematics.hip_positions[leg_idx]
        # Rotate the vector from the body frame into the leg's local coordinate frame.
        # This is the same crucial step that was needed for body_ik to work correctly.
        hip_base_angle = self.kinematics.hip_base_angles[leg_idx]
        R_hip_inv = np.array([[ np.cos(hip_base_angle), np.sin(hip_base_angle), 0],
                              [-np.sin(hip_base_angle), np.cos(hip_base_angle), 0],
                              [0,                       0,                      1]])
        v_foot_local = R_hip_inv @ v_foot_body

        angles = self.kinematics.leg_ik(v_foot_local, knee_direction=self.knee_direction)

        if angles is None:
            # If the target is unreachable, fall back to the last known good angles for this leg.
            return last_known_angles[leg_idx]

        return angles
