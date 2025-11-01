import numpy as np
from hexapod_py.kinematics.ik import HexapodKinematics
from .tripod_gait import TripodGait
from .ripple_gait import RippleGait

class HexapodLocomotion:
    def __init__(self, step_height=40, max_step_length=180, gait_type='tripod', body_height=350, standoff_distance=200, knee_direction=-1, gait_speed_factor=0.03, max_linear_velocity=300, max_angular_velocity=np.pi/2):
        
        # Measure of the joint in mm
        center_to_HipJoint = 152.024
        hipJoint_to_femurJoint = 92.5
        femurJoint_to_tibiaJoint = 191.8
        tibiaJoint_to_tipFoot = 284.969
        
        # Standard leg numbering:
        # New Clockwise Numbering:
        # 0: Rear-Left, 1: Middle-Left, 2: Front-Left
        # 3: Front-Right, 4: Middle-Right, 5: Rear-Right
        # Angles are measured counter-clockwise from the positive X-axis. The
        # order of angles must match the leg numbering scheme.
        self.hip_angles = np.deg2rad([210, 270, 330, 30, 90, 150])

        hip_positions = [
            tuple(coord) 
            for coord in np.column_stack(
                (center_to_HipJoint * np.cos(self.hip_angles),
                 center_to_HipJoint * np.sin(self.hip_angles),
                 np.zeros_like(self.hip_angles))).tolist()
        ]
        
        leg_lengths = [
            hipJoint_to_femurJoint,
            femurJoint_to_tibiaJoint,
            tibiaJoint_to_tipFoot
        ]
        


        self.kinematics = HexapodKinematics(segment_lengths=leg_lengths, hip_positions=hip_positions, joint_limits=np.deg2rad([90, 110, 120]))
        self.knee_direction = knee_direction
        self.available_gaits = {
            # max_step_length is now a fallback/limit, not the primary driver of step size
            'tripod': TripodGait(self.kinematics, step_height, max_step_length, self.knee_direction),
            'ripple': RippleGait(self.kinematics, step_height, max_step_length, self.knee_direction)
        }
        self.set_gait(gait_type)
        self.body_height = body_height
        self.step_height = step_height
        self.gait_speed_factor = gait_speed_factor
        self.max_linear_velocity = max_linear_velocity  # mm/s
        self.max_angular_velocity = max_angular_velocity # rad/s

        if standoff_distance is None:
            # To achieve a 90-degree angle at the knee, the horizontal distance
            # from the femur joint to the foot tip must be equal to the femur length.
            # The total horizontal distance (standoff) is this plus the coxa length.
            standoff_distance = self.kinematics.segment_lengths[0] + self.kinematics.segment_lengths[1]
        self.standoff_distance = standoff_distance
        self.recalculate_stance()

        # Initialize a cache for the last known valid joint angles for each leg.
        # This is crucial for handling unreachable IK targets gracefully.
        self.last_known_angles = list(self.default_joint_angles)
        
        # State management for smooth transitions
        self.locomotion_state = 'STANDING' # Can be 'STANDING', 'WALKING', 'SETTLING'
        self.settling_counter = 0
        self.settling_duration = 20 # Number of frames/ticks for the settling interpolation
        self.settling_start_angles = None

    def set_gait(self, gait_type):
        self.gait_type = gait_type
        self.current_gait = self.available_gaits.get(gait_type)
        if self.current_gait:
            self.current_gait.gait_phase = 0.0

    def set_body_pose(self, translation, rotation):
        """
        Calculates the leg joint angles required to achieve a given body pose.

        This is a wrapper around the body_ik function that uses the robot's
        default standing positions as the reference.
        """
        # 1. Calculate the target position for each foot in its local leg coordinate system.
        #    We pass self.default_foot_positions, which are the fixed world
        #    coordinates for the feet to stay planted on.
        new_foot_targets_local = self.kinematics.body_ik(translation, rotation, self.kinematics.hip_positions, self.default_foot_positions)

        # 2. For each leg, calculate the joint angles to reach its new target position.
        all_angles = []
        for i in range(6):
            angles = self.kinematics.leg_ik(new_foot_targets_local[i], knee_direction=self.knee_direction)
            if angles is not None:
                self.last_known_angles[i] = angles
                all_angles.append(angles)
            else:
                # If target is unreachable, use the last known good angles for this leg.
                all_angles.append(self.last_known_angles[i])

        return all_angles
    
    def run_gait(self, vx, vy, omega, roll=0.0, pitch=0.0, step_height=None):
        is_moving = any(abs(v) > 0.001 for v in [vx, vy, omega, roll, pitch])

        # --- State Machine for Locomotion ---
        if self.locomotion_state == 'STANDING':
            if is_moving:
                self.locomotion_state = 'WALKING'
            else:
                return self.default_joint_angles

        elif self.locomotion_state == 'WALKING':
            if not is_moving:
                # Check if gait is close to a stable point (phase 0 or 0.5)
                phase = self.current_gait.gait_phase
                if phase < 0.05 or abs(phase - 0.5) < 0.05:
                    self.locomotion_state = 'SETTLING'
                    self.settling_counter = 0
                    # Store the current leg angles to interpolate from
                    self.settling_start_angles = np.array(self.last_known_angles)
                # else, continue walking for a moment to reach a stable stop point
            # Continue walking if moving or if not at a good stopping point

        elif self.locomotion_state == 'SETTLING':
            if is_moving:
                self.locomotion_state = 'WALKING'
            else:
                if self.settling_counter >= self.settling_duration:
                    self.locomotion_state = 'STANDING'
                    self.current_gait.gait_phase = 0.0 # Reset phase for next walk
                    return self.default_joint_angles
                
                # Interpolate from the start of settling to the default stance
                t = self.settling_counter / self.settling_duration
                interpolated_angles = (1 - t) * self.settling_start_angles + t * np.array(self.default_joint_angles)
                
                self.settling_counter += 1
                
                # Update last known angles and return the interpolated ones
                self.last_known_angles = interpolated_angles.tolist()
                return self.last_known_angles

        if self.locomotion_state == 'STANDING':
            return self.default_joint_angles

        if self.current_gait:
            # Update gait parameters if new values are provided from the UI
            if step_height is not None:
                self.current_gait.step_height = step_height
    
            # Calculate the magnitude of the movement command
            # If we are walking but not moving, we need to continue the gait cycle to reach a stable stop.
            # We use the last known command magnitude to determine the speed for this final step.
            if is_moving:
                command_magnitude = max(np.linalg.norm([vx, vy]), abs(omega))
            else: # Not moving, but in 'WALKING' state, so we need to finish the step
                command_magnitude = 1.0 # Use a nominal speed to finish the step
            # Dynamically calculate gait cycle speed. If there's no movement, the gait pauses.
            speed = command_magnitude * self.gait_speed_factor

            # Calculate the duration of one full gait cycle (T_cycle) in seconds.
            # This assumes the simulation runs at a rate that makes `speed` meaningful.
            # A smaller gait_speed_factor means a longer cycle.
            # We assume a nominal simulation rate (e.g., 240Hz) and that `speed` advances the phase each step.
            # T_cycle = (1 / (speed * sim_rate)) if speed > 0 else float('inf')
            # For simplicity, we'll tie step length directly to velocity input, not cycle time.
            
            # If we are in the WALKING state but received a stop command, we should
            # continue using the *last* known velocity to complete the step gracefully,
            # but set the actual target velocity to zero for the IK calculation.
            # The `speed` variable will keep the gait phase advancing.
            if is_moving:
                target_vx = vx * self.max_linear_velocity
                target_vy = vy * self.max_linear_velocity
                target_omega = omega * self.max_angular_velocity
            else: # In WALKING state, but is_moving is false
                target_vx, target_vy, target_omega = 0, 0, 0

            # Note: recalculate_stance() is intentionally not called here for performance.
            # It's only called when standoff or body_height are changed.

            new_angles = self.current_gait.run(target_vx, target_vy, target_omega, roll, pitch, speed, self.default_foot_positions, self.last_known_angles, self.body_height, self.current_gait.step_height)
            # Update the last known angles with the new valid ones.
            for i in range(6):
                self.last_known_angles[i] = new_angles[i]
            return new_angles
        else:
            return [None] * 6

    def recalculate_stance(self, standoff_distance=None):
        if standoff_distance is None:
            standoff_distance = self.standoff_distance
        self.standoff_distance = standoff_distance
        self.default_foot_positions = []
        for pos in self.kinematics.hip_positions:
            v = np.array([pos[0], pos[1], 0])
            if np.linalg.norm(v) > 0:
                v_norm = v / np.linalg.norm(v)
            else:
                v_norm = v
            default_pos_xy = v + v_norm * standoff_distance
            self.default_foot_positions.append(np.array([default_pos_xy[0], default_pos_xy[1], -self.body_height]))
        self.foot_positions = np.array(self.default_foot_positions)

        # Pre-calculate and cache the joint angles for the default neutral stance.
        # This is much more efficient than recalculating them on-the-fly during gait.
        self.default_joint_angles = []
        new_foot_targets_local = self.kinematics.body_ik(np.zeros(3), np.zeros(3), self.kinematics.hip_positions, self.foot_positions)
        for i in range(6):
            angles = self.kinematics.leg_ik(new_foot_targets_local[i], knee_direction=self.knee_direction) 
            if angles is None:
                print(f"WARNING: Default stance for leg {i} is unreachable! Check body_height and standoff. Defaulting to home.")
                angles = np.zeros(3) # Fallback to a safe home position if stance is invalid
            self.default_joint_angles.append(angles)

    def calculate_sit_angles(self):
        """
        Calculates the joint angles for a "sitting" position.
        Legs are tucked underneath the body. This is a good intermediate
        pose before standing up.
        """
        sit_angles = []
        # Define a target position in the local frame for each leg
        # This is forward of the hip and significantly raised up
        l_coxa, l_femur, _ = self.kinematics.segment_lengths
        # A more conservative sitting pose: coxa length forward, half femur length down.
        target_pos_local = np.array([l_coxa, 0, -l_femur * 0.5])

        for i in range(6):
            angles = self.kinematics.leg_ik(target_pos_local, knee_direction=self.knee_direction)
            sit_angles.append(angles if angles is not None else self.last_known_angles[i])

        return sit_angles
