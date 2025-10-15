"""
Hexapod Platform Abstract Base Class
====================================

This module defines the abstract interface for any platform that can control
the hexapod robot. This allows high-level code (like locomotion) to be
written independently of whether it's running on a simulation or a physical
robot.
"""

from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any

class HexapodPlatform(ABC):
    """
    An abstract base class that defines the standard interface for controlling
    a hexapod robot, whether it's a simulation or a physical entity.
    """
    def __init__(self):
        """Initializes the platform with a default neutral stance."""
        # A neutral standing pose (e.g., all zeros or a specific stand-up pose)
        self.target_joint_angles: List[Optional[List[float]]] = [[0.0, 0.0, 0.0]] * 6

    @abstractmethod
    def start(self):
        """Initializes and connects to the platform."""
        pass

    @abstractmethod
    def stop(self):
        """Disconnects from the platform and cleans up resources."""
        pass

    def set_joint_angles(self, joint_angles: List[Optional[List[float]]]):
        """
        Updates the internal target joint angles for the hexapod.

        This method stores the desired state. The platform's own update loop
        is responsible for applying this state to the hardware or simulator.

        :param joint_angles: A list of 6 lists, where each inner list contains
                             the [coxa, femur, tibia] angles in radians for a leg.
        """
        self.target_joint_angles = joint_angles

    @abstractmethod
    def _apply_joint_angles(self):
        """Internal method to send the stored target angles to the hardware/simulator."""
        pass

    @abstractmethod
    def get_camera_image(self, camera_id: int, width: int = 640, height: int = 480) -> Optional[Any]:
        """
        Captures an image from a specified camera.

        :param camera_id: The identifier for the camera (e.g., 0 for front, 1 for rear).
        :param width: The desired width of the image.
        :param height: The desired height of the image.
        :return: An image object (e.g., a numpy array) or None if capture fails.
        """
        pass

    @abstractmethod
    def get_front_camera_image(self, width: int = 640, height: int = 480) -> Optional[Any]:
        """
        Captures an image from the front-facing camera.

        :param width: The desired width of the image.
        :param height: The desired height of the image.
        :return: An image object (e.g., a numpy array) or None if capture fails.
        """
        pass

    @abstractmethod
    def get_rear_camera_image(self, width: int = 640, height: int = 480) -> Optional[Any]:
        """
        Captures an image from the rear-facing camera.

        :param width: The desired width of the image.
        :param height: The desired height of the image.
        :return: An image object (e.g., a numpy array) or None if capture fails.
        """
        pass

    @abstractmethod
    def get_imu_data(self) -> Optional[Dict[str, Dict[str, float]]]:
        """
        Retrieves data from the Inertial Measurement Unit (IMU).

        :return: A dictionary containing accelerometer and gyroscope data, e.g.,
                 {'accel': {'x': ax, 'y': ay, 'z': az},
                  'gyro':  {'x': gx, 'y': gy, 'z': gz}}
                 or None if data is unavailable.
        """
        pass