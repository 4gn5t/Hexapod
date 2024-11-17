import numpy as np
from math import atan2, acos, sqrt, degrees, radians
from config.robot_config import RobotConfig

class InverseKinematics:
    def __init__(self):
        self.coxa_length = RobotConfig.COXA_LENGTH
        self.femur_length = RobotConfig.FEMUR_LENGTH
        self.tibia_length = RobotConfig.TIBIA_LENGTH

    def calculate_angles(self, x: float, y: float, z: float) -> tuple:
        """
        Calculate joint angles for given cartesian coordinates
        Args:
            x: Forward/backward position
            y: Left/right position
            z: Height from ground
        Returns:
            tuple: (hip_angle, knee_angle, ankle_angle) in degrees
        """
        # Calculate hip angle
        hip_angle = degrees(atan2(y, x))
        
        # Calculate the distance from hip to foot in the X-Z plane
        length_hip_foot = sqrt(x**2 + y**2) - self.coxa_length
        delta_z = self.tibia_length - z
        
        # Calculate length between knee and foot
        length_knee_foot = sqrt(length_hip_foot**2 + delta_z**2)
        
        # Calculate knee angle using law of cosines
        knee_angle = degrees(acos(
            (self.femur_length**2 + length_knee_foot**2 - self.tibia_length**2) /
            (2 * self.femur_length * length_knee_foot)
        ))
        
        # Calculate ankle angle
        ankle_angle = degrees(acos(
            (self.femur_length**2 + self.tibia_length**2 - length_knee_foot**2) /
            (2 * self.femur_length * self.tibia_length)
        ))
        
        return hip_angle, knee_angle, ankle_angle