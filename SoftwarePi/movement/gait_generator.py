from dataclasses import dataclass
from typing import List, Dict
import numpy as np
from time import sleep

@dataclass
class LegPosition:
    x: float
    y: float
    z: float

class GaitGenerator:
    def __init__(self, controller, ik_solver):
        self.controller = controller
        self.ik_solver = ik_solver
        self.step_height = RobotConfig.STEP_HEIGHT
        
        # Define leg groups for tripod gait
        self.leg_group1 = ['RIGHT_FRONT', 'LEFT_MIDDLE', 'RIGHT_BACK']
        self.leg_group2 = ['LEFT_FRONT', 'RIGHT_MIDDLE', 'LEFT_BACK']
        
    def move_leg_to_position(self, leg_name: str, position: LegPosition):
        """Move a single leg to specified position"""
        try:
            hip_angle, knee_angle, ankle_angle = self.ik_solver.calculate_angles(
                position.x, position.y, position.z
            )
            self.controller.move_leg(leg_name, hip_angle, knee_angle, ankle_angle)
        except ValueError as e:
            print(f"Invalid position for leg {leg_name}: {e}")

    def generate_step_cycle(self, forward_distance: float = 50) -> Dict[str, List[LegPosition]]:
        """Generate a complete step cycle for walking"""
        steps = 10  # Number of intermediate positions per step
        step_positions = {}
        
        for leg_name in RobotConfig.LEG_CONFIGS.keys():
            positions = []
            # Generate trajectory
            for i in range(steps):
                t = i / (steps - 1)
                if leg_name in self.leg_group1:
                    # First tripod group
                    x = forward_distance * (2 * t - 1)
                    z = self.step_height * np.sin(t * np.pi) if t < 0.5 else 0
                else:
                    # Second tripod group
                    x = forward_distance * (1 - 2 * t)
                    z = self.step_height * np.sin((t + 0.5) * np.pi) if t >= 0.5 else 0
                    
                positions.append(LegPosition(x=x, y=0, z=z))
            step_positions[leg_name] = positions
            
        return step_positions

    def walk_forward(self, steps: int = 1):
        """Execute walking motion"""
        step_positions = self.generate_step_cycle()
        
        for _ in range(steps):
            # Execute one complete step cycle
            for step in range(len(next(iter(step_positions.values())))):
                # Move each leg to its next position
                for leg_name, positions in step_positions.items():
                    self.move_leg_to_position(leg_name, positions[step])
                sleep(RobotConfig.MOVEMENT_SPEED / 1000)
