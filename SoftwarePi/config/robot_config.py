from dataclasses import dataclass
from typing import Dict, List, Tuple

@dataclass
class LegConfig:
    """Configuration for a single leg"""
    hip_pin: int
    knee_pin: int
    ankle_pin: int
    hip_center: int = 90    # Center position in degrees
    knee_center: int = 90
    ankle_center: int = 90
    hip_range: Tuple[int, int] = (0, 180)
    knee_range: Tuple[int, int] = (0, 180)
    ankle_range: Tuple[int, int] = (0, 180)

class RobotConfig:
    """Main robot configuration"""
    # Define pin numbers for each leg (adjusted to be within 0-15 range)
    LEG_CONFIGS = {
        'RIGHT_FRONT':  LegConfig(hip_pin=0,  knee_pin=1,  ankle_pin=2),
        'RIGHT_MIDDLE': LegConfig(hip_pin=3,  knee_pin=4,  ankle_pin=5),
        'RIGHT_BACK':   LegConfig(hip_pin=6,  knee_pin=7,  ankle_pin=8),
        'LEFT_FRONT':   LegConfig(hip_pin=9,  knee_pin=10, ankle_pin=11),
        'LEFT_MIDDLE':  LegConfig(hip_pin=12, knee_pin=13, ankle_pin=14),
        'LEFT_BACK':    LegConfig(hip_pin=15, knee_pin=0,  ankle_pin=1)  # Note: These will need to be on a second PCA9685
    }
    
    # Robot dimensions (in mm)
    COXA_LENGTH = 50    # Hip joint to knee joint
    FEMUR_LENGTH = 75   # Knee joint to ankle joint
    TIBIA_LENGTH = 120  # Ankle joint to foot
    
    # Default movement parameters
    MOVEMENT_SPEED = 50  # Delay between movements (ms)
    STEP_HEIGHT = 30    # Height to lift leg during walking (mm)