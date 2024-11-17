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
    # Define pin numbers for each leg (adjust these based on your wiring)
    LEG_CONFIGS = {
        'RIGHT_FRONT':  LegConfig(hip_pin=2,  knee_pin=3,  ankle_pin=4),
        'RIGHT_MIDDLE': LegConfig(hip_pin=5,  knee_pin=6,  ankle_pin=7),
        'RIGHT_BACK':   LegConfig(hip_pin=8,  knee_pin=9,  ankle_pin=10),
        'LEFT_FRONT':   LegConfig(hip_pin=11, knee_pin=12, ankle_pin=13),
        'LEFT_MIDDLE':  LegConfig(hip_pin=14, knee_pin=15, ankle_pin=16),
        'LEFT_BACK':    LegConfig(hip_pin=17, knee_pin=18, ankle_pin=19)
    }
    
    # Robot dimensions (in mm)
    COXA_LENGTH = 50    # Hip joint to knee joint
    FEMUR_LENGTH = 75   # Knee joint to ankle joint
    TIBIA_LENGTH = 120  # Ankle joint to foot
    
    # Default movement parameters
    MOVEMENT_SPEED = 50  # Delay between movements (ms)
    STEP_HEIGHT = 30    # Height to lift leg during walking (mm)