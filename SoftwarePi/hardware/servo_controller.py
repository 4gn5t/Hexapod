from time import sleep
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
from config.robot_config import RobotConfig

class ServoController:
    def __init__(self):
        # Initialize the PCA9685 board
        self.kit = ServoKit(channels=16)
        self._setup_servos()
        
    def _setup_servos(self):
        """Initialize all servos with default parameters"""
        for leg_name, config in RobotConfig.LEG_CONFIGS.items():
            # Set servo ranges
            for pin in [config.hip_pin, config.knee_pin, config.ankle_pin]:
                self.kit.servo[pin].set_pulse_width_range(500, 2500)  # Standard servo range
                
    def move_servo(self, pin: int, angle: int, speed: int = None):
        """
        Move a servo to specified angle
        Args:
            pin: Servo pin number
            angle: Desired angle in degrees
            speed: Movement speed (optional, for smooth motion)
        """
        if speed is None:
            self.kit.servo[pin].angle = angle
        else:
            # Implement smooth motion
            current = self.kit.servo[pin].angle
            step = 1 if current < angle else -1
            for pos in range(int(current), int(angle), step):
                self.kit.servo[pin].angle = pos
                sleep(speed / 1000)  # Convert to seconds
                
    def move_leg(self, leg_name: str, hip_angle: int, knee_angle: int, ankle_angle: int):
        """Move all servos for a specific leg"""
        config = RobotConfig.LEG_CONFIGS[leg_name]
        self.move_servo(config.hip_pin, hip_angle)
        self.move_servo(config.knee_pin, knee_angle)
        self.move_servo(config.ankle_pin, ankle_angle)
        
    def center_all_legs(self):
        """Move all legs to their center positions"""
        for leg_name, config in RobotConfig.LEG_CONFIGS.items():
            self.move_leg(leg_name, config.hip_center, config.knee_center, config.ankle_center)
            
    def cleanup(self):
        """Clean up GPIO pins"""
        GPIO.cleanup()