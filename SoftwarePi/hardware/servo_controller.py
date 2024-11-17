from time import sleep
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
from config.robot_config import RobotConfig

class ServoController:
    def __init__(self):
        try:
            # Initialize the PCA9685 board
            self.kit = ServoKit(channels=16)
            self._setup_servos()
            print("Servo controller initialized successfully")
        except Exception as e:
            print(f"Error initializing servo controller: {e}")
            raise
        
    def _setup_servos(self):
        """Initialize all servos with default parameters"""
        try:
            for leg_name, config in RobotConfig.LEG_CONFIGS.items():
                # Set servo ranges for each pin that's within 0-15 range
                for pin in [config.hip_pin, config.knee_pin, config.ankle_pin]:
                    if 0 <= pin <= 15:  # Only configure pins within valid range
                        self.kit.servo[pin].set_pulse_width_range(500, 2500)
                        self.kit.servo[pin].angle = 90  # Set to center position
                        print(f"Configured servo on pin {pin}")
                    else:
                        print(f"Warning: Pin {pin} is out of range (0-15)")
        except Exception as e:
            print(f"Error in _setup_servos: {e}")
            raise
                
    def move_servo(self, pin: int, angle: int, speed: int = None):
        """
        Move a servo to specified angle
        Args:
            pin: Servo pin number
            angle: Desired angle in degrees
            speed: Movement speed (optional, for smooth motion)
        """
        try:
            if 0 <= pin <= 15:  # Check if pin is in valid range
                if speed is None:
                    self.kit.servo[pin].angle = angle
                else:
                    # Implement smooth motion
                    current = self.kit.servo[pin].angle
                    step = 1 if current < angle else -1
                    for pos in range(int(current), int(angle), step):
                        self.kit.servo[pin].angle = pos
                        sleep(speed / 1000)  # Convert to seconds
            else:
                print(f"Warning: Attempted to move servo on invalid pin {pin}")
        except Exception as e:
            print(f"Error moving servo on pin {pin}: {e}")
        
    def move_leg(self, leg_name: str, hip_angle: int, knee_angle: int, ankle_angle: int):
        """Move all servos for a specific leg"""
        try:
            config = RobotConfig.LEG_CONFIGS[leg_name]
            self.move_servo(config.hip_pin, hip_angle)
            self.move_servo(config.knee_pin, knee_angle)
            self.move_servo(config.ankle_pin, ankle_angle)
            print(f"Moved leg {leg_name} to angles: {hip_angle}, {knee_angle}, {ankle_angle}")
        except Exception as e:
            print(f"Error moving leg {leg_name}: {e}")
        
    def center_all_legs(self):
        """Move all legs to their center positions"""
        try:
            for leg_name, config in RobotConfig.LEG_CONFIGS.items():
                self.move_leg(leg_name, config.hip_center, config.knee_center, config.ankle_center)
                print(f"Centered leg {leg_name}")
        except Exception as e:
            print(f"Error centering legs: {e}")
            
    def cleanup(self):
        """Clean up GPIO pins"""
        try:
            GPIO.cleanup()
            print("GPIO cleanup completed")
        except Exception as e:
            print(f"Error in cleanup: {e}")
