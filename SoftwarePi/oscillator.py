import time
import math
from adafruit_servokit import ServoKit

kit_left = ServoKit(channels=16, address=0x40)  # Left 
kit_right = ServoKit(channels=16, address=0x60)  # Right 

LEFT_FRONT = [0, 1, 2]
LEFT_MIDDLE = [3, 4, 5]
LEFT_BACK = [6, 7, 8]
RIGHT_FRONT = [0, 1, 2]
RIGHT_MIDDLE = [3, 4, 5]
RIGHT_BACK = [6, 7, 8]

class ServoConfig:
    def __init__(self, center, amplitude, phase_offset=0):
        self.center = center      
        self.amplitude = amplitude  
        self.phase_offset = phase_offset  


MOVE_PATTERNS = {
    'walking': {
        'hip': ServoConfig(center=90, amplitude=45),
        'knee': ServoConfig(center=90, amplitude=30),
        'ankle': ServoConfig(center=90, amplitude=30)
    },
    'turning': {
        'hip': ServoConfig(center=90, amplitude=60),
        'knee': ServoConfig(center=90, amplitude=20),
        'ankle': ServoConfig(center=90, amplitude=20)
    }
}

def generate_smooth_oscillation(t, config, frequency=1.0):
    angle = config.center + config.amplitude * math.sin(2 * math.pi * frequency * t + config.phase_offset)
    return max(60, min(120, angle))

def move_servo(kit, channel, angle):
    safe_angle = max(0, min(180, angle))
    kit.servo[channel].angle = safe_angle
    print(f"Moved servo on {'Left' if kit == kit_left else 'Right'} PWM, channel {channel} to {safe_angle} degrees")

class HexapodController:
    def __init__(self):
        self.time_step = 0.2  
        self.current_pattern = 'walking'
        
    def calculate_leg_phase_offsets(self, leg_index, pattern_type='walking'):
        if pattern_type == 'walking':
            return {
                0: 0,          # LEFT_FRONT
                1: math.pi,    # LEFT_MIDDLE
                2: 0,          # LEFT_BACK
                3: math.pi,    # RIGHT_FRONT
                4: 0,          # RIGHT_MIDDLE
                5: math.pi,    # RIGHT_BACK
            }[leg_index]
        elif pattern_type == 'turning':
            return (leg_index * 2 * math.pi / 6)
        return 0

    def move_leg(self, kit, leg, t, leg_index):
        pattern = MOVE_PATTERNS[self.current_pattern]
        phase_offset = self.calculate_leg_phase_offsets(leg_index, self.current_pattern)
        
        hip_config = ServoConfig(
            pattern['hip'].center, 
            pattern['hip'].amplitude, 
            phase_offset
        )
        knee_config = ServoConfig(
            pattern['knee'].center, 
            pattern['knee'].amplitude, 
            phase_offset + math.pi/2 
        )
        ankle_config = ServoConfig(
            pattern['ankle'].center, 
            pattern['ankle'].amplitude, 
            phase_offset + math.pi/2
        )
        
        hip_angle = generate_smooth_oscillation(t, hip_config)
        knee_angle = generate_smooth_oscillation(t, knee_config)
        ankle_angle = generate_smooth_oscillation(t, ankle_config)
        
        if kit == kit_right:
            hip_angle = 180 - hip_angle
        
        move_servo(kit, leg[0], hip_angle)
        move_servo(kit, leg[1], knee_angle)
        move_servo(kit, leg[2], ankle_angle)

    def wave_gait(self, duration=10):
        start_time = time.time()
        t = 0
        
        legs = [
            (kit_left, LEFT_FRONT, 0),
            (kit_left, LEFT_MIDDLE, 1),
            (kit_left, LEFT_BACK, 2),
            (kit_right, RIGHT_FRONT, 3),
            (kit_right, RIGHT_MIDDLE, 4),
            (kit_right, RIGHT_BACK, 5)
        ]
        
        try:
            while (time.time() - start_time) < duration:
                for kit, leg, index in legs:
                    self.move_leg(kit, leg, t, index)
                t += self.time_step
                time.sleep(self.time_step)
                
        except KeyboardInterrupt:
            print("\nStopping movement")
        finally:
            self.reset_position()

    def ripple_gait(self, duration=10):
        self.current_pattern = 'turning'
        start_time = time.time()
        t = 0
        
        legs = [
            (kit_left, LEFT_FRONT, 0),
            (kit_left, LEFT_MIDDLE, 1),
            (kit_left, LEFT_BACK, 2),
            (kit_right, RIGHT_FRONT, 3),
            (kit_right, RIGHT_MIDDLE, 4),
            (kit_right, RIGHT_BACK, 5)
        ]
        
        try:
            while (time.time() - start_time) < duration:
                for kit, leg, index in legs:
                    self.move_leg(kit, leg, t, index)
                t += self.time_step
                time.sleep(self.time_step)
                
        except KeyboardInterrupt:
            print("\nStopping movement")
        finally:
            self.current_pattern = 'walking'
            self.reset_position()

    def reset_position(self):
        print("Resetting to neutral position")
        for kit in [kit_left, kit_right]:
            for leg in [LEFT_FRONT, LEFT_MIDDLE, LEFT_BACK] if kit == kit_left else [RIGHT_FRONT, RIGHT_MIDDLE, RIGHT_BACK]:
                move_servo(kit, leg[0], 90)  # Hip
                move_servo(kit, leg[1], 90)  # Knee
                move_servo(kit, leg[2], 90)  # Ankle
        print("Reset complete")

def main():
    controller = HexapodController()
    
    try:
        print("Starting wave gait...")
        controller.wave_gait(duration=10)
        
        time.sleep(1)  
        
        print("Starting ripple gait...")
        controller.ripple_gait(duration=10)
        
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        controller.reset_position()

if __name__ == "__main__":
    main()