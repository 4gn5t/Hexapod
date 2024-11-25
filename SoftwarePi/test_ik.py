import numpy as np
import time
from adafruit_servokit import ServoKit

kit_left = ServoKit(channels=16, address=0x40)  # Left 
kit_right = ServoKit(channels=16, address=0x60)  # Right 

LEFT_FRONT = [0, 1, 2]
LEFT_MIDDLE = [3, 4, 5]
LEFT_BACK = [6, 7, 8]
RIGHT_FRONT = [0, 1, 2]
RIGHT_MIDDLE = [3, 4, 5]
RIGHT_BACK = [6, 7, 8]

COXA_LENGTH = 50    
FEMUR_LENGTH = 75   
TIBIA_LENGTH = 120  

def clamp_angle(angle):
    return max(30, min(150, angle))

def move_servo(kit, channel, angle):
    clamped_angle = clamp_angle(angle)
    kit.servo[channel].angle = clamped_angle
    print(f"Moved servo on {'Left' if kit == kit_left else 'Right'} PWM, channel {channel} to {clamped_angle} degrees")

def inverse_kinematics(x, y, z):
    try:
        hip_angle = np.degrees(np.arctan2(y, x))
        
        L = np.sqrt(x**2 + y**2) - COXA_LENGTH
        D = np.sqrt(L**2 + z**2)
        
        if D > (FEMUR_LENGTH + TIBIA_LENGTH):
            raise ValueError("Position out of reach")
        
        knee_angle = np.degrees(np.arccos(
            (FEMUR_LENGTH**2 + TIBIA_LENGTH**2 - D**2) / 
            (2 * FEMUR_LENGTH * TIBIA_LENGTH)
        ))
        
        ankle_angle = np.degrees(np.arctan2(z, L) + np.arccos(
            (FEMUR_LENGTH**2 + D**2 - TIBIA_LENGTH**2) /
            (2 * FEMUR_LENGTH * D)
        ))
        
        hip_angle = 90 + hip_angle
        knee_angle = 180 - knee_angle
        ankle_angle = 90 + ankle_angle
        
        return (clamp_angle(hip_angle), 
                clamp_angle(knee_angle), 
                clamp_angle(ankle_angle))
                
    except Exception as e:
        print(f"IK calculation error: {e}")
        return None

def move_leg_to_position(kit, leg, x, y, z):
    angles = inverse_kinematics(x, y, z)
    if angles:
        hip_angle, knee_angle, ankle_angle = angles
        set_leg_position(kit, leg, hip_angle, knee_angle, ankle_angle)

def set_leg_position(kit, leg, hip_angle, knee_angle, ankle_angle):
    move_servo(kit, leg[0], hip_angle)
    move_servo(kit, leg[1], knee_angle)
    move_servo(kit, leg[2], ankle_angle)

def walking_gait_ik():
    body_height = -100
    stride_length = 100
    stride_height = 50
    
    move_leg_to_position(kit_left, LEFT_FRONT, stride_length/2, 0, body_height + stride_height)
    move_leg_to_position(kit_right, RIGHT_MIDDLE, stride_length/2, 0, body_height + stride_height)
    move_leg_to_position(kit_left, LEFT_BACK, stride_length/2, 0, body_height + stride_height)
    
    move_leg_to_position(kit_right, RIGHT_FRONT, -stride_length/2, 0, body_height)
    move_leg_to_position(kit_left, LEFT_MIDDLE, -stride_length/2, 0, body_height)
    move_leg_to_position(kit_right, RIGHT_BACK, -stride_length/2, 0, body_height)
    time.sleep(0.1)
    
    for leg in [LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK, RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK]:
        kit = kit_left if leg in [LEFT_FRONT, LEFT_MIDDLE, LEFT_BACK] else kit_right
        move_leg_to_position(kit, leg, 0, 0, body_height)
    time.sleep(0.1)
    
    move_leg_to_position(kit_right, RIGHT_FRONT, stride_length/2, 0, body_height + stride_height)
    move_leg_to_position(kit_left, LEFT_MIDDLE, stride_length/2, 0, body_height + stride_height)
    move_leg_to_position(kit_right, RIGHT_BACK, stride_length/2, 0, body_height + stride_height)
    
    move_leg_to_position(kit_left, LEFT_FRONT, -stride_length/2, 0, body_height)
    move_leg_to_position(kit_right, RIGHT_MIDDLE, -stride_length/2, 0, body_height)
    move_leg_to_position(kit_left, LEFT_BACK, -stride_length/2, 0, body_height)
    time.sleep(0.1)
    
    for leg in [LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK, RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK]:
        kit = kit_left if leg in [LEFT_FRONT, LEFT_MIDDLE, LEFT_BACK] else kit_right
        move_leg_to_position(kit, leg, 0, 0, body_height)
    time.sleep(0.1)

def emulate_walking_ik(steps):
    print("Starting walking emulation with inverse kinematics")
    try:
        for _ in range(steps):
            walking_gait_ik()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Resetting servos to neutral position")
        for leg in [LEFT_FRONT, RIGHT_MIDDLE, LEFT_BACK, RIGHT_FRONT, LEFT_MIDDLE, RIGHT_BACK]:
            kit = kit_left if leg in [LEFT_FRONT, LEFT_MIDDLE, LEFT_BACK] else kit_right
            move_leg_to_position(kit, leg, 0, 0, -100) 
        print("All servos reset")

if __name__ == "__main__":
    try:
        emulate_walking_ik(5)  #  5 steps
    except KeyboardInterrupt:
        print("Walking stopped by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Resetting servos to neutral position")
        for kit in [kit_left, kit_right]:
            for channel in range(9):
                move_servo(kit, channel, 90)
        print("All servos reset")