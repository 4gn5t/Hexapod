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

COXA_LENGTH = 40
FEMUR_LENGTH = 60
TIBIA_LENGTH = 90

HIP_MIN, HIP_MAX, HIP_OFFSET = 30, 160, 90
KNEE_MIN, KNEE_MAX, KNEE_OFFSET = 30, 160, 90
ANKLE_MIN, ANKLE_MAX, ANKLE_OFFSET = 30, 160, 90

def clamp_angle(angle, min_angle, max_angle):
    return max(min_angle, min(max_angle, angle))

def move_servo(kit, channel, angle, offset):
    clamped_angle = clamp_angle(angle + offset, 0, 180)
    kit.servo[channel].angle = clamped_angle
    print(f"Moved servo on {'Left' if kit == kit_left else 'Right'} PWM, channel {channel} to {clamped_angle:.2f} degrees")

def set_leg_position(kit, leg, hip_angle, knee_angle, ankle_angle):
    move_servo(kit, leg[0], hip_angle, HIP_OFFSET)
    move_servo(kit, leg[1], knee_angle, KNEE_OFFSET)
    move_servo(kit, leg[2], ankle_angle, ANKLE_OFFSET)

def forward_kinematics(hip_angle, knee_angle, ankle_angle):
    hip = math.radians(hip_angle)
    knee = math.radians(knee_angle)
    ankle = math.radians(ankle_angle)

    x = COXA_LENGTH * math.cos(hip) + FEMUR_LENGTH * math.cos(hip + knee) + TIBIA_LENGTH * math.cos(hip + knee + ankle)
    y = COXA_LENGTH * math.sin(hip) + FEMUR_LENGTH * math.sin(hip + knee) + TIBIA_LENGTH * math.sin(hip + knee + ankle)
    z = -(FEMUR_LENGTH * math.sin(knee) + TIBIA_LENGTH * math.sin(knee + ankle))

    return x, y, z

def generate_walking_sequence():
    neutral = [90, 90, 90]
    forward = [70, 110, 70]
    backward = [110, 70, 110]
    lift = [90, 160, 30]

    sequence = [
        neutral,
        lift,
        forward,
        neutral,
        backward,
    ]
    return sequence

def walking_cycle_fk():
    sequence = generate_walking_sequence()
    legs = [
        (kit_left, LEFT_FRONT), (kit_right, RIGHT_MIDDLE), (kit_left, LEFT_BACK),
        (kit_right, RIGHT_FRONT), (kit_left, LEFT_MIDDLE), (kit_right, RIGHT_BACK)
    ]

    for i in range(len(sequence)):
        for j, (kit, leg) in enumerate(legs):
            angles = sequence[(i + j) % len(sequence)]
            set_leg_position(kit, leg, *angles)
            x, y, z = forward_kinematics(*angles)
            print(f"Leg {leg} position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        time.sleep(0.3)

def emulate_walking_fk(steps):
    print("Starting walking emulation with forward kinematics")
    for _ in range(steps):
        walking_cycle_fk()
    print("Walking emulation complete")

if __name__ == "__main__":
    try:
        emulate_walking_fk(5)  #  5 steps
    except KeyboardInterrupt:
        print("Walking stopped by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Resetting servos to neutral position")
        for kit in [kit_left, kit_right]:
            for channel in range(9):
                move_servo(kit, channel, 90, 0)
        print("All servos reset")
