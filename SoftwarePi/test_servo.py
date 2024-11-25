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

def move_servo(kit, channel, angle):
    kit.servo[channel].angle = angle
    print(f"Moved servo on {'Left' if kit == kit_left else 'Right'} PWM, channel {channel} to {angle} degrees")

def set_leg_position(kit, leg, hip_angle, knee_angle, ankle_angle):
    move_servo(kit, leg[0], hip_angle)
    move_servo(kit, leg[1], knee_angle)
    move_servo(kit, leg[2], ankle_angle)

def walking_cycle():
    set_leg_position(kit_left, LEFT_FRONT, 45, 135, 45)    
    set_leg_position(kit_right, RIGHT_MIDDLE, 45, 135, 45)
    set_leg_position(kit_left, LEFT_BACK, 45, 135, 45)
    
    set_leg_position(kit_right, RIGHT_FRONT, 135, 45, 135) 
    set_leg_position(kit_left, LEFT_MIDDLE, 135, 45, 135)
    set_leg_position(kit_right, RIGHT_BACK, 135, 45, 135)
    time.sleep(0.3)

    # Cycle 2: Move body forward
    set_leg_position(kit_left, LEFT_FRONT, 90, 90, 90)
    set_leg_position(kit_right, RIGHT_MIDDLE, 90, 90, 90)
    set_leg_position(kit_left, LEFT_BACK, 90, 90, 90)
    
    set_leg_position(kit_right, RIGHT_FRONT, 90, 90, 90)
    set_leg_position(kit_left, LEFT_MIDDLE, 90, 90, 90)
    set_leg_position(kit_right, RIGHT_BACK, 90, 90, 90)
    time.sleep(0.3)

    set_leg_position(kit_right, RIGHT_FRONT, 45, 135, 45) 
    set_leg_position(kit_left, LEFT_MIDDLE, 45, 135, 45)
    set_leg_position(kit_right, RIGHT_BACK, 45, 135, 45)
    
    set_leg_position(kit_left, LEFT_FRONT, 135, 45, 135)   
    set_leg_position(kit_right, RIGHT_MIDDLE, 135, 45, 135)
    set_leg_position(kit_left, LEFT_BACK, 135, 45, 135)
    time.sleep(0.3)

    set_leg_position(kit_right, RIGHT_FRONT, 90, 90, 90)
    set_leg_position(kit_left, LEFT_MIDDLE, 90, 90, 90)
    set_leg_position(kit_right, RIGHT_BACK, 90, 90, 90)
    
    set_leg_position(kit_left, LEFT_FRONT, 90, 90, 90)
    set_leg_position(kit_right, RIGHT_MIDDLE, 90, 90, 90)
    set_leg_position(kit_left, LEFT_BACK, 90, 90, 90)
    time.sleep(0.3)

def emulate_walking(steps):
    print("Starting walking emulation with higher leg lifts")
    for _ in range(steps):
        walking_cycle()
    print("Walking emulation complete")

if __name__ == "__main__":
    try:
        emulate_walking(5)  
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