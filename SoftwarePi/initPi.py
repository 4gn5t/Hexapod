from time import sleep
from hardware.servo_controller import ServoController
from kinematics.inverse_kinematics import InverseKinematics
from movement.gait_generator import GaitGenerator

def main():
    controller = None
    try:
        # Initialize servo controller
        print("Initializing servo controller...")
        controller = ServoController()
        
        # Center all legs as starting position
        print("Centering all legs...")
        controller.center_all_legs()
        sleep(2)
        
        # Example movement - lift right front leg
        print("Testing right front leg movement...")
        controller.move_leg('RIGHT_FRONT', 90, 45, 90)
        sleep(1)
        
        # Return to center
        print("Returning to center position...")
        controller.center_all_legs()
        
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if controller:
            controller.cleanup()

if __name__ == "__main__":
    main()