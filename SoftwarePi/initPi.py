from time import sleep
from hardware.servo_controller import ServoController
from kinematics.inverse_kinematics import InverseKinematics
from movement.gait_generator import GaitGenerator

def main():
    try:
        # Initialize components
        controller = ServoController()
        ik_solver = InverseKinematics()
        gait_generator = GaitGenerator(controller, ik_solver)
        
        # Start in neutral position
        controller.center_all_legs()
        sleep(2)
        
        # Execute walking sequence
        print("Starting to walk...")
        gait_generator.walk_forward(steps=3)  # Take 3 steps forward
        
        # Return to neutral position
        controller.center_all_legs()
        
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    finally:
        controller.cleanup()

if __name__ == "__main__":
    main()