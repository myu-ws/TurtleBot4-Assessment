#!/usr/bin/env python3

import rclpy
import sys
import select
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main(args=None):
    rclpy.init(args=args)
    navigator = TurtleBot4Navigator()

    # Set initial pose
    home_pose = navigator.getPoseStamped([3.2302, -2.8796, 0.0283], TurtleBot4Directions.EAST)
    navigator.setInitialPose(home_pose)

    # Wait for Nav2 to become active
    navigator.waitUntilNav2Active()

    # Define goal options
    goal_options = [
        {'name': 'Home', 'pose': home_pose},
        {'name': 'Oscilloscope', 'pose': navigator.getPoseStamped([-0.8391, -0.7826, -0.0014], TurtleBot4Directions.WEST)},
        {'name': 'Pneumatics Bench', 'pose': navigator.getPoseStamped([5.5610, -0.8407, 0.0024], TurtleBot4Directions.EAST)},
        # Unreachable Pose (PLC Panel):
        {'name': 'PLC Panel', 'pose': navigator.getPoseStamped([5.1458, -4.0317, 0.0024], TurtleBot4Directions.NORTH_WEST)},
        {'name': 'Workbench A', 'pose': navigator.getPoseStamped([0.9087, -2.9146, 0.0024], TurtleBot4Directions.SOUTH)},
        {'name': 'Exit', 'pose': None}
    ]

    navigator.info("Hello, I am your Lab Assistant Robot. I can take you to your destination.")

    while True:
        print("\nPlease select where you want me to take you:")
        for i, option in enumerate(goal_options):
            print(f"    {i}. {option['name']}")
        user_input = input("Selection: ")

        try:
            selected_index = int(user_input)
        except ValueError:
            navigator.error(f"Invalid input: {user_input}")
            continue

        if selected_index < 0 or selected_index >= len(goal_options):
            navigator.error("Invalid index selected.")
            continue

        if goal_options[selected_index]['name'] == 'Exit':
            break

        goal_pose = goal_options[selected_index]['pose']
        navigator.startToPose(goal_pose)

        # Get result
        result = navigator.getResult()
        result_str = str(result)
        print(f"[DEBUG] Navigation result: {result_str}")

        if result_str == "TaskResult.SUCCEEDED":
            navigator.info("Goal succeeded!")
            continue

        elif result_str in ["TaskResult.FAILED", "TaskResult.CANCELED"]:
            navigator.error("Goal failed!")

            print("\nNavigation failed.")
            print("You have 10 seconds to enter a new destination.")
            print("Press Enter without typing anything to return to home.\n")

            # Print options immediately
            for i, option in enumerate(goal_options):
                print(f"    {i}. {option['name']}")

            print("New selection or Enter to go home: ", end="", flush=True)

            # Wait up to 10 seconds for user input (non-blocking)
            i, _, _ = select.select([sys.stdin], [], [], 10)
            if i:
                new_input = sys.stdin.readline().strip()
            else:
                new_input = ""

            if new_input == "":
                navigator.info("No input received. Navigating to home...")
                navigator.startToPose(home_pose)
                home_result = str(navigator.getResult())
                if home_result == "TaskResult.SUCCEEDED":
                    navigator.info("Returned to home successfully.")
                else:
                    navigator.error("Failed to return to home.")
            else:
                try:
                    new_index = int(new_input)
                    if 0 <= new_index < len(goal_options):
                        if goal_options[new_index]['name'] == 'Exit':
                            break
                        navigator.startToPose(goal_options[new_index]['pose'])
                        second_result = str(navigator.getResult())
                        if second_result == "TaskResult.SUCCEEDED":
                            navigator.info("New goal succeeded!")
                        else:
                            navigator.error("New goal also failed. Going home...")
                            navigator.startToPose(home_pose)
                            navigator.getResult()
                    else:
                        navigator.error("Invalid new index. Going home...")
                        navigator.startToPose(home_pose)
                        navigator.getResult()
                except ValueError:
                    navigator.error("Invalid input. Going home...")
                    navigator.startToPose(home_pose)
                    navigator.getResult()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
