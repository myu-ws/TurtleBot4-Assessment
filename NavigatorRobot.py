#!/usr/bin/env python3

# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Hilary Luo (hluo@clearpathrobotics.com)

import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main(args=None):
    rclpy.init(args=args)

    navigator = TurtleBot4Navigator()


    # Set initial pose
    initial_pose = navigator.getPoseStamped([3.2302, -2.8796, 0.0283], TurtleBot4Directions.SOUTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()


    # Prepare goal pose options
    goal_options = [
        {'name': 'Home',
         'pose': navigator.getPoseStamped([3.2302, -2.8796, 0.0283], TurtleBot4Directions.SOUTH)},
         
        {'name': 'Oscilloscope',
         'pose': navigator.getPoseStamped([-0.8391, -0.7826, -0.0014], TurtleBot4Directions.WEST)},

        {'name': 'Pneumatics Bench',
         'pose': navigator.getPoseStamped([5.5610, -0.8407, 0.0024], TurtleBot4Directions.EAST)},

        {'name': 'PLC Panel',
         'pose': navigator.getPoseStamped([3.3241, 0.4404, 0.0024], TurtleBot4Directions.NORTH_WEST)},

        {'name': 'Workbench A',
         'pose': navigator.getPoseStamped([0.9087, -2.9146, 0.0024], TurtleBot4Directions.SOUTH)},


        {'name': 'Exit',
         'pose': None}
    ]

    navigator.info('Hello, I am your Lab Assistant Robot. I can take you to your destination.')

    while True:
        # Create a list of the goals for display
        options_str = 'Please select where you want me to take you:\n'
        for i in range(len(goal_options)):
            options_str += f'    {i}. {goal_options[i]["name"]}\n'

        # Prompt the user for the goal location
        raw_input = input(f'{options_str}Selection: ')

        selected_index = 0

        # Verify that the value input is a number
        try:
            selected_index = int(raw_input)
        except ValueError:
            navigator.error(f'Invalid goal selection: {raw_input}')
            continue

        # Verify that the user input is within a valid range
        if selected_index < 0 or selected_index >= len(goal_options):
            navigator.error('Selection out of range.')
            continue

            selected_goal = goal_options[selected_index]

        if selected_goal['name'] == 'Exit':
            navigator.info('Exiting navigation session.')
            break

        navigator.info(f"Navigating to {selected_goal['name']}...")
        navigator.startToPose(selected_goal['pose'])

        result = navigator.navigator.waitUntilNav2GoalReached()


        if result:
            navigator.info(f"Successfully reached {selected_goal['name']}.")
        else:
            navigator.error(f"Failed to reach {selected_goal['name']}. Returning to home position...")
            navigator.startToPose(initial_pose)

            # Wait to confirm return to home
            home_result = navigator.waitUntilNav2GoalReached()

            if home_result:
                navigator.info("Successfully returned to home.")
            else:
                navigator.error("Failed to return home. Please check the robot.")
        
         

    rclpy.shutdown()


if __name__ == '__main__':
    main()
