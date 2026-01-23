# l-systems
- Michael Jenz
- January 2025

[](https://github.com/user-attachments/assets/86b930c2-e895-4b2a-a156-6076d28d3048)

## Overview
This package is an implementation of l-systems in ros2 kilted using rclcpp. This project served as a way for me to get comfortable with the cpp api but also resulted in some interesting service client interaction.

The implementation shown above has the turtlesim robot drawing a fractal tree using two rules. It is pretty amazing how with such simple rules you can create a life-like structure.

Rules:
  1. X --> F+[[X]-X]-F[-FX]+X
  2. F -->> FF

Each of the above symbols defines an action for the turtlesim bot.

Actions:
  1. "X" means stay in that position
  2. "F" means move forward
  3. "-" means turn right
  3. "+" means turn left
  4. "[" saves the current position and angle
  5. "]" restores the saved position and angle

Implementing the iteration of these rules to build a set of directions for the robot, and then executing these actions was a great learning experience for me in cpp. Looking forward to getting more expereince with ros2 kilted and rclcpp!

## How to use
Build the project, source the workspace, and run the command `ros2 launch turtle_control waypoint.launch.xml`.
