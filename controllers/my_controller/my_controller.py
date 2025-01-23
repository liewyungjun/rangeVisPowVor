from controller import Robot, Keyboard
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from RLVBVP3 import RLVBVP3

import math

# Initialize the robot and keyboard
robot = Robot()
print(robot.getBasicTimeStep())

time_step = int(robot.getBasicTimeStep())

lidar=robot.getDevice('lidar')
lidar.enable(time_step)
lidar.enablePointCloud()

# Motors (assumes Robotino uses "wheel1", "wheel2", "wheel3" for motors)
motors = [robot.getDevice(f"wheel{i}_joint") for i in range(3)]
print(motors)
for motor in motors:
    motor.setPosition(float('inf'))  # Enable velocity control
    motor.setVelocity(0)

# Robot parameters
L = 0.135  # Distance from the center of the robot to the wheel (in meters)

# Angles of the wheels (in radians)
angles = [0, 2 * math.pi / 3, 4 * math.pi / 3]
sin_theta = [-math.sin(angle) for angle in angles]
cos_theta = [math.cos(angle) for angle in angles]

# Define velocities
v_x, v_y, omega = 0.0, 0.0, 0.0
# Main control loop
while robot.step(time_step) != -1:
    # Read keyboard inputs
    range_image=lidar.getRangeImage()
    print("--------------------")
    print("{}".format(range_image))
    v_x, v_y, omega = 0.0, 0.0, 0.0  # Stop when no key is pressed

    # Calculate wheel velocities
    wheel_velocities = [
        -sin_theta[i] * v_x + cos_theta[i] * v_y + L * omega
        for i in range(3)
    ]

    # Set wheel velocities
    for i, motor in enumerate(motors):
        motor.setVelocity(wheel_velocities[i])
