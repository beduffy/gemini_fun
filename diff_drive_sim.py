import pygame
import numpy as np
import math


def differential_drive_kinematics(v_r, v_l, R, L):
    """
    Calculates the robot's linear and angular velocities from wheel velocities.

    Args:
        v_r: Right wheel velocity (meters per second)
        v_l: Left wheel velocity (meters per second)
        R: Wheel radius (meters)
        L: Distance between wheel centers (meters)

    Returns:
        v: Robot's linear velocity (meters per second)
        omega: Robot's angular velocity (radians per second)
    """

    v = (v_r + v_l) / 2
    omega = (v_r - v_l) / L
    return v, omega


def odometry_update(v_r, v_l, ticks_per_meter, R, L, Δt, x_t, y_t, theta_t):
    """
    Updates the robot's pose (position and orientation) based on wheel odometry.

    Args:
        v_r: Right wheel velocity (meters per second)
        v_l: Left wheel velocity (meters per second)
        ticks_per_meter: Conversion factor from encoder ticks to meters (ticks/meter)
        R: Wheel radius (meters)
        L: Distance between wheel centers (meters)
        Δt: Time elapsed since the last pose update (seconds)
        x_t: Robot's x-coordinate at time t (meters)
        y_t: Robot's y-coordinate at time t (meters)
        theta_t: Robot's orientation angle at time t (radians)

    Returns:
        x_t+1: Robot's updated x-coordinate (meters)
        y_t+1: Robot's updated y-coordinate (meters)
        theta_t+1: Robot's updated orientation angle (radians)
    """

    # Convert encoder ticks to meters
    Delta_s_r = v_r * Δt  # Calculate right wheel displacement (meters)
    Delta_s_l = v_l * Δt  # Calculate left wheel displacement (meters)

    # Calculate linear and angular velocities (same as previous function)
    v, omega = differential_drive_kinematics(v_r, v_l, R, L)

    # Calculate displacement components
    Delta_x = v * Δt * math.cos(theta_t) / 2
    Delta_y = v * Δt * math.sin(theta_t) / 2

    # Update pose
    x_t_plus_1 = x_t + Delta_x
    y_t_plus_1 = y_t + Delta_y
    theta_t_plus_1 = theta_t + omega * Δt

    return x_t_plus_1, y_t_plus_1, theta_t_plus_1

# Define robot and simulation parameters
robot_radius = 0.1  # Meters
# robot_radius = 0.1  # Meters
robot_width = 0.2  # Meters
wheel_radius = 0.05  # Meters
robot_color = (0, 255, 0)  # Green
background_color = (255, 255, 255)  # White
screen_width = 800
screen_height = 600
max_velocity = 500  # Meters per second
max_angular_velocity = 2.0  # Radians per second
dt = 0.01  # Simulation time step (seconds)

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Differential Drive Simulation")
clock = pygame.time.Clock()

# Function to draw the robot
def draw_robot(x, y, theta):
#   # Convert meters to screen coordinates (corrected)
#   center_x = int(x * screen_width)
#   center_y = int(screen_height - y * screen_height)

#   points = [
#       rotate_point(robot_width/2, -robot_radius, theta),
#       rotate_point(robot_width/2, robot_radius, theta),
#       rotate_point(-robot_width/2, robot_radius, theta),
#       rotate_point(-robot_width/2, -robot_radius, theta)
#   ]

#   # Shift and draw the polygon using the calculated center
#   shifted_points = [(p[0] + center_x, p[1] + center_y) for p in points]
#   pygame.draw.polygon(screen, robot_color, shifted_points)
   # Convert meters to screen coordinates (corrected)
#   center_x = int(x * screen_width)
#   center_y = int(screen_height - y * screen_height)
  center_x = 100
  center_y = 100

  # Draw a circle instead of a polygon (for sanity check)
  radius = int(robot_radius * screen_width)  # Adjust radius based on screen size
  pygame.draw.circle(screen, robot_color, (center_x, center_y), radius)

# Function to rotate a point around the origin
def rotate_point(x, y, angle):
  new_x = x * math.cos(angle) - y * math.sin(angle)
  new_y = x * math.sin(angle) + y * math.cos(angle)
  return new_x, new_y

# Odometry variables
x = 100.0  # Robot's x-coordinate (meters)
y = 10.0  # Robot's y-coordinate (meters)
theta = 0.0  # Robot's orientation angle (radians)

# Simulation loop
running = True
while running:
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False

  # Get user input (example using keyboard arrows)
  keys = pygame.key.get_pressed()
  if keys[pygame.K_LEFT]:
    v_left = max_velocity
    v_right = 0
  elif keys[pygame.K_RIGHT]:
    v_right = max_velocity
    v_left = 0
  else:
    v_left = v_right = 0

  # Calculate linear and angular velocities
  v, omega = differential_drive_kinematics(v_left, v_right, wheel_radius, robot_width)

  # Limit velocities
  if abs(v) > max_velocity:
    v = np.sign(v) * max_velocity
  if abs(omega) > max_angular_velocity:
    omega = np.sign(omega) * max_angular_velocity

  # Update robot pose using odometry
  x, y, theta = odometry_update(v, omega, 1/wheel_radius, wheel_radius, robot_width, dt, x, y, theta)

#   Fill background and draw robot
  screen.fill(background_color)
  draw_robot(x, y, theta)

  # Update display
  pygame.display.flip()
  clock.tick(60)

# Quit pygame
pygame.quit()