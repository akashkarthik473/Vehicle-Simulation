import pygame
import math
from pid import PID
import csv
from datetime import datetime

class Vehicle:
    def __init__(self, x, y):
        # Position and velocity
        self.pos = pygame.Vector2(x, y)
        self.velocity = pygame.Vector2(0, 0)
        self.acceleration = pygame.Vector2(0, 0)
        self.angle = 0
        self.angular_velocity = 0

        # Vehicle parameters
        self.mass = 1000  # kg
        self.max_engine_force = 4000  # N
        self.wheel_base = 2.5  # m
        self.max_steering_angle = 0.5  # rad
        self.max_steering_speed = 2.0  # rad/s
        self.max_speed = 50  # m/s

        # Physics constants
        self.rolling_friction = 0.01  # coefficient of rolling friction
        self.air_drag = 0.3  # drag coefficient
        self.steering_damping = 0.8  # steering return to center
        self.moment_of_inertia = 1000  # kg⋅m²

        # Control inputs
        self.steering = 0
        self.throttle = 0

        # Visual parameters
        self.radius = 20  # pixels
        self.color = "red"
        self.wheel_color = "black"
        self.wheel_radius = 8

        # Logging setup
        self.log_file = open(f'vehicle_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(['Time', 'Position X', 'Position Y', 'Velocity X', 'Velocity Y', 
                                'Speed', 'Angle', 'Angular Velocity', 'Throttle', 'Steering', 
                                'Acceleration X', 'Acceleration Y'])

    def log_data(self, time):
        speed = self.velocity.length()
        self.log_writer.writerow([
            time, self.pos.x, self.pos.y,
            self.velocity.x, self.velocity.y,
            speed, math.degrees(self.angle),
            self.angular_velocity,
            self.throttle, self.steering,
            self.acceleration.x, self.acceleration.y
        ])
        self.log_file.flush()

    def __del__(self):
        if hasattr(self, 'log_file'):
            self.log_file.close()

    def update(self, dt):
        # Calculate heading vector
        heading = pygame.Vector2(math.cos(self.angle), math.sin(self.angle))
        right = pygame.Vector2(-heading.y, heading.x)

        # Engine force in heading direction
        engine_force = self.max_engine_force * self.throttle
        force = heading * engine_force

        # Air drag (proportional to velocity squared)
        speed = self.velocity.length()
        if speed > 0:
            drag_force = -self.air_drag * speed * self.velocity
            force += drag_force

        # Rolling friction
        if speed > 0:
            friction_force = -self.rolling_friction * self.mass * 9.81 * self.velocity / speed
            force += friction_force

        # Calculate acceleration (F = ma)
        self.acceleration = force / self.mass

        # Update velocity and position
        self.velocity += self.acceleration * dt
        self.pos += self.velocity * dt

        # Steering dynamics
        target_steering = self.steering * self.max_steering_angle
        current_steering = self.angular_velocity / self.max_steering_speed
        steering_diff = target_steering - current_steering
        
        # Apply steering torque
        steering_torque = steering_diff * 1000  # Adjust this value to change steering responsiveness
        angular_acceleration = steering_torque / self.moment_of_inertia
        self.angular_velocity += angular_acceleration * dt
        
        # Apply steering damping
        self.angular_velocity *= self.steering_damping
        
        # Update angle
        self.angle += self.angular_velocity * dt

        # Speed limiting
        if self.velocity.length() > self.max_speed:
            self.velocity.scale_to_length(self.max_speed)

    def draw(self, screen):
        # Draw vehicle body
        heading = pygame.Vector2(math.cos(self.angle), math.sin(self.angle))
        right = pygame.Vector2(-heading.y, heading.x)
        
        # Calculate corners of the vehicle rectangle
        front = self.pos + heading * self.radius
        back = self.pos - heading * self.radius
        left = self.pos + right * (self.radius * 0.5)
        right_pos = self.pos - right * (self.radius * 0.5)
        
        # Draw vehicle body
        pygame.draw.polygon(screen, self.color, [front, left, back, right_pos])
        
        # Draw wheels
        wheel_offset = self.radius * 0.7
        front_left = front + right * wheel_offset
        front_right = front - right * wheel_offset
        back_left = back + right * wheel_offset
        back_right = back - right * wheel_offset
        
        for wheel_pos in [front_left, front_right, back_left, back_right]:
            pygame.draw.circle(screen, self.wheel_color, wheel_pos, self.wheel_radius)

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
font = pygame.font.Font(None, 36)  # Initialize font with default font, size 36
running = True
dt = 0



# Create vehicle
vehicle = Vehicle(0, screen.get_height() / 2)

# Motion diagram
motion_dots = []
frame_count = 0

total_time = 1

# Target speed
target_speed = 10
current_speed = 0

# Create PID controller
pid = PID(Kp=10, Ki=0, Kd=0, saturation=231)

while running:
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Manual control of target speed using arrow keys
    keys = pygame.key.get_pressed()
    speed_step = 0.5                   # adjustable granularity
    if keys[pygame.K_UP]:   target_speed += speed_step
    if keys[pygame.K_DOWN]: target_speed -= speed_step
    target_speed = max(0, min(target_speed, 50))

    speed_text  = font.render(f"Speed: {current_speed:5.1f} m/s", True, "white")
    setpt_text  = font.render(f"Target: {target_speed:5.1f}",     True, "white")
    screen.blit(speed_text, (10,10))
    screen.blit(setpt_text,  (10,30))

    screen.fill("purple")

    # Update current speed
    current_speed = vehicle.velocity.length()

    # Update PID controller
    pid.update_setpoint(target_speed)
    throttle = pid.compute(current_speed)
    vehicle.throttle = throttle


    # Add new dot every 6 frames
    frame_count += 1
    if frame_count >= 6:
        motion_dots.append(vehicle.pos.copy())
        frame_count = 0
        # Remove dots that are off screen
        motion_dots = [pos for pos in motion_dots if pos.x < screen.get_width()]

    # Draw the motion diagram dots
    for pos in motion_dots:
        pygame.draw.circle(screen, "blue", pos, 5)

    # Draw the main circle
    pygame.draw.circle(screen, "red", vehicle.pos, 40)
    
    # Update position and velocity
    vehicle.update(dt)
    vehicle.draw(screen)

    # flip() the display to put your work on screen
    pygame.display.flip()

    total_time += dt
    vehicle.log_data(total_time)

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000

pygame.quit()