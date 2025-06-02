import pygame
import math
from pid import PID
import csv
from datetime import datetime

class Vehicle:
    def __init__(self, x, y):
        #position and velocity
        self.pos = pygame.Vector2(x, y)
        self.velocity = pygame.Vector2(0, 0)
        self.angle = 0

        #vehicle parameters
        self.mass = 1000 #in kg
        self.max_engine_force = 4000 #in Newtons
        self.wheel_base = 2.5 #in meters
        self.max_steering_angle = 0.5 #in radians

        #physics constants
        self.acceleration = 1000 #in m/s^2
        self.friction = 0.7 #coefficient of friction
        self.drag = 0.3 #drag coefficient

        #control inputs
        self.steering = 0
        self.throttle = 0

        #visual parameters
        self.radius = 40 #in pixels
        self.color = "red"

        #logging
        self.log_file = open(f'vehicle_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(['Time', 'Position X', 'Position Y', 'Velocity X', 'Velocity Y', 
                                'Speed', 'Angle', 'Throttle', 'Steering', 'Acceleration'])
    def log_data(self, time):
        speed = self.velocity.length()
        current_acceleration = self.velocity.x / time
        self.log_writer.writerow([
            time,                    # Time
            self.pos.x,             # Position X
            self.pos.y,             # Position Y
            self.velocity.x,        # Velocity X
            self.velocity.y,        # Velocity Y
            speed,                  # Speed
            math.degrees(self.angle), # Angle in degrees
            self.throttle,          # Throttle input
            self.steering,          # Steering input
            current_acceleration    # Current acceleration
        ])
        self.log_file.flush()
    def __del__(self):
        if hasattr(self, 'log_file'):
            self.log_file.close()

    def update(self, dt):
        #update position
        heading = pygame.Vector2(math.cos(self.angle), math.sin(self.angle))
        self.pos += heading * self.velocity.length() * dt

        
        # Calculate force from throttle (in Newtons)
        engine_force = self.max_engine_force * self.throttle
        
        # Calculate acceleration from force (F = ma, so a = F/m)
        acceleration = engine_force / self.mass
        
        #apply acceleration to velocity
        self.velocity.x += acceleration * dt
        
        #apply drag (air resistance)
        drag_force = -self.drag * self.velocity.x * abs(self.velocity.x)
        self.velocity.x += (drag_force / self.mass) * dt
        
        #apply steering
        self.angle += self.steering * dt
        
        #apply friction only when not accelerating
        if abs(self.throttle) < 0.1:
            friction_force = -self.friction * self.mass * 9.81  # F = μmg
            self.velocity.x += (friction_force / self.mass) * dt
    
    def draw(self, screen):
        #draw vehicle
        pygame.draw.circle(screen, self.color, self.pos, self.radius)

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
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
pid = PID(Kp=1, Ki=0, Kd=0, saturation=1.0)

while running:
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
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