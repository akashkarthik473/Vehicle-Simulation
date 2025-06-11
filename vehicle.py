import pygame
import math
import csv
from datetime import datetime
from car_config import *
from PowerLimiter import PowerLimiter

class Vehicle:
    def __init__(self, x, y):
        # Core physics state
        self.pos = pygame.Vector2(x, y)
        self.velocity = pygame.Vector2(0, 0)
        self.acceleration = pygame.Vector2(0, 0)
        self.angle = 0
        self.angular_velocity = 0

        # Car specs from config
        self.mass = MASS
        self.wheel_radius = WHEEL_RADIUS
        self.gear_ratio = GEAR_RATIO
        self.motor_max_torque = MOTOR_MAX_TORQUE
        self.motor_max_rpm = MOTOR_MAX_RPM
        self.pack_voltage = PACK_NOMINAL_VOLTAGE
        self.pack_resistance = PACK_SAG_RESISTANCE

        # Calculated limits
        self.max_wheel_torque = self.motor_max_torque * self.gear_ratio
        self.max_force = self.max_wheel_torque / self.wheel_radius
        self.max_speed = (self.motor_max_rpm * 2 * math.pi * self.wheel_radius) / (60 * self.gear_ratio)

        # Control and physics
        self.steering = 0
        self.throttle = 0
        self.brake = 0
        self.rolling_friction = 0.005
        self.air_drag = 0.15
        self.power_limiter = PowerLimiter()

        # Steering parameters
        self.wheel_base = 2.5
        self.max_steering_angle = 0.5
        self.max_steering_speed = 2.0
        self.steering_damping = 0.8
        self.moment_of_inertia = self.mass * 1.5

        # Brake parameters
        self.max_brake_force = 2000
        self.brake_bias = 0.6

        # Visual
        self.radius = 20
        self.color = "red"
        self.wheel_color = "black"
        self.wheel_radius = 8

        # Data logging
        self.log_file = open(f'vehicle_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(['Time', 'Position X', 'Position Y', 'Velocity X', 'Velocity Y', 
                                'Speed', 'Angle', 'Angular Velocity', 'Throttle', 'Brake', 'Steering', 
                                'Acceleration X', 'Acceleration Y', 'Motor RPM', 'Motor Torque',
                                'Mechanical Power (kW)', 'Limited Torque'])

    def log_data(self, time):
        speed = self.velocity.length()
        wheel_rpm = (speed * 60) / (2 * math.pi * self.wheel_radius)
        motor_rpm = wheel_rpm * self.gear_ratio
        
        # Calculate mechanical power (kW)
        mech_power_watts = self.motor_torque * motor_rpm * 2 * math.pi / 60
        mech_power_kw = mech_power_watts / 1000
        
        self.log_writer.writerow([
            time, self.pos.x, self.pos.y,
            self.velocity.x, self.velocity.y,
            speed, math.degrees(self.angle),
            self.angular_velocity,
            self.throttle, self.brake, self.steering,
            self.acceleration.x, self.acceleration.y,
            motor_rpm, self.motor_torque,
            mech_power_kw, self.limited_torque
        ])
        self.log_file.flush()

    def update(self, dt):
        heading = pygame.Vector2(math.cos(self.angle), math.sin(self.angle))
        right = pygame.Vector2(-heading.y, heading.x)

        # Motor physics
        speed = self.velocity.length()
        wheel_rpm = (speed * 60) / (2 * math.pi * self.wheel_radius)
        motor_rpm = wheel_rpm * self.gear_ratio

        # Calculate requested motor torque
        torque_factor = max(0, 1 - (motor_rpm / self.motor_max_rpm))
        requested_torque = self.motor_max_torque * self.throttle * torque_factor

        # Calculate mechanical power
        mech_power_watts = requested_torque * motor_rpm * 2 * math.pi / 60
        mech_power_kw = mech_power_watts / 1000

        # Apply power limiting
        self.limited_torque = self.power_limiter.step(
            mech_power_kw, 
            motor_rpm, 
            requested_torque
        )
        self.motor_torque = self.limited_torque

        # Convert motor torque to wheel force
        wheel_torque = self.motor_torque * self.gear_ratio
        engine_force = (wheel_torque / self.wheel_radius) * heading

        # Brake force
        brake_force = pygame.Vector2(0, 0)
        if speed > 0 and self.brake > 0:
            brake_force = -self.max_brake_force * self.brake * self.velocity / speed

        # Forces
        if speed > 0:
            drag_force = -self.air_drag * speed * self.velocity
            friction_force = -self.rolling_friction * self.mass * 9.81 * self.velocity / speed
            engine_force += drag_force + friction_force + brake_force

        self.acceleration = engine_force / self.mass
        self.velocity += self.acceleration * dt
        self.pos += self.velocity * dt

        # Steering
        target_steering = self.steering * self.max_steering_angle
        current_steering = self.angular_velocity / self.max_steering_speed
        steering_torque = (target_steering - current_steering) * 1000
        angular_acceleration = steering_torque / self.moment_of_inertia
        self.angular_velocity += angular_acceleration * dt
        self.angular_velocity *= self.steering_damping
        self.angle += self.angular_velocity * dt

        if speed > self.max_speed:
            self.velocity.scale_to_length(self.max_speed)

    def draw(self, screen):
        heading = pygame.Vector2(math.cos(self.angle), math.sin(self.angle))
        right = pygame.Vector2(-heading.y, heading.x)
        
        front = self.pos + heading * self.radius
        back = self.pos - heading * self.radius
        left = self.pos + right * (self.radius * 0.5)
        right_pos = self.pos - right * (self.radius * 0.5)
        
        pygame.draw.polygon(screen, self.color, [front, left, back, right_pos])
        
        wheel_offset = self.radius * 0.7
        front_left = front + right * wheel_offset
        front_right = front - right * wheel_offset
        back_left = back + right * wheel_offset
        back_right = back - right * wheel_offset
        
        for wheel_pos in [front_left, front_right, back_left, back_right]:
            pygame.draw.circle(screen, self.wheel_color, wheel_pos, self.wheel_radius)

    def __del__(self):
        if hasattr(self, 'log_file'):
            self.log_file.close() 