import pygame
import math
from PowerLimiter import PowerLimiter
import csv
from datetime import datetime
from car_config import *
from vehicle import Vehicle
from collections import deque
import matplotlib.pyplot as plt

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
font = pygame.font.Font(None, 36)
running = True
dt = 0

vehicle = Vehicle(0, screen.get_height() / 2)
motion_dots = []
frame_count = 0
total_time = 1

# Driver inputs (0.0 to 1.0)
throttle_input = 0.0
brake_input = 0.0
steering_input = 0.0

# Input sensitivity
throttle_sensitivity = 0.02
brake_sensitivity = 0.02
steering_sensitivity = 0.03

# 0 to 60 timer
zero_to_sixty_started = False
zero_to_sixty_start_time = 0
zero_to_sixty_completed = False
zero_to_sixty_time = 0
target_speed_60mph = 26.8  # m/s

hist_t, hist_speed, hist_throttle, hist_brake = deque(maxlen=600), deque(maxlen=600), deque(maxlen=600), deque(maxlen=600)
plt.ion()
fig, ax = plt.subplots()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Driver inputs
    keys = pygame.key.get_pressed()
    
    # Throttle
    if keys[pygame.K_w]:
        throttle_input = min(1.0, throttle_input + throttle_sensitivity)
    else:
        throttle_input = max(0.0, throttle_input - throttle_sensitivity * 2)  # Faster release
    
    # Brake
    if keys[pygame.K_s]:
        brake_input = min(1.0, brake_input + brake_sensitivity)
    else:
        brake_input = max(0.0, brake_input - brake_sensitivity * 2)  # Faster release
    
    # Steering
    if keys[pygame.K_a]:
        steering_input = max(-1.0, steering_input - steering_sensitivity)
    elif keys[pygame.K_d]:
        steering_input = min(1.0, steering_input + steering_sensitivity)
    else:
        if steering_input > 0:
            steering_input = max(0, steering_input - steering_sensitivity)
        elif steering_input < 0:
            steering_input = min(0, steering_input + steering_sensitivity)

    vehicle.throttle = throttle_input
    vehicle.brake = brake_input
    vehicle.steering = steering_input

    current_speed = vehicle.velocity.length()

    # 0-60 mph timing
    if not zero_to_sixty_started and current_speed > 0.1:
        zero_to_sixty_started = True
        zero_to_sixty_start_time = total_time
    
    if zero_to_sixty_started and not zero_to_sixty_completed and current_speed >= target_speed_60mph:
        zero_to_sixty_completed = True
        zero_to_sixty_time = total_time - zero_to_sixty_start_time

    # Update plots
    hist_t.append(total_time)
    hist_speed.append(current_speed)
    hist_throttle.append(throttle_input * 100)  # Scale for visibility
    hist_brake.append(brake_input * 100)  # Scale for visibility
    
    ax.clear()
    ax.plot(hist_t, hist_speed, label='Speed (m/s)', color='blue')
    ax.plot(hist_t, hist_throttle, label='Throttle (%)', color='green')
    ax.plot(hist_t, hist_brake, label='Brake (%)', color='red')
    ax.set_ylabel('Speed (m/s) / Input (%)')
    ax.set_xlabel('Time (s)')
    ax.legend()
    ax.grid(True)
    plt.pause(0.001)

    screen.fill("purple")

    # Display info
    speed_text = font.render(f"Speed: {current_speed:5.1f} m/s", True, "white")
    throttle_text = font.render(f"Throttle: {throttle_input*100:3.0f}%", True, "white")
    brake_text = font.render(f"Brake: {brake_input*100:3.0f}%", True, "white")
    steering_text = font.render(f"Steering: {steering_input*100:3.0f}%", True, "white")
    power_text = font.render(f"Power: {vehicle.power_limiter.target_kw:3.0f} kW", True, "white")
    
    screen.blit(speed_text, (10, 10))
    screen.blit(throttle_text, (10, 30))
    screen.blit(brake_text, (10, 50))
    screen.blit(steering_text, (10, 70))
    screen.blit(power_text, (10, 90))

    # 0-60 mph display
    if zero_to_sixty_completed:
        zero_to_sixty_text = font.render(f"0-60 mph: {zero_to_sixty_time:.1f}s", True, "green")
        screen.blit(zero_to_sixty_text, (10, 110))
    elif zero_to_sixty_started:
        current_time = total_time - zero_to_sixty_start_time
        zero_to_sixty_text = font.render(f"0-60 mph: {current_time:.1f}s...", True, "yellow")
        screen.blit(zero_to_sixty_text, (10, 110))
    else:
        zero_to_sixty_text = font.render("0-60 mph: Ready", True, "white")
        screen.blit(zero_to_sixty_text, (10, 110))

    # Controls info
    controls_text = font.render("Controls: W=Throttle, S=Brake, A/D=Steering", True, "yellow")
    screen.blit(controls_text, (10, screen.get_height() - 30))

    frame_count += 1
    if frame_count >= 6:
        motion_dots.append(vehicle.pos.copy())
        frame_count = 0
        motion_dots = [pos for pos in motion_dots if pos.x < screen.get_width()]

    for pos in motion_dots:
        pygame.draw.circle(screen, "blue", pos, 5)

    pygame.draw.circle(screen, "red", vehicle.pos, 40)
    vehicle.update(dt)
    vehicle.draw(screen)
    pygame.display.flip()

    total_time += dt
    vehicle.log_data(total_time)
    dt = clock.tick(60) / 1000

pygame.quit()