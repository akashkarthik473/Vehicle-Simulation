import pygame

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

# Physics constants
INITIAL_VELOCITY = 0  # pixels per second
ACCELERATION = 1000     # pixels per second squared

# Motion state
player_pos = pygame.Vector2(0, screen.get_height() / 2)
velocity = INITIAL_VELOCITY

# Motion diagram
motion_dots = []
frame_count = 0

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("purple")

    # Add new dot every 6 frames
    frame_count += 1
    if frame_count >= 6:
        motion_dots.append(player_pos.copy())
        frame_count = 0
        # Remove dots that are off screen
        motion_dots = [pos for pos in motion_dots if pos.x < screen.get_width()]

    # Draw the motion diagram dots
    for pos in motion_dots:
        pygame.draw.circle(screen, "blue", pos, 5)

    # Draw the main circle
    pygame.draw.circle(screen, "red", player_pos, 40)
    
    # Update position and velocity
    player_pos.x += velocity * dt
    velocity += ACCELERATION * dt

    # flip() the display to put your work on screen
    pygame.display.flip()

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000

pygame.quit()