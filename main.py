# Modules
import pygame
import math

# Init
pygame.init()

# Clock
clock = pygame.time.Clock()
FPS = 60

# Constants
BG_COLOR = (0, 0, 0)
FULLSCREEN = True
DEFAULT_WIDTH = 1000
DEFAULT_HEIGHT = 600
WIDTH = pygame.display.Info().current_w if FULLSCREEN == True else DEFAULT_WIDTH
HEIGHT = pygame.display.Info().current_h if FULLSCREEN == True else DEFAULT_HEIGHT
print(WIDTH, HEIGHT)
screen = pygame.display.set_mode((WIDTH, HEIGHT), vsync=1)
running = True

# Base pendulum parameters
BASE_LENGTH = int(WIDTH * 0.1)
PENDULUM_WIDTH_RATIO = 0.002
BOB_RADIUS = int(WIDTH * 0.01)

# Pendulum parameters
FIRST_P_LENGTH = BASE_LENGTH
SECOND_P_LENGTH = BASE_LENGTH
FIRST_P_WIDTH = int(WIDTH * PENDULUM_WIDTH_RATIO)
SECOND_P_WIDTH = int(WIDTH * PENDULUM_WIDTH_RATIO)
FIRST_P_THETA = math.pi/2
SECOND_P_THETA = math.pi/2

FIRST_P_MASS = 1
SECOND_P_MASS = 1
FIRST_THETA_DOT = 0
SECOND_THETA_DOT = 0

FIRST_P_COLOR = (255, 255, 255)
SECOND_P_COLOR = (255, 255, 255)

# Pendulum constants
GRAVITY = 9.81
DAMPING_FACTOR = 1

def compute_accelerations(theta1, theta2, theta1_dot, theta2_dot):
    num1 = -GRAVITY * (2 * FIRST_P_MASS + SECOND_P_MASS) * math.sin(theta1)
    num2 = -SECOND_P_MASS * GRAVITY * math.sin(theta1 - 2 * theta2)
    num3 = -2 * math.sin(theta1 - theta2) * SECOND_P_MASS
    num4 = theta2_dot ** 2 * SECOND_P_LENGTH + theta1_dot ** 2 * FIRST_P_LENGTH * math.cos(theta1 - theta2)
    den = FIRST_P_LENGTH * (2 * FIRST_P_MASS + SECOND_P_MASS - SECOND_P_MASS * math.cos(2 * theta1 - 2 * theta2))
    theta1_ddot = (num1 + num2 + num3 * num4) / den

    num5 = 2 * math.sin(theta1 - theta2)
    num6 = (theta1_dot ** 2 * FIRST_P_LENGTH * (FIRST_P_MASS + SECOND_P_MASS))
    num7 = GRAVITY * (FIRST_P_MASS + SECOND_P_MASS) * math.cos(theta1)
    num8 = theta2_dot ** 2 * SECOND_P_LENGTH * SECOND_P_MASS * math.cos(theta1 - theta2)
    den2 = SECOND_P_LENGTH * (2 * FIRST_P_MASS + SECOND_P_MASS - SECOND_P_MASS * math.cos(2 * theta1 - 2 * theta2))
    theta2_ddot = (num5 * (num6 + num7 + num8)) / den2

    return theta1_ddot, theta2_ddot

def rk4_step(damping, theta1, theta2, theta1_dot, theta2_dot, dt):
    # Initial accelerations
    a1, b1 = compute_accelerations(theta1, theta2, theta1_dot, theta2_dot)
    
    # Compute k1
    k1_theta1_dot = dt * a1
    k1_theta2_dot = dt * b1
    k1_theta1 = dt * theta1_dot
    k1_theta2 = dt * theta2_dot
    
    # Compute k2
    a2, b2 = compute_accelerations(theta1 + 0.5 * k1_theta1, theta2 + 0.5 * k1_theta2, theta1_dot + 0.5 * k1_theta1_dot, theta2_dot + 0.5 * k1_theta2_dot)
    k2_theta1_dot = dt * a2
    k2_theta2_dot = dt * b2
    k2_theta1 = dt * (theta1_dot + 0.5 * k1_theta1_dot)
    k2_theta2 = dt * (theta2_dot + 0.5 * k1_theta2_dot)

    # Compute k3
    a3, b3 = compute_accelerations(theta1 + 0.5 * k2_theta1, theta2 + 0.5 * k2_theta2, theta1_dot + 0.5 * k2_theta1_dot, theta2_dot + 0.5 * k2_theta2_dot)
    k3_theta1_dot = dt * a3
    k3_theta2_dot = dt * b3
    k3_theta1 = dt * (theta1_dot + 0.5 * k2_theta1_dot)
    k3_theta2 = dt * (theta2_dot + 0.5 * k2_theta2_dot)

    # Compute k4
    a4, b4 = compute_accelerations(theta1 + k3_theta1, theta2 + k3_theta2, theta1_dot + k3_theta1_dot, theta2_dot + k3_theta2_dot)
    k4_theta1_dot = dt * a4
    k4_theta2_dot = dt * b4
    k4_theta1 = dt * (theta1_dot + k3_theta1_dot)
    k4_theta2 = dt * (theta2_dot + k3_theta2_dot)

    # Final values
    new_theta1_dot = theta1_dot + (1/6) * (k1_theta1_dot + 2*k2_theta1_dot + 2*k3_theta1_dot + k4_theta1_dot)
    new_theta2_dot = theta2_dot + (1/6) * (k1_theta2_dot + 2*k2_theta2_dot + 2*k3_theta2_dot + k4_theta2_dot)
    new_theta1 = theta1 + (1/6) * (k1_theta1 + 2*k2_theta1 + 2*k3_theta1 + k4_theta1)
    new_theta2 = theta2 + (1/6) * (k1_theta2 + 2*k2_theta2 + 2*k3_theta2 + k4_theta2)

    # Damping
    new_theta1_dot *= damping
    new_theta2_dot *= damping
    
    return new_theta1, new_theta2, new_theta1_dot, new_theta2_dot

# Pendulum class
class Pendulum:
    def __init__(self, length, theta, mass, color, width, x_pos, y_pos):
        self.length = length
        self.width = width
        self.theta = theta
        self.mass = mass
        self.color = color
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.end_x = self.x_pos + self.length * math.sin(self.theta)
        self.end_y = self.y_pos + self.length * math.cos(self.theta)

    def update_position(self, theta):
        self.theta = theta
        self.end_x = self.x_pos + self.length * math.sin(self.theta)
        self.end_y = self.y_pos + self.length * math.cos(self.theta)

    def draw(self):
        pygame.draw.line(screen, self.color, (self.x_pos, self.y_pos), (self.end_x, self.end_y), self.width)
        self.circ = pygame.draw.circle(screen, self.color, (self.end_x, self.end_y), BOB_RADIUS)

# Main loop
pendulum_1 = Pendulum(FIRST_P_LENGTH, FIRST_P_THETA, FIRST_P_MASS, FIRST_P_COLOR, FIRST_P_WIDTH, WIDTH / 2, HEIGHT/2)
pendulum_2 = Pendulum(SECOND_P_LENGTH, SECOND_P_THETA, SECOND_P_MASS, SECOND_P_COLOR, SECOND_P_WIDTH, pendulum_1.end_x, pendulum_1.end_y)
positions = []
POSITION_LIMIT = int(FPS * 2)
TRAIL_SIZE = int(WIDTH * 0.005)
ENABLE_TRAIL = True
while running:
    for event in pygame.event.get():
        mouse_pos_x, mouse_pos_y = pygame.mouse.get_pos()
        if event.type == pygame.QUIT:
            running = False

    dt = 0.28
    FIRST_P_THETA, SECOND_P_THETA, FIRST_THETA_DOT, SECOND_THETA_DOT = rk4_step(DAMPING_FACTOR, FIRST_P_THETA, SECOND_P_THETA, FIRST_THETA_DOT, SECOND_THETA_DOT, dt)
    pendulum_1.update_position(FIRST_P_THETA)
    pendulum_2.x_pos, pendulum_2.y_pos = pendulum_1.end_x, pendulum_1.end_y
    pendulum_2.update_position(SECOND_P_THETA)
    screen.fill(BG_COLOR)
    pendulum_1.draw()
    pendulum_2.draw()
    if ENABLE_TRAIL:
        positions.append((pendulum_2.end_x, pendulum_2.end_y))
        if len(positions) > POSITION_LIMIT:
            positions.remove(positions[0])
        for i in range(len(positions)):
            pygame.draw.rect(screen, pendulum_2.color, (pygame.Rect(positions[i][0], positions[i][1], TRAIL_SIZE, TRAIL_SIZE)))
    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
