import pygame
import sys

pygame.init()

# Screen settings
WIDTH, HEIGHT = 400, 600
ROAD_WIDTH = 240
ROAD_X = (WIDTH - ROAD_WIDTH) // 2
LANE_COUNT = 1
LANE_WIDTH = ROAD_WIDTH // LANE_COUNT

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Road Environment")
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 28)

# Colors
WHITE = (255, 255, 255)
GRAY = (64, 64, 64)
GREEN = (34, 177, 76)

# Distance tracking
distance = 0
line_offset = 0
SCROLL_SPEED = 4


def draw_road():
    global line_offset
    
    screen.fill(GREEN)
    pygame.draw.rect(screen, GRAY, (ROAD_X, 0, ROAD_WIDTH, HEIGHT))
    
    # Road outline (continuous white lines)
    pygame.draw.line(screen, WHITE, (ROAD_X, 0), (ROAD_X, HEIGHT), 4)
    pygame.draw.line(screen, WHITE, (ROAD_X + ROAD_WIDTH, 0), (ROAD_X + ROAD_WIDTH, HEIGHT), 4)
    
    # Animated lane lines
    line_offset += SCROLL_SPEED
    if line_offset >= 40:
        line_offset = 0
    
    for i in range(1, LANE_COUNT):
        x = ROAD_X + i * LANE_WIDTH
        for y in range(-40 + line_offset, HEIGHT, 40):
            pygame.draw.line(screen, WHITE, (x, y), (x, y + 20), 4)


def main():
    global distance
    running = True

    while running:
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Draw road
        draw_road()

        # Update distance
        distance += 1
        distance_text = font.render(f"Distance: {distance // 10}", True, WHITE)
        screen.blit(distance_text, (10, 10))

        pygame.display.update()
        clock.tick(60)


if __name__ == "__main__":
    main()