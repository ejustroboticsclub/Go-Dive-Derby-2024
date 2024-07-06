import pygame
import sys

# Initialize pygame
pygame.init()

# Set up display
width, height = 650, 350
map_width, map_height = 400, 200
door_width = 200
door_height = 50
map_x, map_y = 50, 50  # Adjusted map position
window = pygame.display.set_mode((width, height))
pygame.display.set_caption('Underwater Tomb Mapping')

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)

# Define item positions
item_positions = {'coffin': None, 'treasure': None, 'roll1': None, 'roll2': None}

# Define buttons
buttons = {
    'coffin': pygame.Rect(500, 50, 120, 50),  
    'treasure': pygame.Rect(500, 120, 120, 50),  
    'roll1': pygame.Rect(500, 190, 120, 50),  
    'roll2': pygame.Rect(500, 260, 120, 50),  
}

selected_item = None

# Fonts
font = pygame.font.SysFont(None, 36)

# Function to draw a cuboid (coffin)
def draw_cuboid(surface, x, y, width=100, height=25, depth=20):
    """
    Draws a 3D cuboid to represent a coffin.

    Args:
        surface (pygame.Surface): The surface to draw on.
        x (int): The x-coordinate of the front face top-left corner.
        y (int): The y-coordinate of the front face top-left corner.
        width (int, optional): The width of the cuboid. Default is 100.
        height (int, optional): The height of the cuboid. Default is 25.
        depth (int, optional): The depth of the cuboid. Default is 20.
    """
    pygame.draw.rect(surface, BLUE, (x, y, width, height))  # Front face
    pygame.draw.polygon(surface, BLUE, [(x, y), (x + depth, y - depth), (x + width + depth, y - depth), (x + width, y)])  # Top face
    pygame.draw.polygon(surface, BLUE, [(x + width, y), (x + width + depth, y - depth), (x + width + depth, y + height - depth), (x + width, y + height)])  # Side face

# Function to draw a cube (treasure)
def draw_cube(surface, x, y, size=40):
    """
    Draws a 3D cube to represent a treasure chest.

    Args:
        surface (pygame.Surface): The surface to draw on.
        x (int): The x-coordinate of the front face top-left corner.
        y (int): The y-coordinate of the front face top-left corner.
        size (int, optional): The size of the cube. Default is 40.
    """
    pygame.draw.rect(surface, RED, (x, y, size, size))  # Front face
    pygame.draw.polygon(surface, RED, [(x, y), (x + size / 2, y - size / 2), (x + size + size / 2, y - size / 2), (x + size, y)])  # Top face
    pygame.draw.polygon(surface, RED, [(x + size, y), (x + size + size / 2, y - size / 2), (x + size + size / 2, y + size - size / 2), (x + size, y + size)])  # Side face

# Function to draw a circle (papyrus roll)
def draw_circle(surface, x, y, radius=15):
    """
    Draws a circle to represent a papyrus roll.

    Args:
        surface (pygame.Surface): The surface to draw on.
        x (int): The x-coordinate of the circle's center.
        y (int): The y-coordinate of the circle's center.
        radius (int, optional): The radius of the circle. Default is 15.
    """
    pygame.draw.circle(surface, GREEN, (x + radius, y + radius), radius)

# Main loop
running = True
while running:
    window.fill(WHITE)

    # Draw the map area (excluding the door area)
    pygame.draw.line(window, BLACK, (map_x, map_y), (map_x + map_width, map_y), 2)  # Top border
    pygame.draw.line(window, BLACK, (map_x, map_y), (map_x, map_y + map_height), 2)  # Left border
    pygame.draw.line(window, BLACK, (map_x + map_width, map_y), (map_x + map_width, map_y + map_height), 2)  # Right border
    pygame.draw.line(window, BLACK, (map_x, map_y + map_height), (map_x + map_width - door_width, map_y + map_height), 2)  # Bottom border left part

    # Draw the items
    for item, pos in item_positions.items():
        if pos:
            if item == 'coffin':
                draw_cuboid(window, pos[0], pos[1])
            elif item == 'treasure':
                draw_cube(window, pos[0], pos[1])
            elif item == 'roll1' or item == 'roll2':
                draw_circle(window, pos[0], pos[1])

    # Draw buttons
    for item, rect in buttons.items():
        pygame.draw.rect(window, GREEN if item == selected_item else RED, rect)
        text = font.render(item.capitalize(), True, BLACK)
        window.blit(text, (rect.x + 10, rect.y + 10))

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()

            # Check button clicks
            for item, rect in buttons.items():
                if rect.collidepoint(pos):
                    selected_item = item
                    break
            else:
                # Place item on the map
                if selected_item:
                    if (map_x <= pos[0] <= map_x + map_width and map_y <= pos[1] <= map_y + map_height) and not (map_x + map_width - door_width <= pos[0] <= map_x + map_width and map_y + map_height - door_height <= pos[1] <= map_y + map_height):
                        item_positions[selected_item] = (pos[0] - 25, pos[1] - 15)
                        selected_item = None

    pygame.display.flip()

pygame.quit()
sys.exit()