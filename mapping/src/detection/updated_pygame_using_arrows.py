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
TOMB_COLOR = (127, 205, 255)

# Define item positions
item_positions = {'coffin': None, 'treasure': None, 'roll1': None, 'roll2': None}

# Define buttons
buttons = {
    'coffin': pygame.Rect(500, 50, 120, 50),  
    'treasure': pygame.Rect(500, 120, 120, 50),  
    'roll1': pygame.Rect(500, 190, 120, 50),  
    'roll2': pygame.Rect(500, 260, 120, 50),  
}

# Define items for selection
items_list = ['coffin', 'treasure', 'roll1', 'roll2']
selected_item = None
item_index = 0

# Fonts
font = pygame.font.SysFont(None, 36)

# Define ROV properties
rov_size = 20
rov_x = map_x + map_width - door_width + (door_width - rov_size) // 2
rov_y = map_y + map_height - rov_size
rov_speed = 5

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

# Function to draw the ROV
def draw_rov(surface, x, y):
    """
    Draws a square to represent the ROV (Remotely Operated Vehicle).

    Args:
        surface (pygame.Surface): The surface to draw on.
        x (int): The x-coordinate of the top-left corner of the ROV.
        y (int): The y-coordinate of the top-left corner of the ROV.
    """
    pygame.draw.rect(surface, BLACK, (x, y, rov_size, rov_size))

# Main loop
running = True
item_placing_mode = False

while running:
    window.fill(WHITE)

    # Draw the map area (excluding the door area)
    pygame.draw.rect(window, TOMB_COLOR, (map_x, map_y, map_width, map_height))
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

    # Draw ROV
    draw_rov(window, rov_x, rov_y)

    # Draw item selection indicator if in placing mode
    if item_placing_mode:
        selected_item = items_list[item_index]
        item_text = font.render(f'Selecting: {selected_item}', True, BLACK)
        window.blit(item_text, (map_x, map_y + map_height + 10))

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                rov_y -= rov_speed
            elif event.key == pygame.K_DOWN:
                rov_y += rov_speed
            elif event.key == pygame.K_LEFT:
                rov_x -= rov_speed
            elif event.key == pygame.K_RIGHT:
                rov_x += rov_speed
            elif event.key == pygame.K_RETURN:
                if not item_placing_mode:
                    item_placing_mode = True
                else:
                    item_positions[selected_item] = (rov_x, rov_y)
                    item_placing_mode = False
            elif event.key == pygame.K_TAB and item_placing_mode:
                item_index = (item_index + 1) % len(items_list)

    # Constrain ROV within map boundaries
    rov_x = max(map_x, min(rov_x, map_x + map_width - rov_size))
    rov_y = max(map_y, min(rov_y, map_y + map_height - rov_size))

    pygame.display.flip()

pygame.quit()
sys.exit()