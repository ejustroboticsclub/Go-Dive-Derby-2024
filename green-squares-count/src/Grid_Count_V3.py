import cv2
import pygame
import sys
import numpy as np

# Initialize Pygame
pygame.init()

# Set up display
WIDTH, HEIGHT = 1000, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('Algae Percentage')

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# Circle properties
circle_radius = 10
circles = []

# Font
font = pygame.font.Font(None, 36)

# Clock for controlling the frame rate
clock = pygame.time.Clock()

def black_lines(image):
    # Convert image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise and detail
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply adaptive thresholding to create a binary image
    thresh = cv2.adaptiveThreshold(blur, 255, 1, 1, 7, 1)
    thresh = cv2.adaptiveThreshold(thresh, 255, 1, 1, 91, 4)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    c = 0
    for i in contours:
        area = cv2.contourArea(i)
        if area > 50:
            # Draw contours on the original image in black
            cv2.drawContours(image, contours, c, (0, 0, 0), 4)
        c += 1


# Load the image
image = cv2.imread('4.jpg')

# Resize image if it is too large
h, w, *_ = image.shape
if h > 700 or w > 700:
    image = cv2.resize(image, (0, 0), fx=0.4, fy=0.4)

# Apply the black_lines function to detect and draw black lines
black_lines(image)

# Convert image to HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define range for green color in HSV space
lower_green = np.array([35, 30, 30])
upper_green = np.array([85, 255, 255])
# Create a mask for green color
mask = cv2.inRange(hsv, lower_green, upper_green)

# Apply morphological opening to remove small noise
kernel = np.ones((5, 5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

# Find contours in the green mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

grid_squares = []
num_squares = 0
for contour in contours:
    # Approximate the contour to a polygon
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    area = cv2.contourArea(contour)

    # Filter out small or excessively large areas
    if 300 < area < 500000:
        grid_squares.append(approx)
        num_squares += 1

# Print the number of detected green grid squares
print("Number of green grid squares:", num_squares)

# Draw contours of detected grid squares on the original image in red
cv2.drawContours(image, grid_squares, -1, (0, 0, 255), 3)

# Convert the OpenCV image to Pygame surface
image_pygame = pygame.image.frombuffer(image.tobytes(), image.shape[:2][::-1], "RGB")

# Counter
counter = num_squares

# Main game loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            if event.button == 1:  # Left mouse button
                circles.append(pos)
                counter += 1
            elif event.button == 3:  # Right mouse button
                removed = False
                for circle in circles:
                    if (circle[0] - pos[0]) ** 2 + (circle[1] - pos[1]) ** 2 < circle_radius ** 2:
                        circles.remove(circle)
                        removed = True
                        break
                if removed:
                    counter -= 1

    # Game logic (update game state)

    # Draw everything
    screen.fill(WHITE)

    # Draw the OpenCV image
    screen.blit(image_pygame, (0, 0))

    # Draw circles
    for circle in circles:
        pygame.draw.circle(screen, RED, circle, circle_radius)

    # Draw countera
    counter_text = font.render(f"Counter: {counter}", True, (255, 0, 0))
    screen.blit(counter_text, (500, 10))

    # Update the display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(60)
