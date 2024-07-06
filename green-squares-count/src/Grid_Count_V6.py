import cv2
import pygame
import sys
import os

# Initialize Pygame
pygame.init()

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# Circle properties
circle_radius = 5
circles = []

# Font
font = pygame.font.Font(None, 36)

# Clock for controlling the frame rate
clock = pygame.time.Clock()

# Initialize video capture
cap = cv2.VideoCapture('rtsp://192.168.1.120:8554/video0_unicast')

# Create directory for captured frames if it doesn't exist
if not os.path.exists('captured_frames'):
    os.makedirs('captured_frames')

# Variables for captured frame
captured_frame = None
counter = 0  # Initialize counter
frame_number = 0

# Initialize second window
frame_screen = None

# Initialize main screen size
main_screen = pygame.display.set_mode((640, 480))  # Default size, will update based on video frame
pygame.display.set_caption('Algae detection')

def handle_pygame_events():
    global circles, counter, captured_frame, frame_screen, main_screen
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            cap.release()
            pygame.quit()
            sys.exit()

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_d and captured_frame is not None:
                circles = []  # Reset circles
                counter = 0  # Reset counter
            elif event.key == pygame.K_r and frame_screen:
                captured_frame = None
                circles = []  # Reset circles
                counter = 0  # Reset counter
                frame_screen = None
                main_screen = pygame.display.set_mode((frame.shape[1], frame.shape[0]))
                pygame.display.set_caption('Algae detection')
            elif event.key == pygame.K_f:
                # Capture the frame
                global frame_number
                frame_number += 1
                frame_path = f'captured_frames/frame_{frame_number}.jpg'
                cv2.imwrite(frame_path, frame)
                captured_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_screen = pygame.display.set_mode((captured_frame.shape[1], captured_frame.shape[0]))
                pygame.display.set_caption(f'Captured Frame {frame_number}')
                main_screen = None
            elif event.key == pygame.K_l and captured_frame is not None:
                modified_frame = captured_frame.copy()
                for circle in circles:
                    cv2.circle(modified_frame, circle, circle_radius, RED, -1)
                # Add the counter to the modified frame
                cv2.putText(modified_frame, f"Counter: {counter}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, RED, 2, cv2.LINE_AA)
                save_path = f'captured_frames/modified_frame_{frame_number}.jpg'
                cv2.imwrite(save_path, cv2.cvtColor(modified_frame, cv2.COLOR_RGB2BGR))
                print(f'Saved modified frame as {save_path}')

        if event.type == pygame.MOUSEBUTTONDOWN and captured_frame is not None:
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

# Main game loop
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    frame = cv2.flip(frame, 1)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    handle_pygame_events()

    if main_screen:
        # Update main screen size based on video frame
        main_screen = pygame.display.set_mode((frame.shape[1], frame.shape[0]))
        frame_pygame = pygame.image.frombuffer(frame_rgb.tobytes(), frame_rgb.shape[:2][::-1], "RGB")

        # Draw everything in the main window
        main_screen.fill(WHITE)
        main_screen.blit(frame_pygame, (0, 0))
        pygame.display.flip()

    if frame_screen:
        # Draw the captured frame and circles in the separate window
        frame_screen.fill(WHITE)
        frame_screen.blit(pygame.image.frombuffer(captured_frame.tobytes(), captured_frame.shape[:2][::-1], "RGB"), (0, 0))

        # Draw circles
        for circle in circles:
            pygame.draw.circle(frame_screen, RED, circle, circle_radius)

        # Draw counter
        counter_text = font.render(f"Counter: {counter}", True, RED)
        frame_screen.blit(counter_text, (10, 10))

        pygame.display.flip()

    # Cap the frame rate
    clock.tick(60)  # Adjust the frame rate as needed
