import cv2
import numpy as np

# Step 1: Read the image
image = cv2.imread('test2.jpg')

# Step 2: Convert to HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Step 3: Create a mask for green color
lower_green = np.array([35, 30, 30])
upper_green = np.array([85, 255, 255])
mask = cv2.inRange(hsv, lower_green, upper_green)

# Step 4: Apply morphological operations
kernel = np.ones((9, 9), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

cv2.imshow('Green Grid Squares', mask)
cv2.waitKey(0)

# Step 5: Find contours in the mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Step 6: Filter contours to identify grid squares
grid_squares = []
num_squares = 0
for contour in contours:
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    area = cv2.contourArea(contour)

    if 10 < area < 500000:  
        if len(approx) < 8:
            grid_squares.append(contour)
            num_squares+= 1
        else:
            grid_squares.append(contour)
            num_squares += len(approx) / 4
    else:
        print(area)




# Step 7: Count the squares

print("Number of green grid squares:", num_squares)

# Optionally, draw the contours on the image and display it
output_image = image.copy()
cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
cv2.drawContours(output_image, grid_squares, -1, (0, 255, 0), 3)
cv2.imshow('Green Grid Squares2', image)
cv2.imshow('Green Grid Squares', output_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
