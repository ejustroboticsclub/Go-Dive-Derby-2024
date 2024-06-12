import cv2
import numpy as np
from color_correct import correct

DELTA_X_THRESHOLD = 10
DELTA_Y_THRESHOLD = 10
STOP_RATIO_THRESHOLD = 0.8

class CirclesDetector:
    def __init__(self):
        pass

    def detect(self, frame: np.ndarray, color_correct: bool = False) -> tuple[int, np.ndarray]:
        '''
        Detect circles in a frame.
        Input:
            frame: the frame(BGR format)
            color_correct: whether to perform color correction on the image
        Returns:
            (int, np.ndarray): the number of circles detected and the circles themselves if any
            The circles are represented as a numpy array of shape (n, 3) where n is the number of circles detected,
            and each circle is represented as a list of 3 elements: (x, y, r) where x and y are the coordinates of the center of the circle and r is the radius of the circle
        '''

        # Perform color correction if necessary
        if color_correct:
            frame = correct(frame)

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply a median blur to the grayscale image
        gray = cv2.medianBlur(gray, 5)

        # Detect circles in the image using the Hough Circles method
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=200,
            param1=50,
            param2=30,
            minRadius=1,
            maxRadius=350,
        )

        # Check if any circles were detected
        if circles is not None:
            # Convert the (x, y, r) coordinates to integers
            circles = np.uint16(np.around(circles))
            # Return the number of circles detected and the circles themselves
            return (len(circles[0]), circles[0])
        else:
            # Return 0 if no circles were detected
            return (0, None)

    def draw_circles(self, frame: np.ndarray, circles: np.ndarray) -> np.ndarray:
        '''
        Draw circles on a frame.
        Input:
            frame: the frame(BGR format)
            circles: the circles to draw
        Returns:
            np.ndarray: the frame with the circles drawn on it(BGR format)
        '''

        # Make a copy of the frame
        output = frame.copy()

        # Draw the circles on the output frame
        if circles is not None:
            for circle in circles:
                # Extract the coordinates and radius of the circle
                x, y, r = circle
                # Draw the circle
                cv2.circle(output, (x, y), r, (0, 255, 0), 2)
                # Draw a small circle (of radius 1) to show the center.
                cv2.circle(output, (x, y), 1, (0, 0, 255), 3)

        return output

    def sort_circles(self, frame_area: int, circles: np.ndarray) -> list[tuple[np.ndarray, float]]:
        '''
        Sort circles based on the ratio of the bounding box area to the frame area.
        Input:
            frame_area: the area of the frame
            circles: the circles to sort
        Returns:
            list: the sorted circles along with the ratio of the bounding box area to the frame area
        '''

        # Calculate the area of the bounding box for each circle and compute the ratio of the bounding box area to the frame area
        circles_ratios = []
        for circle in circles:
            _, _, r = circle
            # The bounding box is a square with side length equal to the diameter of the circle
            bounding_box_area = (2 * r) ** 2
            # The ratio is the bounding box area divided by the frame area
            ratio = bounding_box_area / frame_area
            # Append the circle and the ratio to the list
            circles_ratios.append((circle, ratio))

        # Sort circles based on the ratio
        circles_sorted = sorted(
            circles_ratios, key=lambda x: x[1], reverse=True)

        return circles_sorted

    def identify_circles_orientation(self, frame: np.ndarray, color_correct: bool = False) -> tuple[str, str]:
        '''
        Identify the orientation of the circles in the frame.
        Input:
            frame: the frame(BGR format)
            color_correct: whether to perform color correction on the image
        Returns:
            if no circles are detected, returns "no circles detected"
            if less than 3 circles are detected, returns "not enough circles detected"
            if more than 3 circles are detected, returns "too many circles detected"
            otherwise, returns a tuple[str, str]: the directions to go after passing the first and second circles respectively
        '''

        # Detect circles in the frame
        num_circles, circles = self.detect(frame, color_correct)

        # Check the number of circles detected
        if num_circles == 0:
            return "no circles detected"
        elif num_circles < 3:
            return "not enough circles detected"
        elif num_circles > 3:
            return "too many circles detected"
        else:
            # Sort the circles based on the ratio of the bounding box area to the frame area
            circles_sorted = self.sort_circles(
                frame.shape[0] * frame.shape[1], circles)

            # Extract only the circles from the sorted list
            sorted_circles = [circle for circle, _ in circles_sorted]

            # Find the positions of the circles along the x-axis
            x_positions = [circle[0] for circle in sorted_circles]

            # Determine the direction to go after passing the closest circle
            if x_positions[0] < x_positions[1]:
                direction_after_first_circle = "right"
            else:
                direction_after_first_circle = "left"

            # Determine the direction to go after passing the second circle
            if x_positions[1] < x_positions[2]:
                direction_after_second_circle = "right"
            else:
                direction_after_second_circle = "left"

            return (direction_after_first_circle, direction_after_second_circle)

    def determine_direction(self, frame: np.ndarray, color_correct: bool = False) -> tuple[tuple[int, int], tuple[int, int], bool]:
        '''
        Determine the direction to move based on the position of the closest circle in the frame.
        Input:
            frame: the frame(BGR format)
            color_correct: whether to perform color correction on the image
        Returns:
            if no circles are detected, returns "no circles detected"
            otherwise, returns a tuple[tuple[int, int], tuple[int, int], bool]: the horizontal and vertical distances between the circle center and the image center, the direction to move in, and a boolean indicating whether to stop
            the stop is triggered when the circle is close to the center and the ratio of the bounding box area to the frame area is greater than or equal to STOP_RATIO_THRESHOLD 
        '''

        # Detect circles in the frame
        num_circles, circles = self.detect(frame, color_correct)

        # Check if any circles were detected
        if num_circles == 0:
            return "no circles detected"

        # Get the closest circle
        if num_circles != 1:
            # Sort the circles based on the ratio of the bounding box area to the frame area
            circles_sorted = self.sort_circles(
                frame.shape[0] * frame.shape[1], circles)
            # Extract the closest circle from the sorted list
            closest_circle = circles_sorted[0][0]
        else:
            closest_circle = circles[0]

        # Get the center of the closest circle
        circle_center = (closest_circle[0], closest_circle[1])

        # Get the radius of the closest circle
        r = closest_circle[2]

        # Calculate the area of the frame
        frame_area = frame.shape[0] * frame.shape[1]

        # Calculate the area of the bounding box for the circle
        bounding_box_area = (2 * r) ** 2

        # Calculate the ratio of the bounding box area to the frame area
        ratio = bounding_box_area / frame_area

        # Get the center of the image
        frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)

        # Calculate the horizontal and vertical distances between the circle center and the image center
        delta_x = circle_center[0] - frame_center[0]
        delta_y = circle_center[1] - frame_center[1]

        # Check if the circle is close to the center
        if np.abs(delta_x) <= DELTA_X_THRESHOLD and np.abs(delta_y) <= DELTA_Y_THRESHOLD:
            # Check if the circle is close to the center in both directions
            if ratio >= STOP_RATIO_THRESHOLD:
                return (delta_x, delta_y, True)
        return (delta_x, delta_y, False)
