from circles_detector import CirclesDetector
import cv2

detector = CirclesDetector()
frame = cv2.imread("../../images/circles_test5.jpg")
num_circles, circles = detector.detect(frame, color_correct=False)
frame = detector.draw_circles(frame, circles)
print(detector.identify_circles_orientation(frame, color_correct=False))
print(detector.sort_circles(frame.shape[0] * frame.shape[1], circles))
print(detector.determine_direction(frame, color_correct=False))
cv2.imshow("frame", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()