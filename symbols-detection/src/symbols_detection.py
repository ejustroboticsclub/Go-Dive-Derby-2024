from ultralytics import YOLO
import cv2
import numpy as np

MODEL_PATH = "/home/atef/gdd/models/symbols_model.pt"

class SymbolsDetector():
    def __init__(self):
        self.model = YOLO(MODEL_PATH)

    def image_preprocessing(self, frame):
        """
        Preprocess an image by converting it to grayscale, applying adaptive thresholding, 
        extracting a region of interest (ROI), and rotating the image.

        Args:
        frame (ndarray): Input video frame in BGR format.

        Returns:
        frame (ndarray): The original frame, which currently does not reflect any preprocessing changes.
        """
        # Convert the image to grayscale
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply adaptive thresholding (adjust the parameters according to conditions)
        thresh1 = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 61, 5)
        
        # Convert the grayscale image with thresholding back to BGR format
        gray_img_3channel = cv2.cvtColor(thresh1, cv2.COLOR_GRAY2BGR)
        
        # Crops the Parts of the gripper that confuses the model, used with the thresholding (uncomment if need)
        # roi = gray_img_3channel[0:435, 220:640]
        
        # Rotate the image 90 degrees clockwise (uncomment if needed)
        # rotated = cv2.rotate(gray_img_3channel, cv2.ROTATE_90_CLOCKWISE)
        
        # Note: The processed image is not being returned or used further.
        return gray_img_3channel
    
    def get_predictions(self, frame, preprocessing = False):
        if preprocessing:
            frame = self.image_preprocessing(frame)

        results = self.model.track(frame, persist=True)
        return results


    def draw_detections(self, frame, results):
        """
        Process a single video frame using the YOLO model to perform object tracking, 
        then display and write the processed frame to a video file.

        Args:
        frame (ndarray): The input video frame in BGR format.
        result (cv2.VideoWriter): VideoWriter object to write the processed frames to a video file.


        Returns:
        frame
        """
        
        # Plot the tracking results on the frame
        frame = results[0].plot()

        return frame
    
