from multiprocessing import Process, Queue
from symbols_detection import SymbolsDetector
import cv2
import datetime

# Main camera IP address
CAMERA_IP = "rtsp://192.168.1.120:8554/video0_unicast"

# Initialize the symbol detector
detector = SymbolsDetector()

def capture_frames(queue, camera_ip):
    """
    Captures frames from the specified camera IP and puts them into the queue.
    
    Continuously captures frames from the RTSP stream URL of the camera and 
    puts them into a multiprocessing queue for further processing.
    
    Args:
        queue (Queue): The multiprocessing queue to store frames.
        camera_ip (str): The RTSP stream URL of the camera.
    """
    cap = cv2.VideoCapture(camera_ip)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            # Exit loop if frame capture fails
            break
        if not queue.full():
            # Put frame into the queue if it's not full
            queue.put(frame)

        # Check if the 'q' key is pressed to stop the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the video capture object
    cap.release()

def detect_and_display(queue):
    """
    Detects symbols in the frames from the queue and displays them with detections.
    
    Continuously retrieves frames from the multiprocessing queue, applies symbol detection,
    and displays the frames with detected symbols. Also handles saving frames and videos 
    upon certain key presses.
    
    Args:
        queue (Queue): The multiprocessing queue to retrieve frames.
    """
    COND = True
    counter = 1
    
    while True:
        if not queue.empty():
            frame = queue.get()
            
            if COND:
                # Initialize video writer for saving the video
                frame_height, frame_width, _ = frame.shape
                size = (frame_width, frame_height)
                now = datetime.datetime.now()
                video_name = "symbols_"
                video_name += now.strftime("%H-%M-%S") + ".mp4"
                saved_video = cv2.VideoWriter("../videos/" + video_name, cv2.VideoWriter_fourcc(*'MJPG'), 10, size)
                COND = False

            # Get predictions and draw detections on the frame
            results = detector.get_predictions(frame, preprocessing=False)
            frame = detector.draw_detections(frame, results)

            # Check if the space key is pressed to save a frame
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):  # Space key is pressed
                now = datetime.datetime.now()
                image_name = "symbols_"
                image_name += now.strftime("%H-%M-%S") + ".jpg"
                cv2.imwrite("../images/" + image_name, frame)
                counter += 1
            
            # Write the frame to the video file
            saved_video.write(frame)
            
            # Resize frame for display and show it
            frame = cv2.resize(frame, (1400, 960))
            cv2.imshow("feed", frame)
            
            # Check if the 'q' key is pressed to quit the program
            if key == ord('q'):
                break

    # Destroy all OpenCV windows
    cv2.destroyAllWindows()

def main():
    """
    Main function to set up multiprocessing for frame capture and symbol detection.
    
    Initializes a multiprocessing queue and starts two processes: one for capturing frames 
    from the camera and another for detecting and displaying symbols in the frames.
    """
    queue = Queue(maxsize=1)
    
    # Start frame capture process
    p1 = Process(target=capture_frames, args=(queue, CAMERA_IP))
    
    # Start symbol detection and display process
    p2 = Process(target=detect_and_display, args=(queue,))
    
    p1.start()
    p2.start()
    
    # Wait for both processes to complete
    p1.join()
    p2.join()

if __name__ == "__main__":
    main()
