from multiprocessing import Process, Queue
from shape_detection import ShapeDetector
import cv2
import datetime

CAMERA_IP = "rtsp://192.168.1.120:8554/video0_unicast"
#CAMERA_IP = 0

detector = ShapeDetector()

def capture_frames(queue, camera_ip):
    global cap
    cap = cv2.VideoCapture(camera_ip)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if not queue.full():
            queue.put(frame)

        # Check if the queue needs to be stopped
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()

def detect_and_display(queue):
    while True:
        if not queue.empty():
            frame = queue.get()
            detections = detector.get_detections(frame)
            topic = detector.get_topic(frame, detections)
            topic = str(topic)
            frame = detector.draw_detections(frame, detections)      

            # Specify the file path
            file_path = "/home/atef/shape.txt"

            # Open the file in write mode ('w')
            with open(file_path, 'w') as file:
                # Write the text content to the file
                print("TOPIC SD ISHD ", topic)
                file.write(topic)
            
            frame = cv2.resize(frame, (1400, 960))
            cv2.imshow("frame", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Quit the program
                break

    cv2.destroyAllWindows()

def main():
    queue = Queue(maxsize=1)
    
    p1 = Process(target=capture_frames, args=(queue, CAMERA_IP))
    p2 = Process(target=detect_and_display, args=(queue,))
    
    p1.start()
    p2.start()
    
    p1.join()
    p2.join()

if __name__ == "__main__":
    main()