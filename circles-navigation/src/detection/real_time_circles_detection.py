from circles_detector import CirclesDetector
from multiprocessing import Process, Queue
import cv2

#MAIN_CAMERA = "rtsp://192.168.1.120:8554/video2_unicast"

CAMERA_IP = "rtsp://192.168.1.120:8554/video0_unicast"
#CAMERA_IP = 0
detector = CirclesDetector()


def capture_frames(queue, camera_ip):
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

            _, circles = detector.detect(frame, color_correct=False)
            frame = detector.draw_circles(frame, circles)

            key = cv2.waitKey(1) & 0xFF

            frame = cv2.resize(frame, (1400, 960))
            cv2.imshow("feed", frame)
            
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

