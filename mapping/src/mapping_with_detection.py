from shape_detection import ShapeDetector
import cv2
#import rospy
#from std_msgs.msg import Bool

CAMERA_IP = "rtsp://192.168.1.120:8554/video0_unicast"

shape_data = False

def shape_callback(msg):
    global shape_data
    shape_data = msg.data

def main():
    #rospy.init_node('yolo_object_detection', anonymous=True)
    #rospy.Subscriber('/ROV/shape', Bool, shape_callback)
    detector = ShapeDetector()
    pipeline = "rtspsrc location=" + CAMERA_IP + " latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink max-buffers=1 drop=True"
    
    cap = cv2.VideoCapture(CAMERA_IP)    

    while True:
        _,frame = cap.read()
        detections = detector.get_detections(frame)
        topic = detector.get_topic(frame, detections)
        if (topic is not None) and shape_data:
            #pub = rospy.Publisher(topic, Bool, queue_size=10)
            #pub.publish(True)
            pass
        
        frame = detector.draw_detections(frame, detections)

        cv2.imshow("feed", frame)

        if(cv2.waitKey(1) == ord('q')):
            break
        
    cap.release()

if __name__ == '__main__':
    main()


#"/ROV/shape"