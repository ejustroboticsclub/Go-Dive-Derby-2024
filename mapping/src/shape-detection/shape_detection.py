import cv2
from ultralytics import YOLO

MODEL_PATH = 'best.pt'
RATIO_THRESHOLD = 0

class ShapeDetector:
    def __init__(self):
        self.model = self.load_model(MODEL_PATH)
        
    def load_model(self, MODEL_PATH):
        model = YOLO(MODEL_PATH)
        return model
    
    def get_topic(self, frame, detections):

        #detections = self.get_detections(frame)
        detected_shapes = []

        if detections is not None:
            class_names = self.model.names
            for detection in detections:
                x1, y1, x2, y2, score, class_id = detection
                class_id = int(class_id)
                
                # Calculate the area of the bounding box
                width = x2 - x1
                height = y2 - y1
                area_shape = width * height
                frame_height, frame_width, _ = frame.shape
                area_frame = frame_height*frame_width
                ratio = (area_shape/area_frame)
                if ratio >= RATIO_THRESHOLD:
                    detected_shapes.append(class_names[class_id])
        else:
            return None
        if len(detected_shapes) > 1:
            return None
        elif len(detected_shapes) == 1:
            return detected_shapes[0]
        else:
            return None

    def draw_detections(self, frame, detections):
        if detections is not None:         
            class_names = self.model.names
            for detection in detections:
                x1, y1, x2, y2, score, class_id = detection
                class_id = int(class_id)
                label = f"{class_names[class_id]}: {score:.2f}" 
    
                # Calculate the area of the bounding box
                width = x2 - x1
                height = y2 - y1
                area_shape = width * height
                frame_height, frame_width, _ = frame.shape
                area_frame = frame_height*frame_width
                ratio = (area_shape/area_frame)
    
                if ratio >= RATIO_THRESHOLD:
                    # Draw bounding box
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    # Draw label
                    cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)
        return frame
    
    def get_detections(self,frame):        
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Run the model inference
        results = self.model(img)

        # Extract results
        detections = results[0].boxes.data.cpu().numpy()
        if detections.shape[0] == 0:
            detections = None 

        return detections
    
    def get_class_names(self):
        return self.model.names