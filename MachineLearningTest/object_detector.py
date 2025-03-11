import cv2
import numpy as np
import time
import torch
from ultralytics import YOLO

class ObjectDetector:
    """Class for object detection using YOLOv8"""
    
    def __init__(self, model_path=None, confidence=0.25):
        """
        Initialize object detector
        
        Args:
            model_path: Path to YOLO model file, if None uses yolov8n
            confidence: Detection confidence threshold
        """
        self.confidence = confidence
        
        # Load model - use pretrained YOLOv8n if no model specified
        try:
            if model_path:
                self.model = YOLO(model_path)
                print(f"Loaded custom YOLOv8 model from {model_path}")
            else:
                self.model = YOLO("yolov8n.pt")  # Uses default YOLOv8 nano model
                print("Loaded default YOLOv8n model")
            
            self.model_loaded = True
            
            # Default class names (COCO dataset)
            self.class_names = self.model.names
            
            # Define colors for visualization
            self.colors = {}
            for class_id in self.class_names:
                self.colors[class_id] = tuple(np.random.randint(0, 255, 3).tolist())
                
        except Exception as e:
            print(f"Error loading YOLOv8 model: {str(e)}")
            self.model_loaded = False
    
    def detect(self, frame):
        """
        Run object detection on a frame
        
        Args:
            frame: BGR image (numpy array)
            
        Returns:
            Tuple of (detections, annotated_frame) where:
                - detections is a list of dicts with detection info
                - annotated_frame is the input frame with bounding boxes drawn
        """
        if not self.model_loaded or frame is None:
            return [], frame
        
        # Start timing
        start_time = time.time()
        
        # Run detection
        results = self.model(frame, conf=self.confidence)
        
        # Process detections
        detections = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Get box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                
                # Get confidence and class
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                cls_name = self.class_names[cls_id]
                
                # Create detection object
                detection = {
                    'class_id': cls_id,
                    'class_name': cls_name,
                    'confidence': conf,
                    'box': [x1, y1, x2, y2],
                    'center': [(x1 + x2) // 2, (y1 + y2) // 2],
                    'size': [(x2 - x1), (y2 - y1)]
                }
                detections.append(detection)
        
        # Create annotated frame
        annotated_frame = frame.copy()
        for det in detections:
            x1, y1, x2, y2 = det['box']
            cls_id = det['class_id']
            label = f"{det['class_name']} {det['confidence']:.2f}"
            color = self.colors[cls_id]
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            
            # Draw label background
            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            cv2.rectangle(
                annotated_frame,
                (x1, y1 - text_size[1] - 5),
                (x1 + text_size[0], y1),
                color,
                -1
            )
            
            # Draw label text
            cv2.putText(
                annotated_frame,
                label,
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2
            )
        
        # Calculate inference time
        inference_time = time.time() - start_time
        
        # Print inference time every few frames
        if hasattr(self, 'frame_count'):
            self.frame_count += 1
            if self.frame_count % 10 == 0:
                print(f"Object detection time: {inference_time:.3f}s ({1/inference_time:.2f} FPS)")
        else:
            self.frame_count = 0
        
        return detections, annotated_frame
    
    def filter_detections(self, detections, class_ids=None):
        """
        Filter detections by class ID
        
        Args:
            detections: List of detection dictionaries
            class_ids: List of class IDs to keep, if None keeps all
        
        Returns:
            Filtered list of detections
        """
        if class_ids is None:
            return detections
        
        return [det for det in detections if det['class_id'] in class_ids]
    
    def get_frc_detections(self, detections):
        """
        Filter detections for FRC-specific objects
        This uses mapping from common COCO objects to FRC game elements
        
        Args:
            detections: List of detection dictionaries
        
        Returns:
            Dictionary with categorized detections
        """
        # Map COCO classes to FRC elements
        # For now using general objects until custom model is trained
        # person: human player or driver
        # sports ball: game piece 
        # car/truck: robot
        
        frc_detections = {
            'robots': [],    # Using car/truck (2, 7) for robots
            'game_pieces': [],  # Using sports ball (32) for game pieces
            'humans': [],    # Using person (0) for human players
            'others': []     # Everything else
        }
        
        for det in detections:
            cls_id = det['class_id']
            
            if cls_id in [2, 7]:  # car, truck
                frc_detections['robots'].append(det)
            elif cls_id == 32:  # sports ball
                frc_detections['game_pieces'].append(det)
            elif cls_id == 0:  # person
                frc_detections['humans'].append(det)
            else:
                frc_detections['others'].append(det)
        
        return frc_detections

# Test function
def test_object_detector():
    """Test the object detector using a webcam"""
    detector = ObjectDetector()
    cap = cv2.VideoCapture(1)  # Use webcam index 1 (or 0 for default)
    
    if not cap.isOpened():
        print("Error: Cannot open webcam")
        return
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break
            
            # Run detection
            detections, annotated_frame = detector.detect(frame)
            
            # Get FRC-specific detections
            frc_detections = detector.get_frc_detections(detections)
            
            # Display detection counts
            robot_count = len(frc_detections['robots'])
            piece_count = len(frc_detections['game_pieces'])
            human_count = len(frc_detections['humans'])
            
            # Add count text to frame
            cv2.putText(
                annotated_frame,
                f"Robots: {robot_count}, Game Pieces: {piece_count}, Humans: {human_count}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )
            
            # Show the frame
            cv2.imshow("Object Detection", annotated_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    test_object_detector()