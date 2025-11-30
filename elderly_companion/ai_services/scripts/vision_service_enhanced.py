#!/usr/bin/env python3
"""
Enhanced Vision Service with Person Tracking and Activity Detection
Handles object detection, person tracking, activity monitoring, and fall detection
"""
import os
import logging
import base64
from pathlib import Path
from typing import Optional, List, Dict, Any, Tuple
from collections import deque
from contextlib import asynccontextmanager
import time

# Configure logging first
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Import OpenCV and numpy
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError as e:
    CV2_AVAILABLE = False
    logger.error(f"OpenCV not available: {e}")
    raise

from fastapi import FastAPI, File, UploadFile, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel

# Import YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    logger.warning("YOLO not available - detection will be limited")

# ROS 2 imports (optional)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
    logger.info("ROS 2 support enabled")
except ImportError:
    ROS2_AVAILABLE = False
    logger.warning("ROS 2 not available - running without ROS 2 integration")
    class Node:
        def __init__(self, *args, **kwargs):
            pass
        def create_publisher(self, *args, **kwargs):
            return DummyPublisher()
        def destroy_node(self):
            pass
    class String:
        def __init__(self):
            self.data = ""
    class Image:
        pass
    class DummyPublisher:
        def publish(self, msg):
            pass

import json

# Global ROS 2 node
vision_node: Optional[VisionService] = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    global vision_node
    try:
        if ROS2_AVAILABLE:
            rclpy.init()
        vision_node = VisionService()
        logger.info("Enhanced Vision Service started successfully")
    except Exception as e:
        logger.error(f"Failed to start vision service: {e}")
    
    yield
    
    # Shutdown
    if vision_node:
        try:
            vision_node.destroy_node()
        except:
            pass
    if ROS2_AVAILABLE:
        try:
            rclpy.shutdown()
        except:
            pass
    logger.info("Enhanced Vision Service shut down")

app = FastAPI(title="Vision Service (Enhanced)", version="2.0.0", lifespan=lifespan)

class DetectionResult(BaseModel):
    objects: List[Dict[str, Any]]
    person_detected: bool
    person_count: int
    timestamp: float
    activities: Optional[List[Dict[str, Any]]] = None
    fall_detected: Optional[bool] = None

class PersonTracker:
    """Tracks persons across frames using IoU (Intersection over Union)"""
    def __init__(self, max_disappeared=5, max_distance=0.5):
        self.next_id = 0
        self.tracks = {}  # {id: {'bbox': bbox, 'centroid': centroid, 'disappeared': count, 'positions': deque}}
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance
        self.position_history_length = 10
    
    def update(self, detections: List[Dict]) -> List[Dict]:
        """
        Update tracker with new detections
        
        Args:
            detections: List of person detections with 'bbox' key
            
        Returns:
            List of detections with 'track_id' added
        """
        if not detections:
            # Mark all tracks as disappeared
            for track_id in list(self.tracks.keys()):
                self.tracks[track_id]['disappeared'] += 1
                if self.tracks[track_id]['disappeared'] > self.max_disappeared:
                    del self.tracks[track_id]
            return []
        
        # Calculate centroids for new detections
        input_centroids = []
        for det in detections:
            bbox = det['bbox']
            centroid = (
                (bbox['x1'] + bbox['x2']) / 2,
                (bbox['y1'] + bbox['y2']) / 2
            )
            input_centroids.append(centroid)
        
        # If no existing tracks, create new ones
        if not self.tracks:
            for i, det in enumerate(detections):
                self.tracks[self.next_id] = {
                    'bbox': det['bbox'],
                    'centroid': input_centroids[i],
                    'disappeared': 0,
                    'positions': deque(maxlen=self.position_history_length)
                }
                det['track_id'] = self.next_id
                self.tracks[self.next_id]['positions'].append(input_centroids[i])
                self.next_id += 1
        else:
            # Match existing tracks to new detections
            track_ids = list(self.tracks.keys())
            track_centroids = [self.tracks[tid]['centroid'] for tid in track_ids]
            
            # Calculate distances (simple Euclidean distance)
            distances = []
            for track_centroid in track_centroids:
                row = []
                for input_centroid in input_centroids:
                    dist = np.sqrt(
                        (track_centroid[0] - input_centroid[0])**2 +
                        (track_centroid[1] - input_centroid[1])**2
                    )
                    row.append(dist)
                distances.append(row)
            
            # Simple greedy matching (can be improved with Hungarian algorithm)
            used_detection_indices = set()
            for i, track_id in enumerate(track_ids):
                if not distances[i]:
                    continue
                min_dist = min(distances[i])
                if min_dist < self.max_distance * 1000:  # Scale threshold
                    min_idx = distances[i].index(min_dist)
                    if min_idx not in used_detection_indices:
                        # Update track
                        self.tracks[track_id]['bbox'] = detections[min_idx]['bbox']
                        self.tracks[track_id]['centroid'] = input_centroids[min_idx]
                        self.tracks[track_id]['disappeared'] = 0
                        self.tracks[track_id]['positions'].append(input_centroids[min_idx])
                        detections[min_idx]['track_id'] = track_id
                        used_detection_indices.add(min_idx)
                else:
                    # Track disappeared
                    self.tracks[track_id]['disappeared'] += 1
            
            # Create new tracks for unmatched detections
            for i, det in enumerate(detections):
                if i not in used_detection_indices:
                    self.tracks[self.next_id] = {
                        'bbox': det['bbox'],
                        'centroid': input_centroids[i],
                        'disappeared': 0,
                        'positions': deque(maxlen=self.position_history_length)
                    }
                    det['track_id'] = self.next_id
                    self.tracks[self.next_id]['positions'].append(input_centroids[i])
                    self.next_id += 1
            
            # Remove disappeared tracks
            for track_id in list(self.tracks.keys()):
                if self.tracks[track_id]['disappeared'] > self.max_disappeared:
                    del self.tracks[track_id]
        
        return detections

class ActivityDetector:
    """Detects activities based on person position and movement"""
    def __init__(self):
        self.activity_history = {}  # {track_id: deque of activities}
    
    def detect_activity(self, detection: Dict, image_height: int) -> str:
        """
        Detect activity based on bounding box position
        
        Args:
            detection: Detection dict with 'bbox' and 'track_id'
            image_height: Height of the image
            
        Returns:
            Activity string: 'standing', 'sitting', 'lying', 'unknown'
        """
        bbox = detection['bbox']
        bbox_height = bbox['y2'] - bbox['y1']
        bbox_center_y = (bbox['y1'] + bbox['y2']) / 2
        bbox_width = bbox['x2'] - bbox['x1']
        
        # Normalize to image height
        height_ratio = bbox_height / image_height
        center_ratio = bbox_center_y / image_height
        aspect_ratio = bbox_height / bbox_width if bbox_width > 0 else 1.0
        
        # Heuristic-based activity detection
        if height_ratio > 0.6 and aspect_ratio > 1.5:
            # Tall and narrow = standing
            activity = 'standing'
        elif height_ratio > 0.4 and aspect_ratio > 1.2:
            # Medium height = sitting
            activity = 'sitting'
        elif height_ratio < 0.3 or aspect_ratio < 1.0:
            # Short or wide = lying down
            activity = 'lying'
        else:
            activity = 'unknown'
        
        # Store in history
        track_id = detection.get('track_id', 'unknown')
        if track_id not in self.activity_history:
            self.activity_history[track_id] = deque(maxlen=5)
        self.activity_history[track_id].append(activity)
        
        return activity
    
    def get_consistent_activity(self, track_id: Any) -> str:
        """Get most consistent activity from history"""
        if track_id not in self.activity_history or not self.activity_history[track_id]:
            return 'unknown'
        
        activities = list(self.activity_history[track_id])
        # Return most common activity
        return max(set(activities), key=activities.count)

class FallDetector:
    """Detects falls based on sudden position changes and lack of movement"""
    def __init__(self, fall_threshold=0.3, stillness_duration=3.0):
        self.fall_threshold = fall_threshold  # Sudden drop in centroid Y position
        self.stillness_duration = stillness_duration  # Seconds of no movement
        self.position_history = {}  # {track_id: deque of (time, centroid_y)}
        self.fall_alerts = {}  # {track_id: last_fall_time}
    
    def check_fall(self, detection: Dict, current_time: float) -> bool:
        """
        Check if a fall has occurred
        
        Args:
            detection: Detection dict with 'bbox' and 'track_id'
            current_time: Current timestamp
            
        Returns:
            True if fall detected
        """
        track_id = detection.get('track_id', 'unknown')
        bbox = detection['bbox']
        centroid_y = (bbox['y1'] + bbox['y2']) / 2
        
        if track_id not in self.position_history:
            self.position_history[track_id] = deque(maxlen=10)
        
        self.position_history[track_id].append((current_time, centroid_y))
        
        if len(self.position_history[track_id]) < 3:
            return False
        
        # Check for sudden drop
        positions = list(self.position_history[track_id])
        recent_y = positions[-1][1]
        previous_y = positions[-3][1]
        
        # Normalize drop by image height (assuming bbox gives us relative position)
        drop_ratio = abs(recent_y - previous_y) / (bbox['y2'] - bbox['y1']) if (bbox['y2'] - bbox['y1']) > 0 else 0
        
        # Check for stillness (no significant movement in recent frames)
        if len(positions) >= 5:
            recent_positions = positions[-5:]
            y_values = [p[1] for p in recent_positions]
            y_variance = np.var(y_values)
            is_still = y_variance < 100  # Threshold for stillness
            
            # Check time span
            time_span = recent_positions[-1][0] - recent_positions[0][0]
            
            # Fall detected if: sudden drop AND stillness
            if drop_ratio > self.fall_threshold and is_still and time_span >= self.stillness_duration:
                # Prevent repeated alerts
                if track_id not in self.fall_alerts or (current_time - self.fall_alerts[track_id]) > 10.0:
                    self.fall_alerts[track_id] = current_time
                    return True
        
        return False

class VisionService(Node):
    def __init__(self):
        if ROS2_AVAILABLE:
            super().__init__('vision_service')
        else:
            Node.__init__(self)
        
        # Use configurable base path
        self.base_path = Path(os.getenv('APP_BASE_PATH', os.getcwd()))
        self.models_path = self.base_path / 'models'
        self.logs_path = self.base_path / 'logs'
        
        # Create directories
        self.models_path.mkdir(parents=True, exist_ok=True)
        self.logs_path.mkdir(parents=True, exist_ok=True)
        
        self.model_name = os.getenv('YOLO_MODEL', 'yolov8n.pt')
        self.model = None
        
        if YOLO_AVAILABLE:
            self.model = self.load_model()
        else:
            logger.warning("YOLO not available - detection will be simulated")
        
        # Enhanced tracking and detection
        self.person_tracker = PersonTracker()
        self.activity_detector = ActivityDetector()
        self.fall_detector = FallDetector()
        
        # ROS 2 publishers (optional)
        if ROS2_AVAILABLE:
            self.detection_pub = self.create_publisher(
                String,
                '/companion/vision/detections',
                10
            )
            self.alert_pub = self.create_publisher(
                String,
                '/companion/vision/alert',
                10
            )
        else:
            self.detection_pub = None
            self.alert_pub = None
        
    def load_model(self) -> Optional[Any]:
        """Load YOLO model with CPU-only configuration"""
        if not YOLO_AVAILABLE:
            return None
        try:
            import os
            os.environ['CUDA_VISIBLE_DEVICES'] = ''
            
            model_path = self.models_path / self.model_name
            if model_path.exists():
                logger.info(f"Loading model from {model_path}")
                model = YOLO(str(model_path))
            else:
                logger.info(f"Downloading model: {self.model_name}")
                model = YOLO(self.model_name)
                logger.info(f"Model downloaded and cached")
            
            # Explicitly set to CPU device
            try:
                import torch
                if hasattr(model, 'model'):
                    if hasattr(model.model, 'to'):
                        model.model.to('cpu')
                logger.info("Model configured for CPU-only operation")
            except Exception as device_error:
                logger.warning(f"Could not explicitly set device to CPU: {device_error}")
            
            logger.info("Model loaded successfully")
            return model
        except Exception as e:
            logger.error(f"Failed to load model: {e}", exc_info=True)
            return None
    
    def detect_objects(self, image: np.ndarray) -> Dict[str, Any]:
        """
        Detect objects with person tracking and activity detection
        
        Args:
            image: Input image as numpy array
            
        Returns:
            Dictionary with detection results including tracking and activities
        """
        try:
            current_time = time.time()
            height, width = image.shape[:2]
            
            if self.model:
                logger.info("Running YOLO detection...")
                try:
                    results = self.model(
                        image, 
                        verbose=False, 
                        imgsz=640,
                        device='cpu',
                        half=False
                    )
                    logger.info(f"YOLO returned {len(results)} result(s)")
                except Exception as e:
                    logger.error(f"YOLO error: {e}", exc_info=True)
                    raise
                
                detections = []
                person_detections = []
                
                for result_idx, result in enumerate(results):
                    try:
                        boxes = result.boxes
                        if boxes is None or len(boxes) == 0:
                            continue
                        
                        for box in boxes:
                            try:
                                if box.cls is None or len(box.cls) == 0:
                                    continue
                                if box.conf is None or len(box.conf) == 0:
                                    continue
                                if box.xyxy is None or len(box.xyxy) == 0:
                                    continue
                                
                                cls_tensor = box.cls[0]
                                conf_tensor = box.conf[0]
                                
                                if hasattr(cls_tensor, 'cpu'):
                                    cls = int(cls_tensor.cpu().item())
                                elif hasattr(cls_tensor, 'item'):
                                    cls = int(cls_tensor.item())
                                else:
                                    cls = int(cls_tensor)
                                
                                if hasattr(conf_tensor, 'cpu'):
                                    conf = float(conf_tensor.cpu().item())
                                elif hasattr(conf_tensor, 'item'):
                                    conf = float(conf_tensor.item())
                                else:
                                    conf = float(conf_tensor)
                                
                                if cls not in self.model.names:
                                    continue
                                
                                class_name = self.model.names[cls]
                                
                                bbox_tensor = box.xyxy[0]
                                if hasattr(bbox_tensor, 'cpu'):
                                    bbox = bbox_tensor.cpu().numpy()
                                elif hasattr(bbox_tensor, 'numpy'):
                                    bbox = bbox_tensor.numpy()
                                elif hasattr(bbox_tensor, 'tolist'):
                                    bbox = np.array(bbox_tensor.tolist())
                                else:
                                    bbox = np.array(bbox_tensor)
                                
                                x1, y1, x2, y2 = float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])
                                
                                detection = {
                                    'class': class_name,
                                    'confidence': round(conf, 3),
                                    'bbox': {
                                        'x1': x1,
                                        'y1': y1,
                                        'x2': x2,
                                        'y2': y2
                                    }
                                }
                                detections.append(detection)
                                
                                # Track persons separately
                                if class_name == 'person':
                                    person_detections.append(detection)
                                    
                            except Exception as box_error:
                                logger.warning(f"Error processing box: {box_error}")
                                continue
                                
                    except Exception as result_error:
                        logger.warning(f"Error processing result: {result_error}")
                        continue
                
                # Track persons across frames
                tracked_persons = self.person_tracker.update(person_detections)
                
                # Detect activities and falls
                activities = []
                fall_detected = False
                
                for person_det in tracked_persons:
                    # Detect activity
                    activity = self.activity_detector.detect_activity(person_det, height)
                    track_id = person_det.get('track_id', 'unknown')
                    consistent_activity = self.activity_detector.get_consistent_activity(track_id)
                    
                    activities.append({
                        'track_id': track_id,
                        'activity': consistent_activity,
                        'bbox': person_det['bbox']
                    })
                    
                    # Check for falls
                    if self.fall_detector.check_fall(person_det, current_time):
                        fall_detected = True
                        logger.warning(f"FALL DETECTED for person {track_id}!")
                        
                        # Publish alert if ROS 2 available
                        if ROS2_AVAILABLE and self.alert_pub:
                            alert_msg = String()
                            alert_msg.data = json.dumps({
                                'type': 'fall',
                                'track_id': track_id,
                                'timestamp': current_time,
                                'location': person_det['bbox']
                            })
                            self.alert_pub.publish(alert_msg)
                
                # Update person detections with track IDs
                for det in detections:
                    if det['class'] == 'person':
                        # Find matching tracked person
                        for tracked in tracked_persons:
                            if (abs(det['bbox']['x1'] - tracked['bbox']['x1']) < 10 and
                                abs(det['bbox']['y1'] - tracked['bbox']['y1']) < 10):
                                det['track_id'] = tracked.get('track_id')
                                break
                
            else:
                detections = []
                tracked_persons = []
                activities = []
            
            # Publish to ROS 2 (if available)
            if ROS2_AVAILABLE and self.detection_pub:
                msg = String()
                msg.data = json.dumps({
                    'person_count': len(tracked_persons),
                    'total_objects': len(detections),
                    'activities': activities,
                    'fall_detected': fall_detected
                })
                self.detection_pub.publish(msg)
            
            return {
                'objects': detections,
                'person_detected': len(tracked_persons) > 0,
                'person_count': len(tracked_persons),
                'total_objects': len(detections),
                'activities': activities,
                'fall_detected': fall_detected
            }
            
        except Exception as e:
            logger.error(f"Detection error: {e}", exc_info=True)
            raise HTTPException(status_code=500, detail=str(e))


@app.post("/detect", response_model=DetectionResult)
async def detect_endpoint(file: UploadFile = File(...)):
    """Detect objects with tracking and activity detection"""
    if not vision_node:
        raise HTTPException(status_code=503, detail="Service not ready")
    
    try:
        logger.info(f"Received file: {file.filename}, content_type: {file.content_type}")
        contents = await file.read()
        
        if not contents or len(contents) == 0:
            raise HTTPException(status_code=400, detail="Empty file uploaded")
        
        logger.info(f"File size: {len(contents)} bytes")
        
        nparr = np.frombuffer(contents, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if image is None:
            logger.error("Failed to decode image")
            raise HTTPException(status_code=400, detail="Invalid image format")
        
        logger.info(f"Image decoded successfully. Shape: {image.shape}")
        
        # Resize large images
        max_size = 640
        height, width = image.shape[:2]
        if height > max_size or width > max_size:
            scale = max_size / max(height, width)
            new_width = int(width * scale)
            new_height = int(height * scale)
            logger.info(f"Resizing image from {width}x{height} to {new_width}x{new_height}")
            image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        
        import time
        timestamp = time.time()
        
        try:
            result = vision_node.detect_objects(image)
            result['timestamp'] = timestamp
            logger.info(f"Detection completed. Found {result.get('total_objects', 0)} objects, {result.get('person_count', 0)} persons")
        except Exception as det_error:
            logger.error(f"Detection error: {det_error}", exc_info=True)
            raise HTTPException(status_code=500, detail=f"Detection failed: {str(det_error)}")
        
        return DetectionResult(**result)
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Detection endpoint error: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    status = {
        "status": "healthy" if vision_node else "unhealthy",
        "service": "vision",
        "model_loaded": vision_node.model is not None if vision_node and hasattr(vision_node, 'model') else False,
        "ros2_available": ROS2_AVAILABLE,
        "features": {
            "person_tracking": True,
            "activity_detection": True,
            "fall_detection": True
        }
    }
    status_code = 200 if status["status"] == "healthy" else 503
    return JSONResponse(content=status, status_code=status_code)

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv('VISION_SERVICE_PORT', '8003'))
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=port,
        log_level="info"
    )

