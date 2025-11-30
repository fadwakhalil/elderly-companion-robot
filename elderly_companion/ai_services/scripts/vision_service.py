#!/usr/bin/env python3
"""
Vision Service for Elderly Companion Robot
Handles object detection, person detection, and scene understanding
"""
import os
import logging
import base64
from pathlib import Path
from typing import Optional, List, Dict, Any
from contextlib import asynccontextmanager

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

# ROS 2 imports (optional - for Docker containers without ROS 2)
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
    # Create dummy classes for when ROS 2 is not available
    class Node:
        def __init__(self, *args, **kwargs):
            pass
        def create_publisher(self, *args, **kwargs):
            return DummyPublisher()
        def destroy_node(self):
            pass
        def get_logger(self):
            return logger
    class String:
        def __init__(self):
            self.data = ""
    class Image:
        pass
    class DummyPublisher:
        def publish(self, msg):
            pass

import json

class DetectionResult(BaseModel):
    objects: List[Dict[str, Any]]
    person_detected: bool
    person_count: int
    timestamp: float

class VisionService(Node):
    def __init__(self):
        if ROS2_AVAILABLE:
            super().__init__('vision_service')
        else:
            Node.__init__(self)
        
        # Use configurable base path (works in Docker and locally)
        self.base_path = Path(os.getenv('APP_BASE_PATH', os.getcwd()))
        self.models_path = self.base_path / 'models'
        self.config_path = self.base_path / 'config'
        self.logs_path = self.base_path / 'logs'
        
        # Create directories if they don't exist
        self.models_path.mkdir(parents=True, exist_ok=True)
        self.logs_path.mkdir(parents=True, exist_ok=True)
        
        self.model_name = os.getenv('YOLO_MODEL', 'yolov8n.pt')
        self.model = None
        
        if YOLO_AVAILABLE:
            self.model = self.load_model()
        else:
            logger.warning("YOLO not available - detection will be simulated")
        
        # ROS 2 publishers (optional)
        if ROS2_AVAILABLE:
            self.detection_pub = self.create_publisher(
                String,
                '/companion/vision/detections',
                10
            )
            self.image_pub = self.create_publisher(
                Image,
                '/companion/vision/image',
                10
            )
        else:
            self.detection_pub = None
            self.image_pub = None
        
    def load_model(self) -> Optional[Any]:
        """Load YOLO model with CPU-only configuration"""
        if not YOLO_AVAILABLE:
            return None
        try:
            # Force CPU-only operation
            import os
            os.environ['CUDA_VISIBLE_DEVICES'] = ''
            
            model_path = self.models_path / self.model_name
            if model_path.exists():
                logger.info(f"Loading model from {model_path}")
                model = YOLO(str(model_path))
            else:
                logger.info(f"Downloading model: {self.model_name}")
                # YOLO will download and cache the model automatically
                # The model will be cached in ~/.ultralytics/ by default
                model = YOLO(self.model_name)
                logger.info(f"Model downloaded and cached. Will use cached version next time.")
            
            # Explicitly set to CPU device
            try:
                import torch
                if hasattr(model, 'model'):
                    if hasattr(model.model, 'to'):
                        model.model.to('cpu')
                    if hasattr(model.model, 'device'):
                        logger.info(f"Model device: {model.model.device if hasattr(model.model, 'device') else 'unknown'}")
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
        Detect objects in image
        
        Args:
            image: Input image as numpy array
            
        Returns:
            Dictionary with detection results
        """
        try:
            if self.model:
                logger.info("Running YOLO detection...")
                try:
                    # Force CPU device and use safer inference parameters
                    # imgsz=640 ensures consistent processing size
                    # device='cpu' explicitly forces CPU usage
                    # half=False disables FP16 which can cause issues
                    results = self.model(
                        image, 
                        verbose=False, 
                        imgsz=640,
                        device='cpu',
                        half=False
                    )
                    logger.info(f"YOLO returned {len(results)} result(s)")
                except RuntimeError as e:
                    logger.error(f"YOLO runtime error: {e}", exc_info=True)
                    # Try with minimal parameters as fallback
                    try:
                        logger.info("Retrying with minimal parameters...")
                        results = self.model(image, verbose=False)
                        logger.info(f"YOLO returned {len(results)} result(s) (fallback)")
                    except Exception as fallback_error:
                        logger.error(f"YOLO fallback also failed: {fallback_error}", exc_info=True)
                        raise
                except Exception as e:
                    logger.error(f"YOLO unexpected error: {e}", exc_info=True)
                    raise
                
                detections = []
                person_count = 0
                
                for result_idx, result in enumerate(results):
                    try:
                        boxes = result.boxes
                        if boxes is None or len(boxes) == 0:
                            logger.debug(f"No detections in result {result_idx}")
                            continue
                        
                        logger.debug(f"Processing {len(boxes)} boxes in result {result_idx}")
                        
                        for box_idx, box in enumerate(boxes):
                            try:
                                # Safely access box properties
                                if box.cls is None or len(box.cls) == 0:
                                    continue
                                if box.conf is None or len(box.conf) == 0:
                                    continue
                                if box.xyxy is None or len(box.xyxy) == 0:
                                    continue
                                
                                # Safely extract class and confidence
                                cls_tensor = box.cls[0]
                                conf_tensor = box.conf[0]
                                
                                # Convert to CPU and then to Python types
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
                                    logger.warning(f"Unknown class ID: {cls}")
                                    continue
                                
                                class_name = self.model.names[cls]
                                
                                # Get bounding box coordinates - ensure CPU tensor
                                bbox_tensor = box.xyxy[0]
                                # Move to CPU if it's a tensor
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
                                
                                if class_name == 'person':
                                    person_count += 1
                                    
                            except Exception as box_error:
                                logger.warning(f"Error processing box {box_idx}: {box_error}")
                                continue
                                
                    except Exception as result_error:
                        logger.warning(f"Error processing result {result_idx}: {result_error}")
                        continue
            else:
                # Simulate detection if model not available
                detections = []
                person_count = 0
                logger.warning("Model not loaded - returning empty detections")
            
            # Publish to ROS 2 (if available)
            if ROS2_AVAILABLE and self.detection_pub:
                msg = String()
                msg.data = json.dumps({
                    'person_count': person_count,
                    'total_objects': len(detections)
                })
                self.detection_pub.publish(msg)
            
            return {
                'objects': detections,
                'person_detected': person_count > 0,
                'person_count': person_count,
                'total_objects': len(detections)
            }
            
        except Exception as e:
            logger.error(f"Detection error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

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
        logger.info("Vision Service started successfully")
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
    logger.info("Vision Service shut down")

app = FastAPI(title="Vision Service", version="1.0.0", lifespan=lifespan)

@app.post("/detect", response_model=DetectionResult)
async def detect_endpoint(file: UploadFile = File(...)):
    """Detect objects in uploaded image"""
    if not vision_node:
        raise HTTPException(status_code=503, detail="Service not ready")
    
    try:
        # Read image
        logger.info(f"Received file: {file.filename}, content_type: {file.content_type}")
        contents = await file.read()
        
        if not contents or len(contents) == 0:
            raise HTTPException(status_code=400, detail="Empty file uploaded")
        
        logger.info(f"File size: {len(contents)} bytes")
        
        # Convert to numpy array
        nparr = np.frombuffer(contents, np.uint8)
        
        # Decode image
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if image is None:
            logger.error("Failed to decode image")
            raise HTTPException(status_code=400, detail="Invalid image format. Supported formats: JPEG, PNG, BMP, etc.")
        
        logger.info(f"Image decoded successfully. Shape: {image.shape}")
        
        # Resize large images to prevent memory issues (max 640x640 for YOLO)
        max_size = 640
        height, width = image.shape[:2]
        if height > max_size or width > max_size:
            scale = max_size / max(height, width)
            new_width = int(width * scale)
            new_height = int(height * scale)
            logger.info(f"Resizing image from {width}x{height} to {new_width}x{new_height}")
            image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        
        # Perform detection
        import time
        timestamp = time.time()
        
        try:
            result = vision_node.detect_objects(image)
            result['timestamp'] = timestamp
            logger.info(f"Detection completed. Found {result.get('total_objects', 0)} objects")
        except Exception as det_error:
            logger.error(f"Detection error: {det_error}", exc_info=True)
            raise HTTPException(status_code=500, detail=f"Detection failed: {str(det_error)}")
        
        return DetectionResult(**result)
        
    except HTTPException:
        # Re-raise HTTP exceptions
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
        "ros2_available": ROS2_AVAILABLE
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
