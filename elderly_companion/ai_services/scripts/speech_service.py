#!/usr/bin/env python3
"""
Speech-to-Text Service for Elderly Companion Robot
Handles audio transcription using Whisper
"""
import os
import json
import logging
import asyncio
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import JSONResponse

# Configure logging first
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ROS 2 imports (optional - for Docker containers without ROS 2)
try:
    import rclpy
    from rclpy.node import Node
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
    class DummyPublisher:
        def publish(self, msg):
            pass

# Import whisper after logging is set up
try:
    import whisper
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    logger.warning("Whisper not available - some features will be limited")

app = FastAPI(title="Speech Service", version="1.0.0")

class SpeechService(Node):
    def __init__(self):
        if ROS2_AVAILABLE:
            super().__init__('speech_service')
        else:
            Node.__init__(self)
        
        self.model_size = os.getenv('MODEL_SIZE', 'tiny.en')
        self.model = None
        
        if WHISPER_AVAILABLE:
            self.load_model()
        else:
            logger.warning("Whisper not available - transcription will be simulated")
        
        # ROS 2 publisher for transcriptions (optional)
        if ROS2_AVAILABLE:
            self.transcription_pub = self.create_publisher(
                String, 
                '/companion/speech/transcription', 
                10
            )
        else:
            self.transcription_pub = None
        
    def load_model(self):
        """Load Whisper model"""
        if not WHISPER_AVAILABLE:
            return
        try:
            logger.info(f"Loading Whisper model: {self.model_size}")
            self.model = whisper.load_model(self.model_size)
            logger.info("Model loaded successfully")
        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            # Don't raise - allow service to start without model for testing
    
    def transcribe_audio(self, audio_path: str) -> dict:
        """
        Transcribe audio file
        
        Args:
            audio_path: Path to audio file
            
        Returns:
            Dictionary with transcription and metadata
        """
        try:
            if not os.path.exists(audio_path):
                raise FileNotFoundError(f"Audio file not found: {audio_path}")
            
            logger.info(f"Transcribing audio: {audio_path}")
            
            if self.model:
                result = self.model.transcribe(audio_path)
                transcription = result.get('text', '').strip()
                language = result.get('language', 'unknown')
                segments = result.get('segments', [])
            else:
                # Simulate transcription if model not available
                transcription = "Simulated transcription - model not loaded"
                language = "en"
                segments = []
            
            # Publish to ROS 2 (if available)
            if ROS2_AVAILABLE and self.transcription_pub:
                msg = String()
                msg.data = transcription
                self.transcription_pub.publish(msg)
            
            return {
                'text': transcription,
                'language': language,
                'confidence': 0.95,
                'segments': len(segments)
            }
            
        except Exception as e:
            logger.error(f"Transcription error: {e}")
            raise HTTPException(status_code=500, detail=str(e))

# Global ROS 2 node
speech_node: Optional[SpeechService] = None

@app.on_event("startup")
async def startup_event():
    global speech_node
    try:
        if ROS2_AVAILABLE:
            rclpy.init()
        speech_node = SpeechService()
        logger.info("Speech Service started successfully")
    except Exception as e:
        logger.error(f"Failed to start speech service: {e}")
        # Don't raise - allow service to start even if ROS 2 fails

@app.on_event("shutdown")
async def shutdown_event():
    global speech_node
    if speech_node:
        try:
            speech_node.destroy_node()
        except:
            pass
    if ROS2_AVAILABLE:
        try:
            rclpy.shutdown()
        except:
            pass
    logger.info("Speech Service shut down")

@app.websocket("/ws/speech")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time audio streaming"""
    await websocket.accept()
    logger.info("WebSocket connection established")
    
    try:
        while True:
            data = await websocket.receive_text()
            message = json.loads(data)
            
            if message.get('type') == 'audio_chunk':
                # In production, save audio chunk and process
                # For now, return acknowledgment
                response = {
                    'type': 'ack',
                    'status': 'received'
                }
                await websocket.send_text(json.dumps(response))
                
    except WebSocketDisconnect:
        logger.info("WebSocket disconnected")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        await websocket.close()

@app.post("/transcribe")
async def transcribe_endpoint(audio_path: str):
    """HTTP endpoint for file-based transcription"""
    if not speech_node:
        raise HTTPException(status_code=503, detail="Service not ready")
    
    try:
        result = speech_node.transcribe_audio(audio_path)
        return JSONResponse(content=result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    status = {
        "status": "healthy" if speech_node else "unhealthy",
        "service": "speech",
        "model_loaded": speech_node.model is not None if speech_node and hasattr(speech_node, 'model') else False,
        "ros2_available": ROS2_AVAILABLE
    }
    status_code = 200 if status["status"] == "healthy" else 503
    return JSONResponse(content=status, status_code=status_code)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app, 
        host="0.0.0.0", 
        port=8001,
        log_level="info"
    )
