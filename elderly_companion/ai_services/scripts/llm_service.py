#!/usr/bin/env python3
"""
LLM Service for Elderly Companion Robot
Handles conversation generation and personality management
"""
import os
import json
import logging
from pathlib import Path
from typing import Optional, Dict, Any
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

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
        def create_subscription(self, *args, **kwargs):
            return None
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

import yaml

# Global ROS 2 node
llm_node: Optional[LLMService] = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    global llm_node
    try:
        if ROS2_AVAILABLE:
            rclpy.init()
        llm_node = LLMService()
        logger.info("LLM Service started successfully")
    except Exception as e:
        logger.error(f"Failed to start LLM service: {e}")
        # Don't raise - allow service to start even if ROS 2 fails
    
    yield
    
    # Shutdown
    if llm_node:
        try:
            llm_node.destroy_node()
        except:
            pass
    if ROS2_AVAILABLE:
        try:
            rclpy.shutdown()
        except:
            pass
    logger.info("LLM Service shut down")

app = FastAPI(title="LLM Service", version="1.0.0", lifespan=lifespan)

class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=1000)
    context: Optional[Dict[str, Any]] = None
    user_id: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    context: Optional[Dict[str, Any]] = None
    confidence: float = Field(default=0.8, ge=0.0, le=1.0)

class LLMService(Node):
    def __init__(self):
        if ROS2_AVAILABLE:
            super().__init__('llm_service')
        else:
            Node.__init__(self)
        
        self.personality = self.load_personality()
        self.conversation_history: Dict[str, list] = {}
        
        # ROS 2 publishers/subscribers (optional)
        if ROS2_AVAILABLE:
            self.response_pub = self.create_publisher(
                String,
                '/companion/llm/response',
                10
            )
            self.message_sub = self.create_subscription(
                String,
                '/companion/llm/request',
                self.message_callback,
                10
            )
        else:
            self.response_pub = None
            self.message_sub = None
        
    def load_personality(self) -> dict:
        """Load personality configuration"""
        config_path = Path('/app/config/personality.yaml')
        try:
            if config_path.exists():
                with open(config_path, 'r') as f:
                    personality = yaml.safe_load(f)
                logger.info("Personality configuration loaded")
                return personality
        except Exception as e:
            logger.warning(f"Failed to load personality config: {e}")
        
        # Default personality
        return {
            'name': 'Companion',
            'traits': ['empathetic', 'patient', 'supportive'],
            'conversation_style': 'warm and caring'
        }
    
    def message_callback(self, msg: String):
        """ROS 2 callback for incoming messages"""
        logger.info(f"Received ROS 2 message: {msg.data}")
        # Process and respond via ROS 2 if needed
    
    def generate_response(self, message: str, context: Optional[dict] = None, 
                         user_id: Optional[str] = None) -> str:
        """
        Generate response using rule-based system (can be replaced with Ollama)
        
        Args:
            message: User's message
            context: Additional context
            user_id: User identifier for conversation history
            
        Returns:
            Generated response
        """
        message_lower = message.lower()
        
        # Maintain conversation history per user
        if user_id:
            if user_id not in self.conversation_history:
                self.conversation_history[user_id] = []
            self.conversation_history[user_id].append({'role': 'user', 'content': message})
        
        # Rule-based responses (replace with Ollama API call)
        if any(greeting in message_lower for greeting in ['hello', 'hi', 'hey', 'good morning']):
            response = "Hello there! I'm your companion. How are you feeling today?"
        
        elif 'how are you' in message_lower:
            response = "I'm here and ready to help you. How can I assist you today?"
        
        elif any(word in message_lower for word in ['lonely', 'alone', 'bored']):
            response = "I understand it can feel lonely sometimes. Would you like to hear a story, or maybe we could talk about your favorite memories?"
        
        elif any(word in message_lower for word in ['medicine', 'medication', 'pill']):
            response = "I can help remind you about medications. Would you like me to set a reminder for your next dose?"
        
        elif any(word in message_lower for word in ['help', 'assist', 'need']):
            response = "Of course! I'm here to help. What do you need assistance with?"
        
        else:
            response = "I'm listening. Tell me more about how you're feeling. I'm here to keep you company."
        
        # Store response in history
        if user_id:
            self.conversation_history[user_id].append({'role': 'assistant', 'content': response})
            # Keep only last 10 exchanges
            if len(self.conversation_history[user_id]) > 20:
                self.conversation_history[user_id] = self.conversation_history[user_id][-20:]
        
        # Publish to ROS 2 (if available)
        if ROS2_AVAILABLE and self.response_pub:
            msg = String()
            msg.data = response
            self.response_pub.publish(msg)
        
        return response


@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """Main chat endpoint"""
    if not llm_node:
        raise HTTPException(status_code=503, detail="Service not ready")
    
    try:
        response_text = llm_node.generate_response(
            request.message,
            request.context,
            request.user_id
        )
        
        return ChatResponse(
            response=response_text,
            context=request.context,
            confidence=0.85
        )
    except Exception as e:
        logger.error(f"Chat error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    status = {
        "status": "healthy" if llm_node else "unhealthy",
        "service": "llm",
        "personality_loaded": llm_node.personality is not None if llm_node else False,
        "ros2_available": ROS2_AVAILABLE
    }
    status_code = 200 if status["status"] == "healthy" else 503
    return JSONResponse(content=status, status_code=status_code)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8002,
        log_level="info"
    )
