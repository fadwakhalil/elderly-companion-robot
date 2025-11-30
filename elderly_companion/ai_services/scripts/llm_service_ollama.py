#!/usr/bin/env python3
"""
LLM Service with Ollama Integration
Enhanced version that uses local Ollama for natural conversations
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

# Try to import Ollama
try:
    import httpx
    OLLAMA_AVAILABLE = True
except ImportError:
    OLLAMA_AVAILABLE = False
    logger.warning("httpx not available - Ollama integration disabled")

# ROS 2 imports (optional)
try:
    import rclpy
    from rclpy.node import Node
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
        def create_subscription(self, *args, **kwargs):
            return None
        def destroy_node(self):
            pass
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
        logger.info("LLM Service (Ollama) started successfully")
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
    logger.info("LLM Service (Ollama) shut down")

app = FastAPI(title="LLM Service (Ollama)", version="2.0.0", lifespan=lifespan)

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
        
        # Configuration
        self.ollama_host = os.getenv('OLLAMA_HOST', 'http://localhost:11434')
        self.ollama_model = os.getenv('OLLAMA_MODEL', 'llama3.2:3b')
        self.use_ollama = OLLAMA_AVAILABLE and os.getenv('USE_OLLAMA', 'true').lower() == 'true'
        
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
        
        if self.use_ollama:
            logger.info(f"Using Ollama with model: {self.ollama_model}")
            logger.info(f"Ollama host: {self.ollama_host}")
        else:
            logger.info("Using rule-based responses (Ollama not available)")
        
    def load_personality(self) -> dict:
        """Load personality configuration"""
        # Try multiple paths (Docker and local)
        config_paths = [
            Path('/app/config/personality.yaml'),
            Path(os.getcwd()) / 'config' / 'personality.yaml',
            Path(__file__).parent.parent / 'config' / 'personality.yaml'
        ]
        
        for config_path in config_paths:
            try:
                if config_path.exists():
                    with open(config_path, 'r') as f:
                        personality = yaml.safe_load(f)
                    logger.info(f"Personality configuration loaded from {config_path}")
                    return personality
            except Exception as e:
                logger.warning(f"Failed to load personality config from {config_path}: {e}")
        
        # Default personality
        logger.warning("Using default personality configuration")
        return {
            'name': 'Companion',
            'traits': ['empathetic', 'patient', 'supportive'],
            'conversation_style': 'warm and caring'
        }
    
    def message_callback(self, msg: String):
        """ROS 2 callback for incoming messages"""
        logger.info(f"Received ROS 2 message: {msg.data}")
        # Process and respond via ROS 2 if needed
    
    def generate_response_ollama(self, message: str, context: Optional[dict] = None,
                                 user_id: Optional[str] = None) -> str:
        """Generate response using Ollama"""
        if not OLLAMA_AVAILABLE:
            raise RuntimeError("Ollama not available")
        
        # Build conversation history
        history = self.conversation_history.get(user_id or 'default', [])
        
        # Build system prompt with personality
        system_prompt = f"""You are {self.personality.get('name', 'Companion')}, a caring companion robot for elderly individuals.

Your personality traits: {', '.join(self.personality.get('traits', []))}
Your conversation style: {self.personality.get('conversation_style', 'warm and caring')}

Guidelines:
- Be empathetic, patient, and supportive
- Use clear, simple language
- Be proactive but not intrusive
- Show genuine care and interest
- Respect the user's dignity and independence
- Keep responses concise (2-3 sentences)
"""
        
        # Build messages for Ollama
        messages = [{"role": "system", "content": system_prompt}]
        messages.extend(history[-10:])  # Last 10 exchanges
        messages.append({"role": "user", "content": message})
        
        try:
            # Call Ollama API
            with httpx.Client(timeout=30.0) as client:
                response = client.post(
                    f"{self.ollama_host}/api/chat",
                    json={
                        "model": self.ollama_model,
                        "messages": messages,
                        "stream": False
                    }
                )
                response.raise_for_status()
                result = response.json()
                assistant_message = result.get('message', {}).get('content', '')
                
                # Update conversation history
                if user_id:
                    if user_id not in self.conversation_history:
                        self.conversation_history[user_id] = []
                    self.conversation_history[user_id].append({'role': 'user', 'content': message})
                    self.conversation_history[user_id].append({'role': 'assistant', 'content': assistant_message})
                    # Keep only last 20 exchanges
                    if len(self.conversation_history[user_id]) > 20:
                        self.conversation_history[user_id] = self.conversation_history[user_id][-20:]
                
                return assistant_message
                
        except httpx.RequestError as e:
            logger.error(f"Ollama request error: {e}")
            raise HTTPException(status_code=503, detail=f"Ollama service unavailable: {e}")
        except Exception as e:
            logger.error(f"Ollama error: {e}", exc_info=True)
            raise HTTPException(status_code=500, detail=f"Error generating response: {e}")
    
    def generate_response_rule_based(self, message: str, context: Optional[dict] = None,
                                     user_id: Optional[str] = None) -> str:
        """Generate response using rule-based system (fallback)"""
        message_lower = message.lower()
        
        # Maintain conversation history per user
        if user_id:
            if user_id not in self.conversation_history:
                self.conversation_history[user_id] = []
            self.conversation_history[user_id].append({'role': 'user', 'content': message})
        
        # Rule-based responses
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
            if len(self.conversation_history[user_id]) > 20:
                self.conversation_history[user_id] = self.conversation_history[user_id][-20:]
        
        return response
    
    def generate_response(self, message: str, context: Optional[dict] = None,
                         user_id: Optional[str] = None) -> str:
        """Generate response using Ollama or rule-based fallback"""
        if self.use_ollama:
            try:
                return self.generate_response_ollama(message, context, user_id)
            except Exception as e:
                logger.warning(f"Ollama failed, falling back to rule-based: {e}")
                return self.generate_response_rule_based(message, context, user_id)
        else:
            return self.generate_response_rule_based(message, context, user_id)


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
    ollama_status = "unknown"
    if llm_node and llm_node.use_ollama:
        try:
            import httpx
            with httpx.Client(timeout=5.0) as client:
                response = client.get(f"{llm_node.ollama_host}/api/tags")
                ollama_status = "available" if response.status_code == 200 else "unavailable"
        except:
            ollama_status = "unavailable"
    
    status = {
        "status": "healthy" if llm_node else "unhealthy",
        "service": "llm",
        "personality_loaded": llm_node.personality is not None if llm_node else False,
        "ros2_available": ROS2_AVAILABLE,
        "ollama_enabled": llm_node.use_ollama if llm_node else False,
        "ollama_status": ollama_status,
        "ollama_model": llm_node.ollama_model if llm_node else None
    }
    status_code = 200 if status["status"] == "healthy" else 503
    return JSONResponse(content=status, status_code=status_code)

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv('LLM_SERVICE_PORT', '8002'))
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=port,
        log_level="info"
    )

