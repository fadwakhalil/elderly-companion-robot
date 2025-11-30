#!/usr/bin/env python3
"""
ROS 2 AI Bridge Node
Connects AI services (Speech, LLM, Vision) to ROS 2 robot control
Manages state transitions and emergency protocols
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
import httpx
import json
import asyncio
from enum import Enum
from typing import Optional, Dict, Any
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RobotState(Enum):
    """Robot behavior states"""
    IDLE = "idle"
    PATROLLING = "patrolling"
    INTERACTING = "interacting"
    ASSISTING = "assisting"
    ALERTING = "alerting"
    CHARGING = "charging"

class AIBridge(Node):
    """
    Bridge between AI services and ROS 2 robot control
    """
    def __init__(self):
        super().__init__('ai_bridge')
        
        # AI Service URLs
        self.speech_service_url = self.declare_parameter('speech_service_url', 'http://localhost:8001').value
        self.llm_service_url = self.declare_parameter('llm_service_url', 'http://localhost:8002').value
        self.vision_service_url = self.declare_parameter('vision_service_url', 'http://localhost:8003').value
        
        # Current state
        self.current_state = RobotState.IDLE
        self.last_activity_time = None
        self.person_detected = False
        self.last_person_location = None
        
        # ROS 2 Subscribers (from AI services)
        self.speech_sub = self.create_subscription(
            String,
            '/companion/speech/transcription',
            self.speech_callback,
            10
        )
        
        self.vision_sub = self.create_subscription(
            String,
            '/companion/vision/detections',
            self.vision_callback,
            10
        )
        
        # ROS 2 Publishers (to robot control)
        self.llm_request_pub = self.create_publisher(
            String,
            '/companion/llm/request',
            10
        )
        
        self.llm_response_sub = self.create_subscription(
            String,
            '/companion/llm/response',
            self.llm_response_callback,
            10
        )
        
        # Robot control publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        self.state_pub = self.create_publisher(
            String,
            '/companion/state',
            10
        )
        
        # Emergency alert publisher
        self.alert_pub = self.create_publisher(
            String,
            '/companion/alert',
            10
        )
        
        # State management timer
        self.state_timer = self.create_timer(1.0, self.state_manager_callback)
        
        # Activity monitoring timer
        self.activity_timer = self.create_timer(30.0, self.activity_check_callback)
        
        self.get_logger().info("AI Bridge node started")
        self.get_logger().info(f"Current state: {self.current_state.value}")
    
    def speech_callback(self, msg: String):
        """Process speech transcription and send to LLM"""
        transcription = msg.data
        self.get_logger().info(f"Received transcription: {transcription}")
        
        # Update activity
        self.last_activity_time = self.get_clock().now()
        
        # Transition to interacting state
        if self.current_state == RobotState.IDLE or self.current_state == RobotState.PATROLLING:
            self.transition_state(RobotState.INTERACTING)
        
        # Forward to LLM service
        self.send_to_llm(transcription)
    
    def vision_callback(self, msg: String):
        """Process vision detections"""
        try:
            detection_data = json.loads(msg.data)
            person_count = detection_data.get('person_count', 0)
            total_objects = detection_data.get('total_objects', 0)
            
            self.person_detected = person_count > 0
            
            if self.person_detected:
                self.last_activity_time = self.get_clock().now()
                self.get_logger().info(f"Person detected: {person_count} person(s), {total_objects} total objects")
            else:
                self.get_logger().debug("No person detected")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse vision data: {e}")
    
    def llm_response_callback(self, msg: String):
        """Handle LLM response"""
        response = msg.data
        self.get_logger().info(f"LLM response: {response}")
        
        # Can publish to TTS or action topics here
        # For now, just log the response
    
    def send_to_llm(self, message: str):
        """Send message to LLM service via HTTP"""
        try:
            # In production, use async HTTP client
            # For now, publish to ROS 2 topic (LLM service can subscribe)
            msg = String()
            msg.data = message
            self.llm_request_pub.publish(msg)
            
            # Also try direct HTTP call (optional)
            # response = httpx.post(
            #     f"{self.llm_service_url}/chat",
            #     json={"message": message, "user_id": "ros2_bridge"},
            #     timeout=10.0
            # )
            
        except Exception as e:
            self.get_logger().error(f"Error sending to LLM: {e}")
    
    def transition_state(self, new_state: RobotState):
        """Transition to a new state"""
        if self.current_state != new_state:
            old_state = self.current_state
            self.current_state = new_state
            self.get_logger().info(f"State transition: {old_state.value} -> {new_state.value}")
            
            # Publish state change
            state_msg = String()
            state_msg.data = json.dumps({
                'state': new_state.value,
                'previous_state': old_state.value,
                'timestamp': str(self.get_clock().now())
            })
            self.state_pub.publish(state_msg)
            
            # Handle state-specific actions
            self.handle_state_entry(new_state)
    
    def handle_state_entry(self, state: RobotState):
        """Handle actions when entering a new state"""
        if state == RobotState.PATROLLING:
            # Start patrolling behavior
            self.get_logger().info("Starting patrolling behavior")
            # TODO: Publish navigation goals
            
        elif state == RobotState.INTERACTING:
            # Stop moving, face user
            self.get_logger().info("Entering interaction mode")
            # TODO: Stop robot, orient towards user
            
        elif state == RobotState.ALERTING:
            # Emergency protocol
            self.get_logger().warn("ALERT: Emergency situation detected!")
            alert_msg = String()
            alert_msg.data = json.dumps({
                'type': 'emergency',
                'timestamp': str(self.get_clock().now()),
                'state': 'alerting'
            })
            self.alert_pub.publish(alert_msg)
    
    def state_manager_callback(self):
        """Periodic state management"""
        # State-specific behaviors
        if self.current_state == RobotState.INTERACTING:
            # If no activity for 60 seconds, return to idle
            if self.last_activity_time:
                elapsed = (self.get_clock().now() - self.last_activity_time).nanoseconds / 1e9
                if elapsed > 60.0:
                    self.transition_state(RobotState.IDLE)
    
    def activity_check_callback(self):
        """Check for user activity and trigger proactive behaviors"""
        # If no person detected and no activity for 2 hours, initiate check-in
        if not self.person_detected and self.last_activity_time:
            elapsed = (self.get_clock().now() - self.last_activity_time).nanoseconds / 1e9
            if elapsed > 7200.0:  # 2 hours
                self.get_logger().info("No activity detected for 2 hours - initiating check-in")
                # TODO: Navigate to last known location or bedroom
                # TODO: Send proactive message via LLM
    
    def emergency_protocol(self, emergency_type: str):
        """Handle emergency situations"""
        self.transition_state(RobotState.ALERTING)
        
        alert_msg = String()
        alert_msg.data = json.dumps({
            'type': emergency_type,
            'timestamp': str(self.get_clock().now()),
            'action': 'emergency_alert'
        })
        self.alert_pub.publish(alert_msg)
        
        self.get_logger().error(f"EMERGENCY: {emergency_type}")

def main(args=None):
    rclpy.init(args=args)
    node = AIBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

