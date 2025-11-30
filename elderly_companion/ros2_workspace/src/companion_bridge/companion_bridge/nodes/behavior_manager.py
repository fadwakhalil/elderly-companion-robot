#!/usr/bin/env python3
"""
Behavior Manager Node
Manages robot behavior state machine and autonomous behaviors
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from enum import Enum
import json
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class BehaviorState(Enum):
    """Robot behavior states"""
    IDLE = "idle"                    # Waiting, listening
    PATROLLING = "patrolling"        # Moving through home
    INTERACTING = "interacting"      # In conversation
    ASSISTING = "assisting"          # Helping with task
    ALERTING = "alerting"            # Emergency detected
    CHARGING = "charging"            # Returning to charger

class BehaviorManager(Node):
    """
    Manages robot behavior state machine
    """
    def __init__(self):
        super().__init__('behavior_manager')
        
        # State
        self.current_state = BehaviorState.IDLE
        self.state_history = []
        
        # Patrolling configuration
        self.patrol_waypoints = []  # List of (x, y, name) tuples
        self.current_waypoint_index = 0
        
        # Activity monitoring
        self.last_activity_time = None
        self.activity_timeout = 7200.0  # 2 hours in seconds
        
        # Subscribers
        self.state_sub = self.create_subscription(
            String,
            '/companion/state',
            self.state_callback,
            10
        )
        
        self.vision_sub = self.create_subscription(
            String,
            '/companion/vision/detections',
            self.vision_callback,
            10
        )
        
        # Publishers
        self.behavior_pub = self.create_publisher(
            String,
            '/companion/behavior',
            10
        )
        
        self.nav_goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        # Timers
        self.state_timer = self.create_timer(1.0, self.state_loop)
        self.patrol_timer = self.create_timer(30.0, self.patrol_loop)
        self.activity_timer = self.create_timer(60.0, self.activity_check)
        
        self.get_logger().info("Behavior Manager started")
        self.get_logger().info(f"Initial state: {self.current_state.value}")
    
    def state_callback(self, msg: String):
        """Handle state updates from AI bridge"""
        try:
            state_data = json.loads(msg.data)
            new_state_str = state_data.get('state')
            if new_state_str:
                try:
                    new_state = BehaviorState(new_state_str)
                    self.transition_to_state(new_state)
                except ValueError:
                    self.get_logger().warn(f"Unknown state: {new_state_str}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse state message: {e}")
    
    def vision_callback(self, msg: String):
        """Handle vision detections"""
        try:
            detection_data = json.loads(msg.data)
            person_detected = detection_data.get('person_detected', False)
            
            if person_detected:
                self.last_activity_time = self.get_clock().now()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse vision data: {e}")
    
    def transition_to_state(self, new_state: BehaviorState):
        """Transition to a new behavior state"""
        if self.current_state != new_state:
            old_state = self.current_state
            self.current_state = new_state
            self.state_history.append({
                'from': old_state.value,
                'to': new_state.value,
                'timestamp': str(self.get_clock().now())
            })
            
            self.get_logger().info(f"Behavior transition: {old_state.value} -> {new_state.value}")
            self.on_state_entered(new_state)
    
    def on_state_entered(self, state: BehaviorState):
        """Handle actions when entering a state"""
        behavior_msg = String()
        behavior_msg.data = json.dumps({
            'state': state.value,
            'action': self.get_state_action(state),
            'timestamp': str(self.get_clock().now())
        })
        self.behavior_pub.publish(behavior_msg)
        
        if state == BehaviorState.PATROLLING:
            self.start_patrolling()
        elif state == BehaviorState.CHARGING:
            self.go_to_charger()
        elif state == BehaviorState.ALERTING:
            self.handle_emergency()
    
    def get_state_action(self, state: BehaviorState) -> str:
        """Get action description for state"""
        actions = {
            BehaviorState.IDLE: "waiting",
            BehaviorState.PATROLLING: "patrolling",
            BehaviorState.INTERACTING: "interacting",
            BehaviorState.ASSISTING: "assisting",
            BehaviorState.ALERTING: "alerting",
            BehaviorState.CHARGING: "charging"
        }
        return actions.get(state, "unknown")
    
    def start_patrolling(self):
        """Start patrolling behavior"""
        if not self.patrol_waypoints:
            self.get_logger().warn("No patrol waypoints defined")
            return
        
        self.get_logger().info("Starting patrol route")
        self.navigate_to_waypoint(self.patrol_waypoints[0])
    
    def navigate_to_waypoint(self, waypoint):
        """Navigate to a specific waypoint"""
        x, y, name = waypoint
        self.get_logger().info(f"Navigating to waypoint: {name} at ({x}, {y})")
        
        # Create navigation goal
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.nav_goal_pub.publish(goal)
    
    def go_to_charger(self):
        """Navigate to charging station"""
        # Assuming charger is at origin (0, 0)
        charger_waypoint = (0.0, 0.0, "charger")
        self.navigate_to_waypoint(charger_waypoint)
    
    def handle_emergency(self):
        """Handle emergency situation"""
        self.get_logger().error("EMERGENCY PROTOCOL ACTIVATED")
        # Stop all movement
        # Alert family
        # Attempt to reach user
    
    def state_loop(self):
        """Main state management loop"""
        # State-specific behaviors
        if self.current_state == BehaviorState.PATROLLING:
            # Check if reached waypoint, move to next
            pass
        elif self.current_state == BehaviorState.INTERACTING:
            # Monitor interaction timeout
            pass
    
    def patrol_loop(self):
        """Patrolling behavior loop"""
        if self.current_state == BehaviorState.PATROLLING:
            # Cycle through waypoints
            if self.patrol_waypoints:
                # TODO: Check if reached current waypoint
                # If yes, move to next
                pass
    
    def activity_check(self):
        """Check for user activity and trigger proactive behaviors"""
        if self.current_state == BehaviorState.IDLE:
            if self.last_activity_time:
                elapsed = (self.get_clock().now() - self.last_activity_time).nanoseconds / 1e9
                if elapsed > self.activity_timeout:
                    self.get_logger().info("No activity detected - initiating proactive check-in")
                    # Transition to patrolling to check on user
                    self.transition_to_state(BehaviorState.PATROLLING)

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

