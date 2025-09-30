#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math

class AutonomousNavigation(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Navigation2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Goal positions (여기에 원하는 좌표들을 설정)
        self.goals = [
            {'x': 6.5, 'y': 0.0},
            {'x': 3.0, 'y': 2.0},
            {'x': 0.0, 'y': 0.0}
        ]
        self.current_goal_index = 0
        
        # Current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Control parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.8
        self.distance_tolerance = 0.2
        self.angle_tolerance = 0.1
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Autonomous Navigation Started')
        self.get_logger().info(f'Goals: {self.goals}')

    def odom_callback(self, msg):
        # Update current position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def send_nav2_goal(self, x, y):
        """Send goal to Navigation2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation2 server not available')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Sending goal to Navigation2: ({x}, {y})')
        self.nav_client.send_goal_async(goal_msg)
        return True

    def simple_navigation(self, goal_x, goal_y):
        """Simple navigation without mapping"""
        # Calculate distance and angle to goal
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = angle_to_goal - self.current_yaw
        
        # Normalize angle difference
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        twist = Twist()
        
        if distance > self.distance_tolerance:
            if abs(angle_diff) > self.angle_tolerance:
                # Rotate towards goal
                twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                self.get_logger().info(f'Rotating: angle_diff = {angle_diff:.2f}°')
            else:
                # Move towards goal
                twist.linear.x = self.linear_speed
                self.get_logger().info(f'Moving forward: distance = {distance:.2f}m')
                
            self.cmd_vel_pub.publish(twist)
            return False  # Goal not reached
        else:
            # Goal reached
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f'Goal reached! Position: ({self.current_x:.2f}, {self.current_y:.2f})')
            return True  # Goal reached

    def control_loop(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('All goals completed!')
            rclpy.shutdown()
            return
        
        current_goal = self.goals[self.current_goal_index]
        goal_x = current_goal['x']
        goal_y = current_goal['y']
        
        # Try Navigation2 first, fallback to simple navigation
        if self.current_goal_index == 0:  # First goal, try Nav2
            if self.nav_client.wait_for_server(timeout_sec=1.0):
                self.send_nav2_goal(goal_x, goal_y)
                self.current_goal_index += 1
                return
        
        # Use simple navigation
        if self.simple_navigation(goal_x, goal_y):
            self.current_goal_index += 1
            if self.current_goal_index < len(self.goals):
                next_goal = self.goals[self.current_goal_index]
                self.get_logger().info(f'Moving to next goal: ({next_goal["x"]}, {next_goal["y"]})')

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()