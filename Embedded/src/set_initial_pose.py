#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time

class SetInitialPose(Node):
    def __init__(self):
        super().__init__('set_initial_pose')

        # Declare parameters with default values
        # Gazebo spawn: (2.0, 8.0) -> Map coordinates updated
        self.declare_parameter('x', 6.5)  # Gazebo 2.0 -> Map coordinate
        self.declare_parameter('y', -3.0)  # Gazebo 8.0 -> Map coordinate
        self.declare_parameter('yaw', 1.57)  # -1.57 rad (-90도) 회전

        # Initial pose publisher
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Get parameters
        self.x = self.get_parameter('x').get_parameter_value().double_value
        self.y = self.get_parameter('y').get_parameter_value().double_value
        self.yaw = self.get_parameter('yaw').get_parameter_value().double_value
        
        # Wait for publisher to be ready
        time.sleep(2.0)
        
        # Send initial pose
        self.send_initial_pose()
        
        self.get_logger().info(f'Initial pose set: x={self.x}, y={self.y}, yaw={self.yaw} rad')

    def send_initial_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        pose_msg.pose.pose.position.x = self.x
        pose_msg.pose.pose.position.y = self.y
        pose_msg.pose.pose.position.z = 0.0
        
        # Orientation (convert yaw to quaternion)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        # Covariance (uncertainty)
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = 0.25   # x
        pose_msg.pose.covariance[7] = 0.25   # y
        pose_msg.pose.covariance[35] = 0.06  # yaw
        
        # Publish multiple times to ensure delivery
        for i in range(5):
            self.pose_pub.publish(pose_msg)
            time.sleep(0.1)
        
        self.get_logger().info('Initial pose published successfully!')

def main():
    rclpy.init()
    node = SetInitialPose()
    
    # 잠시 대기 후 종료
    time.sleep(1.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()