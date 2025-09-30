#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class AptDoorCtrlNode(Node):
    def __init__(self, args):
        super().__init__('apt_door_ctrl')
        ns = f'/apt_door/{args.floor}'  # Namespace for the specific floor
        
        # Create publisher for door control
        self.pub = self.create_publisher(Float64, f'{ns}/cmd_door', 10)
        self.get_logger().info(f'Publishing {ns}/cmd_door: value={args.value:.2f}')
        msg = Float64()
        msg.data = args.value
        self.pub.publish(msg)

# Parse command line arguments (floor name and door position)
def parse_args():
    parser = argparse.ArgumentParser(description='ROS2 automatic door controller publisher')
    parser.add_argument('floor', type=str, help='Floor name (e.g., floor1, floor2, etc.)')
    parser.add_argument('value', type=float, help='Door position (open: -0.5, close: 0.0)')
    return parser.parse_args()

def main():
    args = parse_args()
    rclpy.init()
    node = AptDoorCtrlNode(args)
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()