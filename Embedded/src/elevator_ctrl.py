#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ElevatorCtrlNode(Node):
    def __init__(self, args):
        super().__init__('elevator_ctrl')
        ns = f'/{args.elevator}' # elevator
        
        # door 이면, /elevator4/cmd_door 퍼블리셔 생성
        if args.cmd == 'door':
            self.pub = self.create_publisher(Float64, f'{ns}/cmd_door', 10)
            self.get_logger().info(f'Publishing {ns}/cmd_door: value={args.value:.2f}')
            msg = Float64()
            msg.data = args.value
            self.pub.publish(msg)
        # lift 이면, /elevator4/cmd_lift 퍼블리셔 생성
        elif args.cmd == 'lift':
            self.pub = self.create_publisher(Float64, f'{ns}/cmd_lift', 10)
            self.get_logger().info(f'Publishing {ns}/cmd_lift: z={args.value:.2f}')
            msg = Float64()
            msg.data = args.value
            self.pub.publish(msg)
        else:
            raise ValueError('Unknown mode')

# 받은 인자 파싱하기 (엘리베이터 이름 / 명령어 / 값) (elevator4 / door / -0.45)
def parse_args():
    parser = argparse.ArgumentParser(description='ROS2 elevator controller publisher')
    parser.add_argument('elevator', type=str, help='엘리베이터 이름 (elevator, elevator2, elevator3, elevator4)')
    parser.add_argument('cmd', choices=['door', 'lift'], help='door 또는 lift')
    parser.add_argument('value', type=float, help='문 위치(m) 또는 리프트 높이(m)')
    return parser.parse_args()

def main():
    args = parse_args()
    rclpy.init()
    node = ElevatorCtrlNode(args)
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
