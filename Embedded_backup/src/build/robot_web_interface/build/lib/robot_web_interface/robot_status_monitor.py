#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import json
import time

class RobotStatusMonitor(Node):
    def __init__(self):
        super().__init__('robot_status_monitor')

        # 로봇 상태를 퍼블리시하는 퍼블리셔
        self.status_publisher = self.create_publisher(String, 'robot_current_state', 10)

        # 로봇의 움직임을 감지하는 서브스크라이버들
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # 상태 변수들
        self.last_movement_time = time.time()
        self.is_moving = False
        self.current_status = "idle"
        self.movement_threshold = 0.01  # 움직임 감지 임계값
        self.idle_timeout = 5.0  # 5초 동안 움직임이 없으면 idle 상태

        # 주기적으로 상태를 체크하고 퍼블리시하는 타이머
        self.status_timer = self.create_timer(1.0, self.update_status)

        self.get_logger().info('Robot Status Monitor initialized')

    def cmd_vel_callback(self, msg):
        """cmd_vel 토픽을 통해 로봇 명령 감지"""
        linear_speed = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.linear.z)
        angular_speed = abs(msg.angular.x) + abs(msg.angular.y) + abs(msg.angular.z)

        if linear_speed > self.movement_threshold or angular_speed > self.movement_threshold:
            self.is_moving = True
            self.last_movement_time = time.time()
            if self.current_status != "working":
                self.current_status = "working"
                self.get_logger().info('Robot is now working (movement detected)')

    def odom_callback(self, msg):
        """오도메트리를 통해 실제 로봇 움직임 감지"""
        linear_speed = abs(msg.twist.twist.linear.x) + abs(msg.twist.twist.linear.y)
        angular_speed = abs(msg.twist.twist.angular.z)

        if linear_speed > self.movement_threshold or angular_speed > self.movement_threshold:
            self.is_moving = True
            self.last_movement_time = time.time()

    def update_status(self):
        """로봇 상태를 업데이트하고 퍼블리시"""
        current_time = time.time()

        # 일정 시간 동안 움직임이 없으면 idle 상태로 변경
        if current_time - self.last_movement_time > self.idle_timeout:
            if self.current_status != "idle":
                self.current_status = "idle"
                self.get_logger().info('Robot is now idle')
            self.is_moving = False

        # 상태 메시지 퍼블리시
        status_msg = String()
        status_msg.data = self.current_status
        self.status_publisher.publish(status_msg)

    def set_status(self, status):
        """외부에서 로봇 상태를 수동으로 설정하는 메서드"""
        valid_statuses = ["idle", "working", "error", "charging", "maintenance"]
        if status in valid_statuses:
            self.current_status = status
            self.get_logger().info(f'Robot status manually set to: {status}')
        else:
            self.get_logger().warn(f'Invalid status: {status}. Valid statuses: {valid_statuses}')

def main(args=None):
    rclpy.init(args=args)

    robot_status_monitor = RobotStatusMonitor()

    try:
        rclpy.spin(robot_status_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        robot_status_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()