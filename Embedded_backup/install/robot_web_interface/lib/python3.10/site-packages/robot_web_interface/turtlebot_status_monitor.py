#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import time

class TurtleBotStatusMonitor(Node):
    def __init__(self):
        super().__init__('turtlebot_status_monitor')

        # 로봇 상태를 퍼블리시하는 퍼블리셔 (robot_current_state 토픽 사용)
        self.status_publisher = self.create_publisher(String, 'robot_current_state', 10)

        # TurtleBot 관련 토픽 구독
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.joint_states_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # 상태 변수들
        self.last_cmd_vel_time = 0
        self.last_odom_time = 0
        self.last_joint_states_time = 0
        self.last_movement_time = 0  # 실제 움직임이 감지될 때까지 0으로 초기화

        self.is_moving = False
        self.current_status = "checking"  # 시작은 checking
        self.movement_threshold = 0.01
        self.idle_timeout = 3.0  # 3초 동안 움직임이 없으면 idle
        self.connection_timeout = 10.0  # 10초 동안 토픽이 없으면 disconnected

        # TurtleBot 연결 상태
        self.has_odom = False
        self.has_joint_states = False
        self.topics_checked = False

        # 주기적으로 상태를 체크하고 퍼블리시하는 타이머
        self.status_timer = self.create_timer(2.0, self.update_status)

        # 초기 체크 타이머 (5초 후에 토픽 상태 확인)
        self.initial_check_timer = self.create_timer(5.0, self.initial_topic_check)

        self.get_logger().info('TurtleBot Status Monitor initialized - checking Gazebo TurtleBot status')

    def cmd_vel_callback(self, msg):
        """cmd_vel 토픽을 통해 로봇 명령 감지"""
        self.last_cmd_vel_time = time.time()

        linear_speed = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.linear.z)
        angular_speed = abs(msg.angular.x) + abs(msg.angular.y) + abs(msg.angular.z)

        if linear_speed > self.movement_threshold or angular_speed > self.movement_threshold:
            self.is_moving = True
            self.last_movement_time = time.time()
            if self.current_status not in ["working", "moving"]:
                self.current_status = "working"
                self.get_logger().info('TurtleBot is now working (movement command detected)')
        else:
            # 움직임 명령이 없으면 moving 상태 해제
            self.is_moving = False

    def odom_callback(self, msg):
        """오도메트리를 통해 실제 로봇 움직임 감지"""
        self.last_odom_time = time.time()
        self.has_odom = True

        linear_speed = abs(msg.twist.twist.linear.x) + abs(msg.twist.twist.linear.y)
        angular_speed = abs(msg.twist.twist.angular.z)

        if linear_speed > self.movement_threshold or angular_speed > self.movement_threshold:
            self.is_moving = True
            self.last_movement_time = time.time()
        else:
            # 실제 움직임이 없으면 moving 상태 해제
            self.is_moving = False

    def joint_states_callback(self, msg):
        """관절 상태를 통해 로봇 활성 상태 확인"""
        self.last_joint_states_time = time.time()
        self.has_joint_states = True

    def initial_topic_check(self):
        """초기 토픽 상태 확인"""
        current_time = time.time()

        if self.has_odom and self.has_joint_states:
            self.current_status = "idle"
            self.get_logger().info('TurtleBot is connected and ready - status set to idle')
        elif self.has_odom or self.has_joint_states:
            self.current_status = "ready"
            self.get_logger().info('TurtleBot partially connected - status set to ready')
        else:
            self.current_status = "disconnected"
            self.get_logger().warn('TurtleBot not detected - status set to disconnected')

        self.topics_checked = True

        # 한 번만 실행하도록 타이머 취소
        self.initial_check_timer.cancel()

    def update_status(self):
        """로봇 상태를 업데이트하고 퍼블리시"""
        current_time = time.time()

        # 초기 체크가 완료되지 않았으면 대기
        if not self.topics_checked:
            self.current_status = "checking"
        else:
            # 토픽 연결 상태 확인
            odom_connected = (current_time - self.last_odom_time) < self.connection_timeout if self.last_odom_time > 0 else False
            joint_connected = (current_time - self.last_joint_states_time) < self.connection_timeout if self.last_joint_states_time > 0 else False

            if not odom_connected and not joint_connected:
                self.current_status = "disconnected"
            elif self.is_moving or (self.last_movement_time > 0 and (current_time - self.last_movement_time) < self.idle_timeout):
                self.current_status = "working"
            else:
                # 토픽이 활성화되어 있고 움직이지 않는 상태 = 작업 가능
                self.current_status = "idle"
                self.is_moving = False

        # 상태 메시지 퍼블리시
        status_msg = String()
        status_msg.data = self.current_status
        self.status_publisher.publish(status_msg)

        # 로그 출력 (디버깅용)
        if hasattr(self, '_last_logged_status') and self._last_logged_status != self.current_status:
            self.get_logger().info(f'TurtleBot status changed to: {self.current_status}')
        self._last_logged_status = self.current_status

def main(args=None):
    rclpy.init(args=args)

    turtlebot_status_monitor = TurtleBotStatusMonitor()

    try:
        rclpy.spin(turtlebot_status_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        turtlebot_status_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()