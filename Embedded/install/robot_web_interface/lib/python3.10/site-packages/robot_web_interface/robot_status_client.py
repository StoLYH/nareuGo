#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
import json
from std_msgs.msg import String
import threading
import time

class RobotStatusClient(Node):
    def __init__(self):
        super().__init__('robot_status_client')

        # 파라미터 선언
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('api_base_url', 'https://j13a501.p.ssafy.io/api')
        self.declare_parameter('access_token', 'your_access_token')
        self.declare_parameter('refresh_token', 'your_refresh_token')
        self.declare_parameter('status_interval', 10.0)  # 10초마다 상태 전송

        # 파라미터 가져오기 - 문자열로 변환
        robot_id_param = self.get_parameter('robot_id').get_parameter_value()
        if robot_id_param.type == robot_id_param.Type.INTEGER:
            self.robot_id = str(robot_id_param.integer_value)
        else:
            self.robot_id = robot_id_param.string_value
        self.api_base_url = self.get_parameter('api_base_url').get_parameter_value().string_value
        self.access_token = self.get_parameter('access_token').get_parameter_value().string_value
        self.refresh_token = self.get_parameter('refresh_token').get_parameter_value().string_value
        self.status_interval = self.get_parameter('status_interval').get_parameter_value().double_value

        # 퍼블리셔 생성 - 로봇 상태를 다른 노드에게 알림
        self.status_publisher = self.create_publisher(String, 'robot_status_response', 10)

        # 서브스크라이버 생성 - 로봇의 현재 상태를 받음
        self.robot_state_subscriber = self.create_subscription(
            String,
            'robot_current_state',
            self.robot_state_callback,
            10
        )

        # 현재 로봇 상태
        self.current_robot_status = "idle"  # idle, working, error 등

        # 주기적으로 상태 전송하는 타이머
        self.status_timer = self.create_timer(self.status_interval, self.send_status_to_server)

        self.get_logger().info(f'Robot Status Client initialized for robot ID: {self.robot_id}')

    def robot_state_callback(self, msg):
        """다른 노드로부터 로봇 상태 업데이트를 받음"""
        self.current_robot_status = msg.data
        self.get_logger().info(f'Robot status updated to: {self.current_robot_status}')

    def send_status_to_server(self):
        """서버에 로봇 상태를 전송"""
        try:
            # GET 요청으로 변경, robotId를 URL 파라미터로 전달
            url = f"{self.api_base_url}/robot/status"

            headers = {
                'Authorization': f'Bearer {self.access_token}',
                'Content-Type': 'application/json'
            }

            cookies = {
                'HTTP Only Cookie': f'Bearer {self.refresh_token}'
            }

            # URL 쿼리 파라미터로 robotId 전달
            params = {
                'robotId': self.robot_id
            }

            response = requests.get(
                url,
                headers=headers,
                cookies=cookies,
                params=params,
                timeout=5
            )

            if response.status_code == 200:
                response_data = response.json()
                # 백엔드 응답 형식: {"robotId": 1, "robotName": "NareuGO"}
                robot_id = response_data.get('robotId')
                robot_name = response_data.get('robotName')

                self.get_logger().info(f'Robot found - ID: {robot_id}, Name: {robot_name}')

                # 응답을 토픽으로 퍼블리시
                status_msg = String()
                status_msg.data = json.dumps(response_data)
                self.status_publisher.publish(status_msg)

            else:
                self.get_logger().error(f'Failed to send status. Status code: {response.status_code}')

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Network error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {str(e)}')

    def get_status_message(self, status):
        """상태에 따른 메시지 반환"""
        status_messages = {
            "idle": "작업 가능",
            "working": "작업 중",
            "error": "오류 발생",
            "charging": "충전 중",
            "maintenance": "점검 중"
        }
        return status_messages.get(status, "알 수 없는 상태")

    def check_robot_availability(self):
        """로봇 작업 가능 여부를 서버에 확인 (필요시 호출)"""
        try:
            url = f"{self.api_base_url}/robot/status"

            headers = {
                'Authorization': f'Bearer {self.access_token}',
                'Content-Type': 'application/json'
            }

            cookies = {
                'HTTP Only Cookie': f'Bearer {self.refresh_token}'
            }

            params = {
                'robotId': self.robot_id
            }

            response = requests.get(
                url,
                headers=headers,
                cookies=cookies,
                params=params,
                timeout=5
            )

            if response.status_code == 200:
                response_data = response.json()
                self.get_logger().info(f'Robot availability check: {response_data}')
                return response_data
            else:
                self.get_logger().error(f'Failed to check availability. Status code: {response.status_code}')
                return None

        except Exception as e:
            self.get_logger().error(f'Error checking robot availability: {str(e)}')
            return None

def main(args=None):
    rclpy.init(args=args)

    robot_status_client = RobotStatusClient()

    try:
        rclpy.spin(robot_status_client)
    except KeyboardInterrupt:
        pass
    finally:
        robot_status_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()