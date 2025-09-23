#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
import json
from std_msgs.msg import String
import time

class DeliveryAddressClient(Node):
    def __init__(self):
        super().__init__('delivery_address_client')

        # 파라미터 선언
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('robot_name', 'NareuGO')
        self.declare_parameter('api_base_url', 'http://j13a501.p.ssafy.io/api')
        self.declare_parameter('access_token', 'your_access_token')
        self.declare_parameter('refresh_token', 'your_refresh_token')

        # 파라미터 가져오기
        robot_id_param = self.get_parameter('robot_id').get_parameter_value()
        if robot_id_param.type == robot_id_param.Type.INTEGER:
            self.robot_id = str(robot_id_param.integer_value)
        else:
            self.robot_id = robot_id_param.string_value

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.api_base_url = self.get_parameter('api_base_url').get_parameter_value().string_value
        self.access_token = self.get_parameter('access_token').get_parameter_value().string_value
        self.refresh_token = self.get_parameter('refresh_token').get_parameter_value().string_value

        # 퍼블리셔 생성
        self.addresses_response_pub = self.create_publisher(String, 'delivery_addresses_response', 10)

        # 서브스크라이버 생성 - 주소 요청을 받음
        self.addresses_request_sub = self.create_subscription(
            String,
            'delivery_addresses_request',
            self.handle_addresses_request,
            10
        )

        self.get_logger().info(f'Delivery Address Client initialized for robot: {self.robot_name} (ID: {self.robot_id})')

    def handle_addresses_request(self, msg):
        """배송 주소 요청 처리"""
        try:
            request_data = json.loads(msg.data)
            delivery_id = request_data.get('delivery_id')

            if not delivery_id:
                self.get_logger().error('No delivery_id provided in addresses request')
                return

            self.get_logger().info(f'Fetching addresses for delivery ID: {delivery_id}')

            # 백엔드에서 주소 정보 가져오기
            addresses_data = self.fetch_delivery_addresses(delivery_id)

            if addresses_data:
                # 주소 정보를 토픽으로 퍼블리시
                response_msg = String()
                response_msg.data = json.dumps({
                    'delivery_id': delivery_id,
                    'addresses': addresses_data,
                    'timestamp': int(time.time()),
                    'status': 'success'
                })
                self.addresses_response_pub.publish(response_msg)

                self.get_logger().info(f'Successfully fetched addresses for delivery {delivery_id}')
            else:
                # 에러 응답 퍼블리시
                error_msg = String()
                error_msg.data = json.dumps({
                    'delivery_id': delivery_id,
                    'error': 'Failed to fetch addresses from backend',
                    'timestamp': int(time.time()),
                    'status': 'error'
                })
                self.addresses_response_pub.publish(error_msg)

                self.get_logger().error(f'Failed to fetch addresses for delivery {delivery_id}')

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in addresses request')
        except Exception as e:
            self.get_logger().error(f'Error handling addresses request: {str(e)}')

    def fetch_delivery_addresses(self, delivery_id):
        """백엔드에서 배송 주소 정보 가져오기"""
        try:
            url = f"{self.api_base_url}/robot/delivery/{delivery_id}/addresses"

            headers = {
                'Authorization': f'Bearer {self.access_token}',
                'Content-Type': 'application/json'
            }

            cookies = {
                'HTTP Only Cookie': f'Bearer {self.refresh_token}'
            }

            response = requests.get(url, headers=headers, cookies=cookies, timeout=10)

            if response.status_code == 200:
                addresses_data = response.json()
                self.get_logger().info(f'Retrieved addresses from backend: {addresses_data}')
                return addresses_data
            else:
                self.get_logger().error(f'Backend returned error code: {response.status_code}')
                if response.text:
                    self.get_logger().error(f'Error response: {response.text}')
                return None

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Network error during addresses request: {str(e)}')
            return None
        except Exception as e:
            self.get_logger().error(f'Unexpected error fetching addresses: {str(e)}')
            return None

    def test_addresses_fetch(self, delivery_id):
        """주소 조회 테스트용 메서드"""
        self.get_logger().info(f'Testing addresses fetch for delivery ID: {delivery_id}')

        test_request = String()
        test_request.data = json.dumps({
            'delivery_id': delivery_id,
            'timestamp': int(time.time())
        })

        # 자기 자신에게 테스트 요청 전송
        self.handle_addresses_request(test_request)

def main(args=None):
    rclpy.init(args=args)

    delivery_address_client = DeliveryAddressClient()

    # 테스트용 - 5초 후에 샘플 배송 ID로 주소 조회 테스트
    def test_after_delay():
        time.sleep(5)
        delivery_address_client.test_addresses_fetch(5)  # 테스트용 delivery_id

    import threading
    test_thread = threading.Thread(target=test_after_delay)
    test_thread.daemon = True
    test_thread.start()

    try:
        rclpy.spin(delivery_address_client)
    except KeyboardInterrupt:
        pass
    finally:
        delivery_address_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()