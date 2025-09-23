#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import json
import threading
import time
import subprocess
import re
from datetime import datetime, timezone

class RobotStatusHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, robot_node=None, **kwargs):
        self.robot_node = robot_node
        super().__init__(*args, **kwargs)

    def do_GET(self):
        """GET 요청 처리"""
        try:
            # URL 파싱
            parsed_url = urlparse(self.path)

            # 경로별 처리
            if parsed_url.path == '/robot/status':
                self.handle_robot_status(parsed_url)
            elif parsed_url.path.startswith('/robot/delivery/') and parsed_url.path.endswith('/addresses'):
                self.handle_robot_delivery(parsed_url)
            else:
                self.send_error_response(404, "Not Found", "Endpoint not found")

        except Exception as e:
            self.robot_node.get_logger().error(f'Error processing GET request: {str(e)}')
            self.send_error_response(500, "Internal Server Error", str(e))

    def handle_robot_status(self, parsed_url):
        """로봇 상태 요청 처리"""
        try:
            # 쿼리 파라미터에서 robotId 추출
            query_params = parse_qs(parsed_url.query)
            robot_id = query_params.get('robotId', [None])[0]

            if not robot_id:
                self.send_error_response(400, "Bad Request", "robotId parameter is required")
                return

            # 요청 정보 로깅
            user_agent = self.headers.get('User-Agent', 'Unknown')
            self.robot_node.get_logger().info(f'Received robot status request for ID: {robot_id} from {self.client_address[0]} (User-Agent: {user_agent})')

            # 로봇 상태 확인
            robot_status = self.robot_node.get_current_robot_status()
            self.robot_node.get_logger().info(f'Current robot status: {robot_status}')

            # 상태에 따라 응답 생성
            if robot_status in ["idle", "ready"]:
                status = "valid"
                message = "작업 가능"
            elif robot_status in ["working", "busy", "moving"]:
                status = "invalid"
                message = "작업 중"
            elif robot_status in ["error", "fault"]:
                status = "invalid"
                message = "오류 상태"
            elif robot_status in ["charging"]:
                status = "invalid"
                message = "충전 중"
            elif robot_status in ["maintenance"]:
                status = "invalid"
                message = "점검 중"
            else:
                status = "invalid"
                message = "작업 불가능"

            # 응답 데이터 생성
            response_data = {
                "status": status,
                "message": message,
                "timestamp": datetime.now(timezone.utc).isoformat()
            }

            # JSON 응답 전송
            self.send_json_response(200, response_data)

            self.robot_node.get_logger().info(f'Sent robot status response: {response_data}')

        except Exception as e:
            self.robot_node.get_logger().error(f'Error handling robot status request: {str(e)}')
            self.send_error_response(500, "Internal Server Error", str(e))

    def handle_robot_delivery(self, parsed_url):
        """로봇 배송 요청 처리"""
        try:
            # URL에서 deliveryId 추출
            path_parts = parsed_url.path.split('/')
            delivery_id = path_parts[3] if len(path_parts) > 3 else None

            if not delivery_id:
                self.send_error_response(400, "Bad Request", "Invalid delivery path")
                return

            # 요청 body 읽기
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error_response(400, "Bad Request", "Request body is required")
                return

            post_data = self.rfile.read(content_length)
            try:
                request_data = json.loads(post_data.decode('utf-8'))
            except json.JSONDecodeError:
                self.send_error_response(400, "Bad Request", "Invalid JSON format")
                return

            # 필수 필드 확인
            seller_address = request_data.get('sellerAddress')
            buyer_address = request_data.get('buyerAddress')

            if not seller_address or not buyer_address:
                self.send_error_response(400, "Bad Request", "sellerAddress and buyerAddress are required")
                return

            self.robot_node.get_logger().info(f'Received delivery request {delivery_id}: {seller_address} -> {buyer_address}')

            # 로봇 상태 확인
            robot_status = self.robot_node.get_current_robot_status()
            if robot_status not in ["idle", "ready"]:
                response_data = {
                    "success": False,
                    "message": f"로봇이 작업 불가능한 상태입니다: {robot_status}",
                    "timestamp": datetime.now(timezone.utc).isoformat()
                }
                self.send_json_response(400, response_data)
                return

            # 주소 파싱 및 로봇 이동 실행
            success, message = self.robot_node.execute_delivery(seller_address, buyer_address, delivery_id)

            # 응답 생성
            response_data = {
                "success": success,
                "message": message,
                "deliveryId": delivery_id,
                "timestamp": datetime.now(timezone.utc).isoformat()
            }

            status_code = 200 if success else 400
            self.send_json_response(status_code, response_data)

            self.robot_node.get_logger().info(f'Sent delivery response: {response_data}')

        except Exception as e:
            self.robot_node.get_logger().error(f'Error handling delivery request: {str(e)}')
            self.send_error_response(500, "Internal Server Error", str(e))

    def send_json_response(self, status_code, data):
        """JSON 응답 전송"""
        response_json = json.dumps(data, ensure_ascii=False, indent=2)

        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Access-Control-Allow-Origin', '*')  # CORS 허용
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

        self.wfile.write(response_json.encode('utf-8'))

    def send_error_response(self, status_code, error, message):
        """에러 응답 전송"""
        error_data = {
            "error": error,
            "message": message,
            "timestamp": datetime.now(timezone.utc).isoformat()
        }
        self.send_json_response(status_code, error_data)

    def do_OPTIONS(self):
        """CORS preflight 요청 처리"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def log_message(self, format, *args):
        """HTTP 서버 로그를 ROS 로거로 리다이렉트"""
        if self.robot_node:
            self.robot_node.get_logger().info(f'HTTP: {format % args}')

class RobotHttpServer(Node):
    def __init__(self):
        super().__init__('robot_http_server')

        # 파라미터 선언
        self.declare_parameter('server_port', 8888)
        self.declare_parameter('server_host', '0.0.0.0')

        # 파라미터 가져오기
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.server_host = self.get_parameter('server_host').get_parameter_value().string_value

        # 로봇 상태 변수
        self.current_robot_status = "idle"
        self.last_status_update = time.time()

        # 로봇 상태 구독
        self.robot_status_sub = self.create_subscription(
            String,
            'robot_current_state',
            self.robot_status_callback,
            10
        )

        # HTTP 서버 설정
        def handler_factory(*args, **kwargs):
            return RobotStatusHandler(*args, robot_node=self, **kwargs)

        self.http_server = HTTPServer((self.server_host, self.server_port), handler_factory)

        # 별도 스레드에서 HTTP 서버 실행
        self.server_thread = threading.Thread(target=self.run_http_server, daemon=True)
        self.server_thread.start()

        # 상태 체크 타이머 (5초마다 로봇 상태 확인)
        self.status_check_timer = self.create_timer(5.0, self.check_robot_status)

        self.get_logger().info(f'Robot HTTP Server started on {self.server_host}:{self.server_port}')
        self.get_logger().info(f'Available endpoint: http://{self.server_host}:{self.server_port}/robot/status?robotId=<id>')

    def run_http_server(self):
        """HTTP 서버 실행"""
        try:
            self.get_logger().info('HTTP server thread started')
            self.http_server.serve_forever()
        except Exception as e:
            self.get_logger().error(f'HTTP server error: {str(e)}')

    def robot_status_callback(self, msg):
        """로봇 상태 업데이트 콜백"""
        self.current_robot_status = msg.data
        self.last_status_update = time.time()
        self.get_logger().debug(f'Robot status updated to: {self.current_robot_status}')

    def check_robot_status(self):
        """로봇 상태 주기적 체크"""
        current_time = time.time()

        # 10초 이상 상태 업데이트가 없으면 연결 끊김으로 간주
        if current_time - self.last_status_update > 10.0:
            self.current_robot_status = "disconnected"
            self.get_logger().warn('Robot status not updated for 10 seconds - marking as disconnected')

    def get_current_robot_status(self):
        """현재 로봇 상태 반환"""
        return self.current_robot_status

    def parse_address(self, address):
        """주소 파싱: '1동 101호' -> (동, 층)"""
        try:
            # 정규식으로 동과 호수 추출
            match = re.match(r'(\d+)동\s*(\d+)호', address)
            if not match:
                return None, None

            dong = int(match.group(1))
            ho = int(match.group(2))

            # 호수의 백의 자리가 층수
            floor = ho // 100

            return dong, floor
        except Exception as e:
            self.get_logger().error(f'Address parsing error: {str(e)}')
            return None, None

    def execute_delivery(self, seller_address, buyer_address, delivery_id):
        """배송 실행"""
        try:
            # 시작 주소 파싱
            start_dong, start_floor = self.parse_address(seller_address)
            if start_dong is None:
                return False, f"잘못된 시작 주소 형식: {seller_address}"

            # 도착 주소 파싱
            end_dong, end_floor = self.parse_address(buyer_address)
            if end_dong is None:
                return False, f"잘못된 도착 주소 형식: {buyer_address}"

            self.get_logger().info(f'Parsed addresses - Start: {start_dong}동 {start_floor}층, End: {end_dong}동 {end_floor}층')

            # go_room.py 스크립트 경로
            script_path = "/home/donggun/S13P21A501/Embedded/src/go_room.py"

            # 시작 위치로 이동
            self.get_logger().info(f'Moving to start location: room {start_dong}')
            result = subprocess.run(
                ["python3", script_path, str(start_dong)],
                capture_output=True,
                text=True,
                timeout=120  # 2분 타임아웃
            )

            if result.returncode != 0:
                error_msg = f"시작 위치 이동 실패: {result.stderr}"
                self.get_logger().error(error_msg)
                return False, error_msg

            # 도착 위치로 이동
            self.get_logger().info(f'Moving to destination: room {end_dong}')
            result = subprocess.run(
                ["python3", script_path, str(end_dong)],
                capture_output=True,
                text=True,
                timeout=120  # 2분 타임아웃
            )

            if result.returncode != 0:
                error_msg = f"도착 위치 이동 실패: {result.stderr}"
                self.get_logger().error(error_msg)
                return False, error_msg

            success_msg = f"배송 완료: {seller_address} -> {buyer_address}"
            self.get_logger().info(success_msg)
            return True, success_msg

        except subprocess.TimeoutExpired:
            error_msg = "로봇 이동 시간 초과"
            self.get_logger().error(error_msg)
            return False, error_msg
        except Exception as e:
            error_msg = f"배송 실행 오류: {str(e)}"
            self.get_logger().error(error_msg)
            return False, error_msg

    def shutdown_server(self):
        """서버 종료"""
        if self.http_server:
            self.get_logger().info('Shutting down HTTP server')
            self.http_server.shutdown()
            self.http_server.server_close()

def main(args=None):
    rclpy.init(args=args)

    robot_http_server = RobotHttpServer()

    try:
        rclpy.spin(robot_http_server)
    except KeyboardInterrupt:
        pass
    finally:
        robot_http_server.shutdown_server()
        robot_http_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()