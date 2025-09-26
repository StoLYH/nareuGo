#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
import sys
import math
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import json
import threading
from datetime import datetime, timezone
import re
import requests

# 엘리베이터 제어 및 로봇 내비게이션 노드
class QuickRoomNav(Node):
    def __init__(self, elevator_ns: str):
        super().__init__('quick_room_nav')
        self.elevator_ns = elevator_ns  # 예: 'elevator', 'elevator_0', 'elevator_1', 'elevator_2'

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 엘리베이터 통신용 퍼블리셔들 (네임스페이스 적용)
        base = f'/{self.elevator_ns}/robot'
        self.current_location_pub = self.create_publisher(Int32, f'{base}/current_location', 10)
        self.target_floor_pub     = self.create_publisher(Int32, f'{base}/target_floor', 10)
        self.boarded_pub          = self.create_publisher(Bool,  f'{base}/boarded', 10)
        self.alighted_pub         = self.create_publisher(Bool,  f'{base}/alighted', 10)

        # 엘리베이터 상태 구독
        self.state_str = None
        self.arrived_floor = None
        self.state_sub = self.create_subscription(
            String, f'/{self.elevator_ns}/status/state',
            self._on_state, 10)
        self.arrived_sub = self.create_subscription(
            Int32, f'/{self.elevator_ns}/status/arrived',
            self._on_arrived, 10)

        # 아파트 좌표(엘리베이터 키 기반) - 값은 x, y, ori(라디안)
        # 이전 1..4 좌표를 키에 매핑해 반영 (파이: 3.141592)
        self.apartments = {
            "elevator":   {"x": 6.43848,  "y": -4.28445, "ori": -1.57079},  # 1번아파트
            "elevator_0": {"x": 14.7429,"y": -4.21861, "ori": -1.57079},  # 2번아파트
            "elevator_1": {"x": 18.6988,"y":  1.64997, "ori": 0.0 },  # 3번아파트r
            "elevator_2": {"x": 18.7798, "y":  8.10504,  "ori": 0.0 },  # 4번아파트
        }

        # 엘리베이터별 탑승/하차 기준 좌표(형식 통일: x, y, ori 라디안)
        self.boarding_poses = {
            "elevator":   {"x": 6.42516,   "y": -6.03101, "ori": 1.57079},
            "elevator_0": {"x": 14.7095, "y": -6.01259, "ori": 1.57079},
            "elevator_1": {"x": 20.5827,  "y":  1.65911, "ori": 3.141592 },
            "elevator_2": {"x": 20.7613,  "y":  8.12548, "ori": 3.141592 },
        }

        # 아파트별 호수 좌표(사용자 제공)
        # key: 엘리베이터 네임스페이스, value: {호수번호: {"x":..., "y":...}}
        self.rooms = {
            "elevator": {     # 아파트 1
                1: {"x": 7.086, "y": -4.655},
                2: {"x": 6.474, "y": -4.625},
            },
            "elevator_0": {   # 아파트 2
                1: {"x": 15.1433, "y": -4.24452},
                2: {"x": 14.6848, "y": -4.24452},
            },
            "elevator_1": {   # 아파트 3
                1: {"x": 18.910, "y":  2.397},
                2: {"x": 18.862, "y":  0.982},
            },
            "elevator_2": {   # 아파트 4
                1: {"x": 19.145, "y":  8.933},
                2: {"x": 19.191, "y":  7.437},
            },
        }

        # 현재 로봇 위치 (층수)
        self.current_floor = 1

        # 엘리베이터 호출 당시 포즈(AMCL로 자동 스냅샷: 보조용)
        self._last_pose = None              # {"x":..., "y":..., "yaw":...}
        self.entry_pose = None              # 호출 직전 저장 포즈
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)

        self._entry_pose_loaded = False

        # 배송 ID 저장
        self.delivery_id = None

        # 픽업 완료 상태 추적 (더 이상 필요 없음)
        # self.pickup_completed = False
        # self.current_delivery_for_pickup = None

        # 배송 진행 상태 추적
        self.delivery_state = "IDLE"  # IDLE, WAITING_PICKUP, DELIVERING
        self.current_delivery_context = None  # 현재 배송 컨텍스트 저장

    # AMCL 포즈 수신 대기: 최초 포즈가 들어올 때까지 대기
    def _wait_amcl_pose(self):
        if self._last_pose is not None:
            return
        self.get_logger().info("AMCL 포즈 대기 중...")
        while self._last_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self._log_pose("AMCL 최초 포즈 수신")

    # 엘리베이터 상태 수신
    def _on_state(self, msg: String):
        self.state_str = msg.data

    # 층 도착 알림 수신
    def _on_arrived(self, msg: Int32):
        self.arrived_floor = msg.data

    # AMCL 포즈 수신 콜백
    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # 2D yaw
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self._last_pose = {"x": p.x, "y": p.y, "yaw": yaw}

    # 조건(predicate)이 참이 될 때까지 대기 (타임아웃 없음)
    def wait_for(self, predicate, timeout_sec: float, desc: str) -> bool:
        """predicate가 True가 될 때까지 대기 (timeout 제거)"""
        # timeout_sec, desc 인자는 호환성 유지용. 타임아웃 사용 안 함.
        while True:
            # spin_once 대신 짧은 대기로 polling 방식 사용
            import time
            time.sleep(0.1)
            if predicate():
                return True

    # 현재 포즈 로그 출력
    def _log_pose(self, tag: str):
        if self._last_pose:
            self.get_logger().info(
                f"[{tag}] x={self._last_pose['x']:.2f}, y={self._last_pose['y']:.2f}, yaw={math.degrees(self._last_pose['yaw']):.1f}°")
        else:
            self.get_logger().warn(f"[{tag}] 현재 포즈 미수신(/amcl_pose)")

    # 지정한 좌표로 이동시키는 함수
    def _navigate_to_pose_blocking(self, x: float, y: float, yaw: float, timeout_sec: float = 60.0, context: str = "") -> bool:
        """timeout 없이 Nav2 goal 결과까지 대기"""
        # 서버 대기(무기한)
        self.nav_to_pose_client.wait_for_server()

        # 목표 생성
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw/2.0)
        goal.pose.pose.orientation.w = math.cos(yaw/2.0)

        # 목표 전송 및 결과 대기 (polling 방식으로 변경)
        send_future = self.nav_to_pose_client.send_goal_async(goal)

        # spin_until_future_complete 대신 polling 방식 사용
        import time
        while not send_future.done():
            time.sleep(0.1)

        goal_handle = send_future.result()
        # 목표 수락 여부 확인
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"이동 goal 거부/실패{f' ({context})' if context else ''}")
            return False

        # 이동 결과 대기 및 판정 (polling 방식으로 변경)
        result_future = goal_handle.get_result_async()

        while not result_future.done():
            time.sleep(0.1)

        res = result_future.result()
        ok = (res.status == GoalStatus.STATUS_SUCCEEDED)
        if ok:
            self.get_logger().info(f"이동 완료{f' ({context})' if context else ''}")
        else:
            self.get_logger().warn(f"이동 실패(status={res.status}){f' ({context})' if context else ''}")
        return ok

    # 탑승 위치(포즈) 로드
    def _load_boarding_pose(self):
        p = self.boarding_poses.get(self.elevator_ns)
        if p is None:
            self.get_logger().warn(f"boarding_poses에 '{self.elevator_ns}' 키가 없습니다. AMCL 스냅샷으로 대체 시도")
            return False
        self.entry_pose = {"x": float(p["x"]), "y": float(p["y"]), "yaw": float(p["ori"])}
        if not self._entry_pose_loaded:
            self.get_logger().info(
                f"[{self.elevator_ns}] 수동 entry_pose 적용: x={self.entry_pose['x']:.2f}, y={self.entry_pose['y']:.2f}, "
                f"yaw={math.degrees(self.entry_pose['yaw']):.1f}°")
            self._entry_pose_loaded = True
        return True

    # 현재 포즈를 entry_pose로 저장
    def _save_entry_pose(self):
        if self._last_pose:
            self.entry_pose = dict(self._last_pose)
            self.get_logger().info(
                f"entry_pose 저장: x={self.entry_pose['x']:.2f}, y={self.entry_pose['y']:.2f}, yaw={math.degrees(self.entry_pose['yaw']):.1f}°")
        else:
            self.get_logger().warn("entry_pose 저장 실패: 아직 /amcl_pose 수신 전")

    # 탑승 좌표 반환
    def _goto_boarding_pose(self, timeout_sec: float = 90.0) -> bool:
        if not self._load_boarding_pose():
            self._save_entry_pose()
        if not self.entry_pose:
            self.get_logger().error("entry_pose 없음: 탑승 포즈를 알 수 없음")
            return False
        x, y, yaw = self.entry_pose["x"], self.entry_pose["y"], self.entry_pose["yaw"]
        ok = self._navigate_to_pose_blocking(x, y, yaw, context="탑승 좌표(boarding_poses)")
        self._log_pose("엘리베이터 앞(탑승 좌표 도착 후)")
        return ok
    



    # 🛗 엘리베이터 호출→탑승→이동→하차 플로우
    def call_elevator_flow(self, target_floor: int) -> bool:
        self.get_logger().info(f"1) 엘리베이터 호출 플로우 시작: {self.current_floor}층 → {target_floor}층")

        # 1) 엘리베이터 앞(아파트 좌표)로 이동
        apt_pose = self.apartments.get(self.elevator_ns)
        if not apt_pose:
            self.get_logger().error(f"[apt_pose] 없음: {self.elevator_ns}")
            return False
        if not self._navigate_to_pose_blocking(apt_pose["x"], apt_pose["y"], apt_pose["ori"], context="엘리베이터 앞(아파트 좌표)"):
            self.get_logger().error("엘리베이터 앞(아파트 좌표) 이동 실패")
            return False

        # 2) 현재 층 알림(호출)
        cur_msg = Int32(); cur_msg.data = self.current_floor
        self.current_location_pub.publish(cur_msg)
        self.get_logger().info("2) 엘리베이터 앞 도착 → 엘리베이터 호출")

        # 3) 문 열림 상태 대기
        self.wait_for(lambda: (self.state_str in ("OPEN_PICKUP_DOORS", "WAIT_BOARD")),
                      timeout_sec=0.0, desc="doors open at pickup")
        time.sleep(1.5)
        self.get_logger().info(f"[{self.elevator_ns}] 문 열림 감지 → 탑승 좌표로 이동")

        # 4) 탑승 좌표(boarding_poses)로 이동
        if not self._goto_boarding_pose():
            self.get_logger().warn("[_goto_boarding_pose]탑승 좌표 이동 실패: 내부 진입은 계속 진행합니다.")

        # 5) 내부 진입
        inside_x = self.entry_pose["x"]
        inside_y = self.entry_pose["y"]
        inside_yaw = self.entry_pose["yaw"]
        move_ok = self._navigate_to_pose_blocking(inside_x, inside_y, inside_yaw, context="엘리베이터 내부")
        if not move_ok:
            self.get_logger().error("내부로 이동 실패 → 탑승 불가")
            return False
        self.get_logger().info("3) 엘베 탑승")

        # 4) 탑승 완료 + 목적층 전달
        b = Bool(); b.data = True
        self.boarded_pub.publish(b) # 탑승 완료 publish
        tgt = Int32(); tgt.data = target_floor
        self.target_floor_pub.publish(tgt) # 목적층 publish
        self.get_logger().info("4) 엘베 탑승 완료 및 목적층 전달")

        # 5) 목적층 도착/문 열림 대기 (coordinate_plugin에서 상태 갱신)
        self.arrived_floor = None
        self.wait_for(lambda: (self.arrived_floor == target_floor) or
                             (self.state_str in ("OPEN_DEST_DOORS", "WAIT_ALIGHT")),
                    timeout_sec=0.0, desc="arrive at destination")
        self.wait_for(lambda: (self.state_str in ("OPEN_DEST_DOORS", "WAIT_ALIGHT")),
                    timeout_sec=0.0, desc="dest doors open")
        self.get_logger().info("5) 목적층 도착 및 문 열림 대기")

        # 6) 엘베 밖(해당 층 엘베 앞)으로 이동
        dest_x, dest_y, dest_yaw = float(apt_pose["x"]), float(apt_pose["y"]), float(apt_pose["ori"])
        self._navigate_to_pose_blocking(dest_x, dest_y, dest_yaw, context="목적층 엘베 앞")
        self.get_logger().info("6) 엘베 밖으로 이동")

        # 7) 하차 알림 → 문 닫힘
        a = Bool(); a.data = True
        self.alighted_pub.publish(a) # 하차 완료 publish
        self.get_logger().info("7) 하차 알림(문 닫기 완료)")
        self.current_floor = target_floor # 현재 층 갱신
        return True

# HTTP 요청 핸들러 클래스
class RobotStatusHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, robot_node=None, **kwargs):
        self.robot_node = robot_node
        super().__init__(*args, **kwargs)

    # GET 요청 처리 함수
    def do_GET(self):
        try:
            parsed_url = urlparse(self.path)
            query_params = parse_qs(parsed_url.query)

            # 로봇 작업 가능 상태 GET (robotID, delivery_id 파라미터 필요)
            if parsed_url.path == '/robot/status':
                self.handle_robot_status(parsed_url)
            # 배송 주소 GET (sellerAddress, buyerAddress 파라미터 필요)
            elif parsed_url.path == '/robot/delivery/1/addresses':
                self.handle_delivery_addresses(parsed_url)
            else:
                self.send_error_response(404, "Not Found", "Endpoint not found")

        except Exception as e:
            
            self.robot_node.get_logger().error(f'Error processing GET request: {str(e)}')
            self.send_error_response(500, "Internal Server Error", str(e))

    # JSON 응답 전송 함수
    def do_OPTIONS(self):
        """CORS preflight 요청 처리"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    # POST 요청 처리 함수
    def do_POST(self):
        try:
            parsed_url = urlparse(self.path)

            # 픽업 완료 알림 처리 (/robot/delivery/{deliveryId}/seller/placed)
            if parsed_url.path.startswith('/robot/delivery/') and parsed_url.path.endswith('/seller/placed'):
                self.handle_pickup_placed(parsed_url)
                return

            # 구매자 수령 완료 및 원점 복귀 처리 (/robot/delivery/{deliveryId}/buyer/orig_pos)
            if parsed_url.path.startswith('/robot/delivery/') and parsed_url.path.endswith('/buyer/orig_pos'):
                self.handle_buyer_orig_pos(parsed_url)
                return

            self.send_error_response(404, "Not Found", "Endpoint not found")
        except Exception as e:
            self.robot_node.get_logger().error(f'Error processing POST request: {str(e)}')
            self.send_error_response(500, "Internal Server Error", str(e))

    # 판매자 호수 도착 -> 웹에 픽업 도착 알림 POST 요청
    def send_pickup_arrived_notification(self, delivery_id: str) -> bool:
        try:
            # url = f"http://localhost:8080/robot/delivery/{delivery_id}/seller/arrived"
            url = f"https://j13a501.p.ssafy.io/api/robot/delivery/{delivery_id}/seller/arrived"

            response = requests.post(url, timeout=10)

            if response.status_code == 200:
                response_data = response.json()
                timestamp = response_data.get('timestamp')
                self.robot_node.get_logger().info(f"✅ 픽업 확인 전송 완료 - Timestamp: {timestamp}")
                return True
            else:
                self.robot_node.get_logger().error(f"❌ 픽업 확인 전송 실패: HTTP {response.status_code}")
                return False

        except requests.exceptions.RequestException as e:
            self.robot_node.get_logger().error(f"❌ 픽업 확인 요청 오류: {str(e)}")
            return False
        except Exception as e:
            self.robot_node.get_logger().error(f"❌ 픽업 확인 처리 오류: {str(e)}")
            return False

    # 구매자 집 도착 -> 웹에 구매자 도착 알림 POST 요청
    def send_buyer_arrived_notification(self, delivery_id: str) -> bool:
        try:
            # url = f"http://localhost:8080/robot/delivery/{delivery_id}/buyer/arrived"
            url = f"https://j13a501.p.ssafy.io/api/robot/delivery/{delivery_id}/buyer/arrived"
            
            payload = {
                "timestamp": datetime.now(timezone.utc).isoformat()
            }

            self.robot_node.get_logger().info(f"🏠 구매자 집 도착 알림 전송: {url}")

            response = requests.post(url, json=payload, headers={'Content-Type': 'application/json'}, timeout=10)

            if response.status_code == 200:
                response_data = response.json()
                timestamp = response_data.get('timestamp')
                self.robot_node.get_logger().info(f"✅ 구매자 집 도착 알림 전송 완료 - Timestamp: {timestamp}")
                return True
            else:
                self.robot_node.get_logger().error(f"❌ 구매자 집 도착 알림 전송 실패: HTTP {response.status_code}")
                return False

        except requests.exceptions.RequestException as e:
            self.robot_node.get_logger().error(f"❌ 구매자 집 도착 알림 요청 오류: {str(e)}")
            return False
        except Exception as e:
            self.robot_node.get_logger().error(f"❌ 구매자 집 도착 알림 처리 오류: {str(e)}")
            return False

    # 배송 완료 -> 웹에 배송 완료 신호 POST 요청
    def send_delivery_complete_notification(self, delivery_id: str) -> bool:
        try:
            # url = f"http://localhost:8080/robot/delivery/{delivery_id}/buyer/orig_pos"
            url = f"https://j13a501.p.ssafy.io/api/robot/delivery/{delivery_id}/buyer/orig_pos"
            payload = {
                "timestamp": datetime.now(timezone.utc).isoformat()
            }

            self.robot_node.get_logger().info(f"📦 배송 완료 신호 전송: {url}")

            response = requests.post(url, json=payload, headers={'Content-Type': 'application/json'}, timeout=10)

            if response.status_code == 200:
                response_data = response.json()
                timestamp = response_data.get('timestamp')
                self.robot_node.get_logger().info(f"✅ 배송 완료 신호 전송 완료 - Timestamp: {timestamp}")
                return True
            else:
                self.robot_node.get_logger().error(f"❌ 배송 완료 신호 전송 실패: HTTP {response.status_code}")
                return False

        except requests.exceptions.RequestException as e:
            self.robot_node.get_logger().error(f"❌ 배송 완료 신호 요청 오류: {str(e)}")
            return False
        except Exception as e:
            self.robot_node.get_logger().error(f"❌ 배송 완료 신호 처리 오류: {str(e)}")
            return False

    # 로봇 상태 요청 처리 - 항상 valid 응답
    def handle_robot_status(self, parsed_url):
        try:
            query_params = parse_qs(parsed_url.query)
            robot_id = query_params.get('robotId', [None])[0]
            delivery_id = query_params.get('delivery_id', [None])[0]

            if not robot_id:
                self.send_error_response(400, "Bad Request", "robotId parameter is required")
                return

            self.robot_node.get_logger().info(f'Received robot status request for ID: {robot_id}')
            if delivery_id:
                self.robot_node.get_logger().info(f'Delivery ID: {delivery_id}')
                # delivery_id를 로봇 노드에 저장 (나중에 사용하기 위해)
                if hasattr(self.robot_node, 'current_delivery_id'):
                    self.robot_node.current_delivery_id = delivery_id

            response_data = {
                "status": "valid",
                "message": "작업 가능",
                "timestamp": datetime.now(timezone.utc).isoformat()
            }

            self.send_json_response(200, response_data)
            self.robot_node.get_logger().info(f'Sent robot status response: {response_data}')

        except Exception as e:
            self.robot_node.get_logger().error(f'Error handling robot status request: {str(e)}')
            self.send_error_response(500, "Internal Server Error", str(e))

    # 배송 주소 요청 처리 및 즉시 배송 시작
    def handle_delivery_addresses(self, parsed_url):
        try:
            # URL path에서 delivery_id 추출 (예: /robot/delivery/1/addresses)
            path_parts = parsed_url.path.split('/')
            delivery_id = None
            for i, part in enumerate(path_parts):
                if part == 'delivery' and i + 1 < len(path_parts):
                    delivery_id = path_parts[i + 1]
                    break

            # URL 쿼리 파라미터에서 주소 데이터 읽기
            query_params = parse_qs(parsed_url.query)

            seller_address = query_params.get('sellerAddress', [None])[0]
            buyer_address = query_params.get('buyerAddress', [None])[0]

            # 판매자, 구매자 주소 받았는지 확인
            if not seller_address or not buyer_address:
                self.send_error_response(400, "Bad Request", "sellerAddress and buyerAddress parameters are required")
                return

            self.robot_node.get_logger().info(f'Received addresses - Seller: {seller_address}, Buyer: {buyer_address}, DeliveryId: {delivery_id}')

            # 주소 파싱
            seller_data = self.parse_address(seller_address)
            buyer_data = self.parse_address(buyer_address)
            if seller_data and buyer_data:
                self.robot_node.get_logger().info(f'Parsed seller: {seller_data[0]}동 {seller_data[1]}층 {seller_data[2]}호')
                self.robot_node.get_logger().info(f'Parsed buyer: {buyer_data[0]}동 {buyer_data[1]}층 {buyer_data[2]}호')

                # 저장된 delivery_id 사용 (로봇 상태 확인에서 받은 값)
                stored_delivery_id = getattr(self.robot_node, 'current_delivery_id', delivery_id)
                final_delivery_id = stored_delivery_id if stored_delivery_id else delivery_id

                self.robot_node.get_logger().info(f'🔍 delivery_id 선택: URL파라미터={delivery_id}, 저장된값={stored_delivery_id}, 최종사용={final_delivery_id}')

                # 별도 스레드에서 배송 시작
                delivery_thread = threading.Thread(
                    target=self.start_delivery,
                    args=(seller_data, buyer_data, final_delivery_id),
                    daemon=True
                )
                delivery_thread.start()

                # 응답
                response_data = {
                    "status": "success",
                    "message": "배송 시작됨",
                    "seller": seller_address,
                    "buyer": buyer_address,
                    "timestamp": datetime.now(timezone.utc).isoformat()
                }
                self.send_json_response(200, response_data)
            else:
                self.send_error_response(400, "Bad Request", "Invalid address format")

        except Exception as e:
            self.robot_node.get_logger().error(f'Error handling delivery addresses: {str(e)}')
            self.send_error_response(500, "Internal Server Error", str(e))

    # 주소 문자열을 파싱하여 (동, 층, 호수)를 로봇 배송 로직에 반환
    def parse_address(self, address_str):
        try:
            # "1동 301호" 형태 파싱
            match = re.match(r'(\d+)동 (\d+)호', address_str.strip())
            if not match:
                self.robot_node.get_logger().error(f'Address parsing failed: {address_str}')
                return None

            dong = int(match.group(1))  # 동 번호
            room_full = match.group(2)  # 전체 호수 (예: 102)

            # 호수에서 층수와 호수 추출
            if len(room_full) == 3:
                # 3자리: 301호 -> 3층 1호
                floor = int(room_full[0])      # 첫 번째 자리 = 층
                room = int(room_full[1:])      # 나머지 = 호수
            elif len(room_full) == 2:
                # 2자리: 102호 -> 1층 2호 (첫 자리는 층이 아니라 01, 02 형태)
                floor = 1                      # 1층으로 고정
                room = int(room_full)          # 전체가 호수
            else:
                # 1자리: 2호 -> 1층 2호
                floor = 1
                room = int(room_full)

            return (dong, floor, room)

        except Exception as e:
            self.robot_node.get_logger().error(f'Address parsing error: {str(e)}')
            return None

    # 픽업 완료 알림 처리 (/robot/delivery/{deliveryId}/seller/placed)
    def handle_pickup_placed(self, parsed_url):
        try:
            # URL에서 delivery_id 추출
            path_parts = parsed_url.path.split('/')
            delivery_id = path_parts[3] if len(path_parts) > 3 else None

            if not delivery_id:
                self.send_error_response(400, "Bad Request", "delivery_id not found in URL")
                return

            self.robot_node.get_logger().info(f'📦 픽업 완료 신호 수신 - DeliveryId: {delivery_id}')

            # 현재 픽업 대기 상태인지 확인
            if (hasattr(self.robot_node, 'delivery_state') and
                self.robot_node.delivery_state == "WAITING_PICKUP" and
                hasattr(self.robot_node, 'current_delivery_context') and
                self.robot_node.current_delivery_context and
                self.robot_node.current_delivery_context.get('delivery_id') == delivery_id):

                self.robot_node.get_logger().info(f'✅ 픽업 완료 - 배송 재개 중... DeliveryId: {delivery_id}')
                
                # --- 코드 추가 시작 ---
                # 설명: 판매자가 물건을 넣었음을 시뮬레이션하기 위해 상자를 생성합니다.
                self.robot_node.get_logger().info("📦 사물함에 상자를 생성합니다.")
                self.robot_node.control_box('spawn')
                # --- 코드 추가 끝 ---

                # 배송 상태를 배송 중으로 변경
                self.robot_node.delivery_state = "DELIVERING"

                # 저장된 컨텍스트에서 배송 정보 추출
                ctx = self.robot_node.current_delivery_context
                nav_s = ctx['nav_s']
                A_s, F_s, R_s = ctx['A_s'], ctx['F_s'], ctx['R_s']
                A_b, F_b, R_b = ctx['A_b'], ctx['F_b'], ctx['R_b']
                ns_seller, ns_buyer = ctx['ns_seller'], ctx['ns_buyer']

                # 응답 먼저 전송
                response_data = {
                    "timestamp": datetime.now(timezone.utc).isoformat()
                }
                self.send_json_response(200, response_data)

                # 별도 스레드에서 배송 계속 진행
                import threading
                delivery_thread = threading.Thread(
                    target=self.continue_delivery_from_pickup,
                    args=(nav_s, A_s, F_s, R_s, A_b, F_b, R_b, ns_seller, ns_buyer, delivery_id),
                    daemon=True
                )
                delivery_thread.start()

            else:
                self.robot_node.get_logger().warn(f'⚠️ 픽업 대기 상태가 아니거나 다른 delivery_id: {delivery_id}')
                self.send_error_response(400, "Bad Request", "Not waiting for pickup or wrong delivery_id")

        except Exception as e:
            self.robot_node.get_logger().error(f'픽업 완료 처리 오류: {str(e)}')
            self.send_error_response(500, "Internal Server Error", str(e))

    # 구매자 수령 완료 및 원점 복귀 처리
    def handle_buyer_orig_pos(self, parsed_url):
        try:
            # URL에서 delivery_id 추출
            path_parts = parsed_url.path.split('/')
            delivery_id = None
            for i, part in enumerate(path_parts):
                if part == 'delivery' and i + 1 < len(path_parts):
                    delivery_id = path_parts[i + 1]
                    break

            if not delivery_id:
                self.send_error_response(400, "Bad Request", "delivery_id not found in URL")
                return

            self.robot_node.get_logger().info(f'📦 구매자 수령 완료 신호 수신 - DeliveryId: {delivery_id}')

            # 현재 구매자 픽업 대기 상태인지 확인
            if (hasattr(self.robot_node, 'delivery_state') and
                self.robot_node.delivery_state == "WAITING_BUYER_PICKUP" and
                hasattr(self.robot_node, 'current_delivery_context') and
                self.robot_node.current_delivery_context and
                self.robot_node.current_delivery_context.get('delivery_id') == delivery_id):

                self.robot_node.get_logger().info(f'✅ 구매자 수령 완료 확인 - 배송 완료 처리 중... DeliveryId: {delivery_id}')
                
                # --- 코드 추가 시작 ---
                # 설명: 구매자가 물건을 꺼냈음을 시뮬레이션하기 위해 상자를 삭제합니다.
                self.robot_node.get_logger().info("📦 사물함에서 상자를 삭제합니다.")
                self.robot_node.control_box('delete')

                # --- 코드 추가 시작 ---
                # 설명: 구매자가 물건을 꺼냈으므로, 복귀 전에 문을 닫습니다.
                self.robot_node.get_logger().info("🚪 구매자 수령 완료. 사물함 문을 닫습니다.")
                self.robot_node.control_storage_door('close')
                # --- 코드 추가 끝 ---


                # 응답 먼저 전송
                response_data = {
                    "timestamp": datetime.now(timezone.utc).isoformat()
                }
                self.send_json_response(200, response_data)

                # 배송 완료 처리
                nav_s = self.robot_node.current_delivery_context['nav_s']

                # 배송 상태를 완료로 변경
                nav_s.delivery_state = "DELIVERY_COMPLETED"

                # 배송 완료 신호는 이미 웹에서 받았으므로 별도 전송 불필요
                nav_s.get_logger().info("✅ 배송 완료 처리 완료 (웹에서 이미 신호 수신)")

                # 배송 완료 로그
                nav_s.get_logger().info("✅ 구매자 물건 수령 완료")
                nav_s.get_logger().info("✅ 물건 배송 완료")
                nav_s.get_logger().info("✅✅✅ 판매자→구매자 배송 플로우 완료 ✅✅✅")

                # 배송 완료 후 구매자 아파트 1층으로 대기 위치 이동
                nav_s.get_logger().info("🏠 배송 완료 - 구매자 아파트 1층으로 대기 위치 이동 시작")

                # 현재 구매자 아파트 정보 가져오기
                current_elevator_ns = nav_s.elevator_ns
                if current_elevator_ns in nav_s.apartments:
                    apt_info = nav_s.apartments[current_elevator_ns]

                    # 현재 층이 1층이 아니면 1층으로 이동
                    if nav_s.current_floor != 1:
                        nav_s.get_logger().info(f"현재 {nav_s.current_floor}층 → 1층으로 엘리베이터 이동")
                        if nav_s.call_elevator_flow(1):
                            nav_s.get_logger().info("✅ 1층 이동 완료 - 대기 위치 준비")
                        else:
                            nav_s.get_logger().warn("⚠️ 1층 이동 실패")
                    else:
                        nav_s.get_logger().info("✅ 이미 1층 위치 - 아파트 앞 대기 위치로 이동")

                    # 아파트 앞 대기 위치로 이동
                    if nav_s._navigate_to_pose_blocking(apt_info["x"], apt_info["y"], apt_info["ori"], context="배송 완료 후 대기 위치"):
                        nav_s.get_logger().info("✅ 구매자 아파트 앞 대기 위치 도착 - 다음 배송 대기")
                    else:
                        nav_s.get_logger().warn("⚠️ 대기 위치 이동 실패")
                else:
                    nav_s.get_logger().warn(f"⚠️ 알 수 없는 엘리베이터 네임스페이스: {current_elevator_ns}")

                # 배송 상태를 IDLE로 되돌림
                nav_s.delivery_state = "IDLE"
                nav_s.current_delivery_context = None
                nav_s.get_logger().info("🤖 로봇 상태: IDLE - 새로운 배송 요청 대기 중")

            else:
                self.robot_node.get_logger().warn(f'⚠️ 구매자 픽업 대기 상태가 아니거나 다른 delivery_id: {delivery_id}')
                self.send_error_response(400, "Bad Request", "Not waiting for buyer pickup or wrong delivery_id")

        except Exception as e:
            self.robot_node.get_logger().error(f'구매자 수령 완료 처리 오류: {str(e)}')
            self.send_error_response(500, "Internal Server Error", str(e))

    # 픽업 완료 후 배송 계속 진행
    def continue_delivery_from_pickup(self, nav_s, A_s, F_s, R_s, A_b, F_b, R_b, ns_seller, ns_buyer, delivery_id):
        try:
        
            # --- 코드 추가 시작 ---
            # 설명: 판매자가 물건을 넣었으므로, 이동하기 전에 문을 닫습니다.
            nav_s.get_logger().info("🚪 픽업 완료. 사물함 문을 닫습니다.")
            nav_s.control_storage_door('close')
            # --- 코드 추가 끝 ---

            # 사물함 문이 완전히 닫힐 때까지 추가 대기
            import time
            nav_s.get_logger().info("⏳ 사물함 문 완전 닫힘 대기...")
            time.sleep(2.0)  # 추가 2초 대기
            nav_s.get_logger().info("✅ 사물함 문 닫힘 완료")

            nav_s.get_logger().info("🚩 물건 픽업 완료, 구매자에게 배송 시작")
            nav_s.get_logger().info(f"🔍 디버그 - A_s={A_s}, A_b={A_b}, ns_seller={ns_seller}, ns_buyer={ns_buyer}")

            # 판매자 아파트 정보
            nav_s.get_logger().info(f"🔍 판매자 아파트 정보 조회 중: ns_seller={ns_seller}")
            apt_s = nav_s.apartments[ns_seller]
            nav_s.get_logger().info(f"✅ 판매자 아파트 정보 조회 완료: {apt_s}")

            # 구매자 정보
            nav_s.get_logger().info(f"🔍 구매자 아파트 정보 조회 중: ns_buyer={ns_buyer}")
            apt_b = nav_s.apartments[ns_buyer]
            nav_s.get_logger().info(f"✅ 구매자 아파트 정보 조회 완료: {apt_b}")

            # [5] 같은 아파트, 같은 층인지 확인
            nav_s.get_logger().info(f"🔍 [5] 아파트/층 비교 - A_s={A_s}, A_b={A_b}, F_s={F_s}, F_b={F_b}")
            same_apartment = (A_s == A_b)  # 같은 아파트
            same_floor = (F_s == F_b)      # 같은 층
            nav_s.get_logger().info(f"🔍 [5] 비교 결과 - same_apartment={same_apartment}, same_floor={same_floor}")

            if same_apartment and same_floor:
                # [5-A] 같은 아파트, 같은 층 → 바로 구매자 호수로 이동
                nav_s.get_logger().info(f"[5] ✅ 같은 아파트({A_s}동), 같은 층({F_s}층) - 엘리베이터 생략하고 바로 이동")
            else:
                # [5-B] 다른 아파트거나 다른 층 → 기존 로직대로 1층으로 이동
                nav_s.get_logger().info("[5-B] 다른 아파트거나 다른 층 - 1층으로 이동 시작")
                nav_s.get_logger().info(f"🔍 판매자 엘베 앞으로 이동: apt_s={apt_s}")
                nav_s.get_logger().info("🚀 네비게이션 시작...")
                navigation_result = nav_s._navigate_to_pose_blocking(apt_s["x"], apt_s["y"], apt_s["ori"], context="판매자 엘베 앞(복귀)")
                nav_s.get_logger().info(f"📍 네비게이션 결과: {navigation_result}")
                if not navigation_result:
                    nav_s.get_logger().error("[5-B] 판매자 엘베 앞 이동 실패")
                    return
                nav_s.get_logger().info("[5] ✅ 판매자 엘베 앞(복귀) 도착")

                # [6] 1층으로 이동
                if nav_s.current_floor == 1:
                    nav_s.get_logger().info(f"[6] ✅ 이미 1층 위치 - 엘리베이터 생략")
                else:
                    if not nav_s.call_elevator_flow(1):
                        nav_s.get_logger().error("1층 이동 실패")
                        return
                    nav_s.get_logger().info("[6] ✅ 1층 이동 완료")

            # 구매자 배송을 위해 기존 노드(nav_s)의 엘리베이터 네임스페이스 변경 (다른 아파트인 경우만)
            if not same_apartment:
                nav_s.elevator_ns = ns_buyer

                # 구매자용 엘리베이터 퍼블리셔/구독자로 업데이트
                base = f'/{nav_s.elevator_ns}/robot'
                nav_s.current_location_pub = nav_s.create_publisher(Int32, f'{base}/current_location', 10)
                nav_s.target_floor_pub = nav_s.create_publisher(Int32, f'{base}/target_floor', 10)
                nav_s.boarded_pub = nav_s.create_publisher(Bool, f'{base}/boarded', 10)
                nav_s.alighted_pub = nav_s.create_publisher(Bool, f'{base}/alighted', 10)

                # 기존 구독자를 새로 생성 (destroy() 대신 덮어쓰기로 메모리 누수 방지)
                nav_s.state_sub = nav_s.create_subscription(
                    String, f'/{nav_s.elevator_ns}/status/state', nav_s._on_state, 10)
                nav_s.arrived_sub = nav_s.create_subscription(
                    Int32, f'/{nav_s.elevator_ns}/status/arrived', nav_s._on_arrived, 10)

            if same_apartment and same_floor:
                # [7] 같은 아파트, 같은 층 → 바로 구매자 호수로 이동 (아파트 앞 이동 생략)
                nav_s.get_logger().info("[7] ✅ 같은 아파트, 같은 층 - 구매자 아파트 앞 이동 생략")
                nav_s.get_logger().info("[8] ✅ 같은 층이므로 엘리베이터 생략")
            else:
                # [7] 구매자 동 앞으로 이동
                if not nav_s._navigate_to_pose_blocking(apt_b["x"], apt_b["y"], apt_b["ori"], context="구매자 아파트 앞"):
                    return
                nav_s.get_logger().info("[7] ✅ 구매자 아파트 앞 도착")

                # [8] 구매자 층으로 이동 (현재 층과 목적층이 동일하면 엘베 생략)
                if nav_s.current_floor == F_b:
                    nav_s.get_logger().info(f"[8] ✅ 이미 {F_b}층 위치 - 엘리베이터 생략")
                else:
                    if not nav_s.call_elevator_flow(F_b):
                        nav_s.get_logger().error("구매자 층 이동 실패")
                        return
                    nav_s.get_logger().info("[8] ✅ 구매자 층 이동 완료")

            # [9] 구매자 호수 앞 도착
            room_b = nav_s.rooms[ns_buyer].get(R_b)
            if room_b is None:
                nav_s.get_logger().error(f"잘못된 구매자 호수: {R_b}")
                return
            # 엘리베이터 탑승 좌표의 방향을 사용 (문을 바라보는 방향)
            boarding_ori_buyer = nav_s.boarding_poses[ns_buyer]["ori"]
            if not nav_s._navigate_to_pose_blocking(room_b["x"], room_b["y"], boarding_ori_buyer, context="구매자 호수"):
                return
            nav_s.get_logger().info("[9] ✅ 구매자 호수 도착")

            # --- 코드 추가 시작 ---
            # 설명: 구매자가 물건을 꺼낼 수 있도록 사물함 문을 엽니다.
            nav_s.get_logger().info("🚪 구매자 수령을 위해 사물함 문을 엽니다.")
            nav_s.control_storage_door('open')
            # --- 코드 추가 끝 ---

            # [10] 🏠 구매자 집 도착 알림을 웹에 POST 요청으로 전송
            if delivery_id:
                nav_s.get_logger().info(f"[10] 구매자 집 도착 알림 전송 중... (DeliveryId: {delivery_id})")
                if self.send_buyer_arrived_notification(delivery_id):
                    nav_s.get_logger().info("[10] ✅ 구매자 집 도착 알림 전송 성공")
                    nav_s.get_logger().info("[10] 📱 구매자에게 FCM 알림이 발송되었습니다.")

                    # 구매자 수령 대기 상태로 전환 (self.robot_node에 저장)
                    self.robot_node.delivery_state = "WAITING_BUYER_PICKUP"
                    nav_s.get_logger().info("[10] 🔄 구매자 수령 대기 상태로 전환 - 웹에서 수령 완료 신호 대기 중...")
                    nav_s.get_logger().info("[10] ⏳ 구매자가 물건을 수령할 때까지 대기합니다...")

                    # 배송 컨텍스트 저장 (구매자 수령 완료 신호를 위해, self.robot_node에 저장)
                    self.robot_node.current_delivery_context = {
                        'delivery_id': delivery_id,
                        'nav_s': nav_s,
                        'buyer_pickup_stage': True
                    }

                    nav_s.get_logger().info("[10] 📱 구매자 픽업 모달이 표시됩니다. 수령 완료 버튼을 클릭하면 배송이 완료됩니다.")
                else:
                    nav_s.get_logger().warn("[10] ⚠️ 구매자 집 도착 알림 전송 실패")
            else:
                nav_s.get_logger().warn("[10] ⚠️ delivery_id 없음 - 구매자 집 도착 알림 전송 건너뜀")

            # 배송 완료는 웹에서 구매자 수령 완료 신호를 받으면 처리됨
            # (handle_buyer_orig_pos 메서드에서 처리)

        except Exception as e:
            nav_s.get_logger().error(f"픽업 완료 후 배송 진행 중 오류: {str(e)}")

    # 🚚 배송 시작
    def start_delivery(self, seller_data, buyer_data, delivery_id):
        try:
            A_s, F_s, R_s = seller_data
            A_b, F_b, R_b = buyer_data

            self.robot_node.get_logger().info("🚀 배송 프로세스 시작")
            self.robot_node.get_logger().info(f"판매자: {A_s}동 {F_s}층 {R_s}호 → 구매자: {A_b}동 {F_b}층 {R_b}호")
            self.robot_node.get_logger().info(f"DeliveryId: {delivery_id}")

            # 배송 로직 직접 호출
            self.execute_delivery(A_s, F_s, R_s, A_b, F_b, R_b, delivery_id)

        except Exception as e:
            self.robot_node.get_logger().error(f'Delivery process error: {str(e)}')

    # 실제 배송 로직
    def execute_delivery(self, A_s, F_s, R_s, A_b, F_b, R_b, delivery_id):
        """실제 배송 로직 실행"""

        elevator_ns_map = {
            1: "elevator",     # 아파트 1
            2: "elevator_0",   # 아파트 2
            3: "elevator_1",   # 아파트 3
            4: "elevator_2",   # 아파트 4
        }

        ns_seller = elevator_ns_map.get(A_s)
        ns_buyer = elevator_ns_map.get(A_b)
        if ns_seller is None or ns_buyer is None:
            self.robot_node.get_logger().error(f"❌ 지원하지 않는 아파트 번호: {A_s if ns_seller is None else A_b}")
            return

        # 새로운 QuickRoomNav 인스턴스 생성하지 않고 기존 로직 유지
        # generator 에러를 방지하기 위해 try-except 사용
        try:
            # 임시로 기존 노드 사용 (속성이 없을 수 있음)
            nav_s = self.robot_node
            # delivery_id 속성이 없으면 생성하고, 있으면 업데이트
            if not hasattr(nav_s, 'delivery_id'):
                nav_s.delivery_id = None
            nav_s.delivery_id = delivery_id

            self.robot_node.get_logger().info("🚩 물건 픽업 시작 (기존 노드 사용)")

        except Exception as e:
            self.robot_node.get_logger().error(f"노드 설정 오류: {str(e)}")
            return

        nav_s.get_logger().info("🚩 물건 픽업 시작")
        nav_s.get_logger().info(f" 판매자: 아파트 {A_s}, {F_s}층, 호수 {R_s} → 구매자: 아파트 {A_b}, {F_b}층, 호수 {R_b}")
        if delivery_id:
            nav_s.get_logger().info(f" DeliveryId: {delivery_id}")

        try:
            # [1] 판매자 동 앞으로 이동
            apt_s = nav_s.apartments[ns_seller]
            if not nav_s._navigate_to_pose_blocking(apt_s["x"], apt_s["y"], apt_s["ori"], context="판매자 아파트 앞"):
                return
            nav_s.get_logger().info("[1] ✅ 판매자 아파트 앞 도착")

            # [2] 판매자 층까지 이동 (현재 층과 목적층이 동일하면 엘베 생략)
            if nav_s.current_floor == F_s:
                nav_s.get_logger().info(f"[2] ✅ 이미 {F_s}층 위치 - 엘리베이터 생략")
            else:
                if not nav_s.call_elevator_flow(F_s):
                    nav_s.get_logger().error("판매자 층 이동 실패")
                    return
                nav_s.get_logger().info("[2] ✅ 판매자 층 이동 완료")

            # [3] 판매자 호수 앞 도착
            room_s = nav_s.rooms[ns_seller].get(R_s)
            if room_s is None:
                nav_s.get_logger().error(f"잘못된 판매자 호수: {R_s}")
                return
            # 엘리베이터 탑승 좌표의 방향을 사용 (문을 바라보는 방향)
            boarding_ori = nav_s.boarding_poses[ns_seller]["ori"]
            if not nav_s._navigate_to_pose_blocking(room_s["x"], room_s["y"], boarding_ori, context="판매자 호수"):
                return
            nav_s.get_logger().info("[3] ✅ 판매자 호수 도착")
            
            
            # --- 코드 추가 시작 ---
            # 설명: 판매자가 물건을 넣을 수 있도록 사물함 문을 엽니다.
            nav_s.get_logger().info("🚪 판매자 픽업을 위해 사물함 문을 엽니다.")
            nav_s.control_storage_door('open')
            # --- 코드 추가 끝 ---
            
            
            # [4] 웹에 픽업 도착 알림 POST 요청 전송 및 대기 상태로 전환
            # delivery_id 값 확인 및 로깅
            nav_s.get_logger().info(f"[4] 🔍 delivery_id 체크: 파라미터={delivery_id}, nav_s.delivery_id={getattr(nav_s, 'delivery_id', 'None')}")

            if delivery_id:
                nav_s.get_logger().info(f"[4] 픽업 도착 알림 전송 중... (DeliveryId: {delivery_id})")
                if self.send_pickup_arrived_notification(delivery_id):
                    nav_s.get_logger().info("[4] ✅ 픽업 도착 알림 전송 성공")

                    # 픽업 대기 상태로 전환 - 배송 컨텍스트 저장
                    nav_s.delivery_state = "WAITING_PICKUP"
                    nav_s.current_delivery_context = {
                        'nav_s': nav_s,
                        'A_s': A_s, 'F_s': F_s, 'R_s': R_s,
                        'A_b': A_b, 'F_b': F_b, 'R_b': R_b,
                        'delivery_id': delivery_id,
                        'ns_seller': ns_seller,
                        'ns_buyer': ns_buyer
                    }

                    nav_s.get_logger().info("[4] 🔄 픽업 대기 상태로 전환 - 웹에서 픽업 완료 신호 대기 중...")
                    nav_s.get_logger().info("[4] 📱 판매자에게 FCM 알림이 발송되었습니다. 물건을 넣어주세요.")

                    # 여기서 함수 종료 - 픽업 완료 POST 요청이 오면 continue_delivery_after_pickup() 호출됨
                    return

                else:
                    nav_s.get_logger().warn("[4] ⚠️ 픽업 도착 알림 전송 실패 - 기본 대기로 진행")
            else:
                nav_s.get_logger().warn("[4] ⚠️ delivery_id 없음 - 픽업 도착 알림 전송 건너뜀")

            # delivery_id가 없거나 알림 전송 실패 시 기존 방식대로 3초 대기
            import time
            time.sleep(3.0)
            nav_s.get_logger().info("[4] ✅ 픽업 완료 (시뮬레이션)")

            # delivery_id 없거나 알림 전송 실패 시에만 여기 도달 - 기본 시뮬레이션으로 계속 진행

            nav_s.get_logger().info("🚩 물건 픽업 완료, 구매자에게 배송 시작")

            # [5] 엘베 앞으로 복귀
            if not nav_s._navigate_to_pose_blocking(apt_s["x"], apt_s["y"], apt_s["ori"], context="판매자 엘베 앞(복귀)"):
                return
            nav_s.get_logger().info("[5] ✅ 판매자 엘베 앞(복귀) 도착")

            # [6] 1층으로 이동
            if not nav_s.call_elevator_flow(1):
                nav_s.get_logger().error("1층 이동 실패")
                return
            nav_s.get_logger().info("[6] ✅ 1층 이동 완료")

            # 구매자 배송을 위해 기존 노드(nav_s)의 엘리베이터 네임스페이스 변경
            nav_s.elevator_ns = ns_buyer

            # 구매자용 엘리베이터 퍼블리셔/구독자로 업데이트
            base = f'/{nav_s.elevator_ns}/robot'
            nav_s.current_location_pub = nav_s.create_publisher(Int32, f'{base}/current_location', 10)
            nav_s.target_floor_pub = nav_s.create_publisher(Int32, f'{base}/target_floor', 10)
            nav_s.boarded_pub = nav_s.create_publisher(Bool, f'{base}/boarded', 10)
            nav_s.alighted_pub = nav_s.create_publisher(Bool, f'{base}/alighted', 10)

            # 기존 구독자를 새로 생성 (destroy() 대신 덮어쓰기로 메모리 누수 방지)
            nav_s.state_sub = nav_s.create_subscription(
                String, f'/{nav_s.elevator_ns}/status/state', nav_s._on_state, 10)
            nav_s.arrived_sub = nav_s.create_subscription(
                Int32, f'/{nav_s.elevator_ns}/status/arrived', nav_s._on_arrived, 10)

            # [7] 구매자 동 앞으로 이동
            apt_b = nav_s.apartments[ns_buyer]
            if not nav_s._navigate_to_pose_blocking(apt_b["x"], apt_b["y"], apt_b["ori"], context="구매자 아파트 앞"):
                return
            nav_s.get_logger().info("[7] ✅ 구매자 아파트 앞 도착")

            # [8] 구매자 층으로 이동 (현재 층과 목적층이 동일하면 엘베 생략)
            if nav_s.current_floor == F_b:
                nav_s.get_logger().info(f"[8] ✅ 이미 {F_b}층 위치 - 엘리베이터 생략")
            else:
                if not nav_s.call_elevator_flow(F_b):
                    nav_s.get_logger().error("구매자 층 이동 실패")
                    return
                nav_s.get_logger().info("[8] ✅ 구매자 층 이동 완료")

            # [9] 구매자 호수 앞 도착
            room_b = nav_s.rooms[ns_buyer].get(R_b)
            if room_b is None:
                nav_s.get_logger().error(f"잘못된 구매자 호수: {R_b}")
                return
            # 엘리베이터 탑승 좌표의 방향을 사용 (문을 바라보는 방향)
            boarding_ori_buyer = nav_s.boarding_poses[ns_buyer]["ori"]
            if not nav_s._navigate_to_pose_blocking(room_b["x"], room_b["y"], boarding_ori_buyer, context="구매자 호수"):
                return
            nav_s.get_logger().info("[9] ✅ 구매자 호수 도착")

            # [10] 🏠 구매자 집 도착 알림을 웹에 POST 요청으로 전송
            if delivery_id:
                nav_s.get_logger().info(f"[10] 구매자 집 도착 알림 전송 중... (DeliveryId: {delivery_id})")
                if self.send_buyer_arrived_notification(delivery_id):
                    nav_s.get_logger().info("[10] ✅ 구매자 집 도착 알림 전송 성공")
                    nav_s.get_logger().info("[10] 📱 구매자에게 FCM 알림이 발송되었습니다.")

                    # TODO: 여기서 구매자 수령 완료 POST 요청을 기다려야 함
                    # 현재는 시뮬레이션으로 5초 대기 후 배송 완료 처리
                    import time
                    time.sleep(5.0)  # 구매자가 물건을 가져갈 시간
                    nav_s.get_logger().info("[10] ✅ 구매자 수령 완료 (시뮬레이션)")

                    # 배송 완료 신호를 웹에 전송
                    if self.send_delivery_complete_notification(delivery_id):
                        nav_s.get_logger().info("[10] ✅ 배송 완료 신호 전송 성공")
                    else:
                        nav_s.get_logger().warn("[10] ⚠️ 배송 완료 신호 전송 실패")
                else:
                    nav_s.get_logger().warn("[10] ⚠️ 구매자 집 도착 알림 전송 실패")
            else:
                nav_s.get_logger().warn("[10] ⚠️ delivery_id 없음 - 구매자 집 도착 알림 전송 건너뜀")

            # [11] 물건 전달 완료
            nav_s.get_logger().info("✅ 물건 배송 완료")
            nav_s.get_logger().info("✅✅✅ 판매자→구매자 배송 플로우 완료 ✅✅✅")

        except Exception as e:
            nav_s.get_logger().error(f"배송 중 오류 발생: {str(e)}")
        # finally 블록 제거 - 메인 노드는 파괴하지 않음

    # JSON 응답 전송 함수
    def send_json_response(self, status_code, data):
        response_json = json.dumps(data, ensure_ascii=False, indent=2)

        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Access-Control-Allow-Origin', '*')
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

    def log_message(self, format, *args):
        """HTTP 서버 로그를 ROS 로거로 리다이렉트"""
        if self.robot_node:
            self.robot_node.get_logger().info(f'HTTP: {format % args}')

# HTTP 서버 및 로봇 제어 통합 클래스
class RobotDeliveryServer(QuickRoomNav):
    def __init__(self):
        # QuickRoomNav 초기화 (기본 엘리베이터 네임스페이스 사용)
        super().__init__('elevator')

        self.server_port = 8888
        self.server_host = '0.0.0.0'
        self.current_delivery_id = None  # 현재 배송 ID 저장

        def handler_factory(*args, **kwargs):
            return RobotStatusHandler(*args, robot_node=self, **kwargs)

        self.http_server = HTTPServer((self.server_host, self.server_port), handler_factory)

        self.server_thread = threading.Thread(target=self.run_http_server, daemon=True)
        self.server_thread.start()

        self.get_logger().info(f'🚀 Robot Delivery Server started on {self.server_host}:{self.server_port}')
        self.get_logger().info(f'📡 Waiting for delivery requests...')
        self.get_logger().info(f'✅ Status endpoint: http://{self.server_host}:{self.server_port}/robot/status?robotId=<id>')

        # 사물함 문 제어를 위한 퍼블리셔 추가
        self.door_control_pub = self.create_publisher(String, '/door_control', 10)      
        
        # --- 코드 추가 시작 ---
        # 설명: 상자 생성/삭제를 제어할 '/box_control' 토픽용 퍼블리셔를 생성합니다.
        self.box_control_pub = self.create_publisher(String, '/box_control', 10)
        # --- 코드 추가 끝 ---  
        
    # HTTP 서버 실행 함수
    def run_http_server(self):
        try:
            self.get_logger().info('HTTP server thread started')
            self.http_server.serve_forever()
        except Exception as e:
            self.get_logger().error(f'HTTP server error: {str(e)}')

    # 서버 종료 함수
    def shutdown_server(self):
        if self.http_server:
            self.get_logger().info('Shutting down HTTP server')
            self.http_server.shutdown()
            self.http_server.server_close()
            
    # --- 코드 추가 시작 ---
    # 설명: 'open' 또는 'close' 명령을 받아 문 제어 토픽을 발행하는 함수입니다.
    def control_storage_door(self, command: str):
        """ 사물함 문을 열거나 닫는 ROS 토픽을 발행합니다. """
        msg = String()
        msg.data = command
        self.door_control_pub.publish(msg)
        self.get_logger().info(f'🚪 Storage door command published: {command}')

        # 문 동작 완료까지 충분한 대기 시간
        import time
        if command == 'open':
            self.get_logger().info("⏳ 사물함 문 열림 대기...")
            time.sleep(5.0)  # 문 열림 완료까지 5초 대기
            self.get_logger().info("✅ 사물함 문 열림 완료")
        elif command == 'close':
            self.get_logger().info("⏳ 사물함 문 닫힘 대기...")
            time.sleep(5.0)  # 문 닫힘 완료까지 5초 대기
            self.get_logger().info("✅ 사물함 문 닫힘 완료")
        else:
            time.sleep(3.0)  # 기타 명령은 기본 3초 대기

    # 설명: 'spawn' 또는 'delete' 명령을 받아 상자 제어 토픽을 발행하는 함수입니다.
    def control_box(self, command: str):
        """ 사물함 안의 상자를 생성하거나 삭제합니다. """
        msg = String()
        msg.data = command
        self.box_control_pub.publish(msg)
        self.get_logger().info(f'📦 Box command published: {command}')
        time.sleep(1) # 명령이 처리될 시간을 확보합니다.
    # --- 코드 추가 끝 ---

    
    
    
def main():
    rclpy.init()

    # 인자 없이 실행 - HTTP 서버 모드
    if len(sys.argv) == 1:
        # HTTP 서버 모드
        server = RobotDeliveryServer()

        try:
            # spin 대신 spin_once를 사용하여 다른 스레드와의 충돌 방지
            while rclpy.ok():
                rclpy.spin_once(server, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            server.shutdown_server()
            server.destroy_node()
            rclpy.shutdown()
        return

    # 모든 데이터는 웹 API를 통해 받으므로 HTTP 서버 모드만 사용

if __name__ == '__main__':
    main()
