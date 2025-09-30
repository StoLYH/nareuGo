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

class QuickRoomNav(Node):
    def __init__(self, elevator_ns: str):
        super().__init__('quick_room_nav')
        self.elevator_ns = elevator_ns  # 예: 'elevator', 'elevator_0', 'elevator_1', 'elevator_2'

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 엘리베이터 통신 채널 설정
        self.setup_elevator_topics()

        # === 코드 추가: 사물함 제어용 퍼블리셔 ===
        self.door_control_pub = self.create_publisher(String, '/door_control', 10)
        self.box_control_pub = self.create_publisher(String, '/box_control', 10)
        # =====================================

        # 아파트 및 호수 좌표 정보
        self.apartments = {
            "elevator":   {"x": 6.43848,  "y": -4.28445, "ori": 4.71239},
            "elevator_0": {"x": 14.7429,"y": -4.21861, "ori": 4.71239},
            "elevator_1": {"x": 18.6988,"y":  1.64997, "ori": 0.0 },
            "elevator_2": {"x": 18.7798, "y":  8.10504,  "ori": 0.0 },
        }
        self.boarding_poses = {
            "elevator":   {"x": 6.42516,   "y": -6.03101, "ori": 1.57},
            "elevator_0": {"x": 14.7095, "y": -6.01259, "ori": 1.57},
            "elevator_1": {"x": 20.5827,  "y":  1.65911, "ori": 4.71 },
            "elevator_2": {"x": 20.7613,  "y":  8.12548, "ori": 4.71 },
        }
        self.rooms = {
            "elevator":   {1: {"x": 7.086, "y": -4.655}, 2: {"x": 6.474, "y": -4.625}},
            "elevator_0": {1: {"x": 15.1433, "y": -4.24452}, 2: {"x": 14.6848, "y": -4.24452}},
            "elevator_1": {1: {"x": 18.910, "y":  2.397}, 2: {"x": 18.862, "y":  0.982}},
            "elevator_2": {1: {"x": 19.145, "y":  8.933}, 2: {"x": 19.191, "y":  7.437}},
        }

        # 로봇 상태 변수
        self.current_floor = 1
        self.state_str = None
        self.arrived_floor = None
        self._last_pose = None
        self.entry_pose = None
        self._entry_pose_loaded = False
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)

    def setup_elevator_topics(self):
        """엘리베이터 네임스페이스에 따라 토픽을 설정하거나 재설정합니다."""
        self.get_logger().info(f"'{self.elevator_ns}' 엘리베이터 통신 채널을 설정합니다.")
        base = f'/{self.elevator_ns}/robot'
        self.current_location_pub = self.create_publisher(Int32, f'{base}/current_location', 10)
        self.target_floor_pub     = self.create_publisher(Int32, f'{base}/target_floor', 10)
        self.boarded_pub          = self.create_publisher(Bool,  f'{base}/boarded', 10)
        self.alighted_pub         = self.create_publisher(Bool,  f'{base}/alighted', 10)

        self.state_sub = self.create_subscription(String, f'/{self.elevator_ns}/status/state', self._on_state, 10)
        self.arrived_sub = self.create_subscription(Int32, f'/{self.elevator_ns}/status/arrived', self._on_arrived, 10)

    # === 코드 추가: 사물함 제어 함수 ===
    def control_storage_door(self, command: str):
        """ 사물함 문을 열거나 닫는 ROS 토픽을 발행합니다. """
        msg = String()
        msg.data = command
        self.door_control_pub.publish(msg)
        self.get_logger().info(f'🚪 사물함 문 제어 명령 발행: {command}')
        time.sleep(3) # 문이 움직일 시간을 대기합니다.

    def control_box(self, command: str):
        """ 사물함 안의 상자를 생성하거나 삭제합니다. """
        msg = String()
        msg.data = command
        self.box_control_pub.publish(msg)
        self.get_logger().info(f'📦 상자 제어 명령 발행: {command}')
        time.sleep(1) # 명령이 처리될 시간을 확보합니다.
    # =====================================

    def _on_state(self, msg: String):
        self.state_str = msg.data

    def _on_arrived(self, msg: Int32):
        self.arrived_floor = msg.data

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self._last_pose = {"x": p.x, "y": p.y, "yaw": yaw}

    def wait_for(self, predicate, timeout_sec: float, desc: str) -> bool:
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True

    def _navigate_to_pose_blocking(self, x: float, y: float, yaw: float, context: str = "") -> bool:
        self.nav_to_pose_client.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw/2.0)
        goal.pose.pose.orientation.w = math.cos(yaw/2.0)

        send_future = self.nav_to_pose_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"이동 goal 거부/실패 ({context})")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result()
        ok = (res.status == GoalStatus.STATUS_SUCCEEDED)
        if ok:
            self.get_logger().info(f"이동 완료 ({context})")
        else:
            self.get_logger().warn(f"이동 실패(status={res.status}) ({context})")
        return ok

    def _load_boarding_pose(self):
        p = self.boarding_poses.get(self.elevator_ns)
        if p is None:
            self.get_logger().warn(f"boarding_poses에 '{self.elevator_ns}' 키가 없습니다.")
            return False
        self.entry_pose = {"x": float(p["x"]), "y": float(p["y"]), "yaw": float(p["ori"])}
        return True

    def _goto_boarding_pose(self) -> bool:
        if not self._load_boarding_pose():
            return False
        x, y, yaw = self.entry_pose["x"], self.entry_pose["y"], self.entry_pose["yaw"]
        return self._navigate_to_pose_blocking(x, y, yaw, context="탑승 좌표(boarding_poses)")

    def call_elevator_flow(self, target_floor: int) -> bool:
        if self.current_floor == target_floor:
            self.get_logger().info(f"✅ 이미 {target_floor}층에 있습니다. 엘리베이터 이동을 생략합니다.")
            return True

        apt_pose = self.apartments.get(self.elevator_ns)
        if not apt_pose:
            self.get_logger().error(f"아파트 포즈 없음: {self.elevator_ns}")
            return False

        if not self._navigate_to_pose_blocking(apt_pose["x"], apt_pose["y"], apt_pose["ori"], context="엘리베이터 앞"):
             return False

        cur_msg = Int32(); cur_msg.data = self.current_floor
        self.current_location_pub.publish(cur_msg)
        self.get_logger().info(f"엘리베이터 호출 ({self.current_floor}층)")

        self.wait_for(lambda: (self.state_str in ("OPEN_PICKUP_DOORS", "WAIT_BOARD")), 0.0, "문 열림 대기")
        self.get_logger().info(f"[{self.elevator_ns}] 문 열림 감지 → 탑승합니다.")

        if not self._goto_boarding_pose():
            self.get_logger().error("엘리베이터 내부로 이동 실패 → 탑승 불가")
            return False
        self.get_logger().info("[엘베 탑승]")

        b = Bool(); b.data = True; self.boarded_pub.publish(b)
        tgt = Int32(); tgt.data = target_floor; self.target_floor_pub.publish(tgt)

        self.arrived_floor = None
        self.wait_for(lambda: (self.arrived_floor == target_floor) or (self.state_str in ("OPEN_DEST_DOORS", "WAIT_ALIGHT")), 0.0, "목적층 도착 대기")
        self.wait_for(lambda: (self.state_str in ("OPEN_DEST_DOORS", "WAIT_ALIGHT")), 0.0, "목적층 문 열림 대기")

        self._navigate_to_pose_blocking(apt_pose["x"], apt_pose["y"], apt_pose["ori"], context="목적층 엘베 앞")
        self.get_logger().info("[엘베 하차]")

        a = Bool(); a.data = True; self.alighted_pub.publish(a)
        self.current_floor = target_floor
        return True

def main():
    rclpy.init()

    if len(sys.argv) != 7:
        print("사용법: ros2 run <패키지> <스크립트> A_s F_s R_s A_b F_b R_b")
        print("예시: ros2 run my_pkg robot_deliver_local.py 1 2 1 3 1 2")
        rclpy.shutdown(); return

    try:
        A_s, F_s, R_s, A_b, F_b, R_b = map(int, sys.argv[1:])
    except ValueError:
        print("❌ 모든 인자는 숫자여야 합니다 (아파트동, 층, 호수)."); rclpy.shutdown(); return

    elevator_ns_map = {1: "elevator", 2: "elevator_0", 3: "elevator_1", 4: "elevator_2"}
    ns_seller = elevator_ns_map.get(A_s)
    ns_buyer  = elevator_ns_map.get(A_b)
    if ns_seller is None or ns_buyer is None:
        print(f"❌ 지원하지 않는 아파트 번호: {A_s if ns_seller is None else A_b}"); rclpy.shutdown(); return

    # === 코드 수정: 단일 노드 생성 및 재사용 ===
    nav = QuickRoomNav(ns_seller)
    
    try:
        # --- 판매자 배송 시작 ---
        nav.get_logger().info(f"--- 🚚 판매자 배송 시작: {A_s}동 {F_s}층 {R_s}호 ---")
        apt_s = nav.apartments[ns_seller]
        room_s = nav.rooms[ns_seller].get(R_s)
        if not room_s:
            nav.get_logger().error(f"잘못된 판매자 호수: {R_s}"); raise SystemExit

        if not nav.call_elevator_flow(F_s): raise SystemExit
        if not nav._navigate_to_pose_blocking(room_s["x"], room_s["y"], apt_s["ori"], context="판매자 호수"): raise SystemExit
        
        # === 코드 추가: 판매자 픽업 시나리오 ===
        nav.control_storage_door('open')
        nav.get_logger().info("⏳ 5초간 대기 (판매자가 물건을 넣는 시간)")
        time.sleep(5.0)
        nav.control_box('spawn')
        nav.control_storage_door('close')
        # ====================================

        if not nav.call_elevator_flow(1): raise SystemExit # 1층으로 복귀
        nav.get_logger().info("--- 판매자 배송 완료. 구매자에게 이동 시작 ---")

        # --- 구매자 배송 시작 ---
        # === 코드 수정: 노드를 재사용하기 위해 엘리베이터 채널만 변경 ===
        if ns_seller != ns_buyer:
            nav.elevator_ns = ns_buyer
            nav.setup_elevator_topics()
        # =======================================================
        
        nav.get_logger().info(f"--- 🚚 구매자 배송 시작: {A_b}동 {F_b}층 {R_b}호 ---")
        apt_b = nav.apartments[ns_buyer]
        room_b = nav.rooms[ns_buyer].get(R_b)
        if not room_b:
            nav.get_logger().error(f"잘못된 구매자 호수: {R_b}"); raise SystemExit

        if not nav.call_elevator_flow(F_b): raise SystemExit
        if not nav._navigate_to_pose_blocking(room_b["x"], room_b["y"], apt_b["ori"], context="구매자 호수"): raise SystemExit

        # === 코드 추가: 구매자 전달 시나리오 ===
        nav.control_storage_door('open')
        nav.get_logger().info("⏳ 5초간 대기 (구매자가 물건을 꺼내는 시간)")
        time.sleep(5.0)
        nav.control_box('delete')
        nav.control_storage_door('close')
        # ====================================

        nav.get_logger().info("✅✅✅ 전체 배송 플로우 완료 ✅✅✅")

    except (SystemExit, KeyboardInterrupt):
        nav.get_logger().info("❗️ 배송 프로세스를 중단합니다.")
    finally:
        nav.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
