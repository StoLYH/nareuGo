#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Int32
import sys
import math

class QuickRoomNav(Node):
    def __init__(self):
        super().__init__('quick_room_nav')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 엘리베이터 통신용 퍼블리셔들
        self.current_location_pub = self.create_publisher(Int32, '/robot/current_location', 10)
        self.boarded_pub = self.create_publisher(String, '/robot/boarded', 10)
        self.alighted_pub = self.create_publisher(String, '/robot/alighted', 10)

        # 방 좌표 (층수 정보 포함)
        self.rooms = {
            1: {"name": "1번방", "x": 6.4, "y": -4.07, "ori": -1.3, "floor": 1},
            2: {"name": "2번방", "x": 14.38, "y": -4.16, "ori": -1.85, "floor": 1},
            3: {"name": "3번방", "x": 18.58, "y": 1.87, "ori": 0.0, "floor": 2},
            4: {"name": "4번방", "x": 18.8, "y": 8.4, "ori": 0.0, "floor": 2}
        }

        # 현재 로봇 위치 (층수)
        self.current_floor = 1

    def call_elevator(self, target_floor):
        """엘리베이터 호출 - 현재 층 정보 전송"""
        msg = Int32()
        msg.data = self.current_floor
        self.current_location_pub.publish(msg)
        print(f"🛗 엘리베이터 호출: 현재 {self.current_floor}층에서 {target_floor}층으로 이동 요청")

    def notify_boarded(self):
        """엘리베이터 탑승 완료 알림"""
        msg = String()
        msg.data = f"Robot boarded at floor {self.current_floor}"
        self.boarded_pub.publish(msg)
        print(f"✅ 엘리베이터 탑승 완료 알림 전송 (현재 층: {self.current_floor})")

    def notify_alighted(self, new_floor):
        """엘리베이터 하차 완료 알림"""
        msg = String()
        msg.data = f"Robot alighted at floor {new_floor}"
        self.alighted_pub.publish(msg)
        self.current_floor = new_floor
        print(f"✅ 엘리베이터 하차 완료 알림 전송 (새 층: {new_floor})")

    def go_to_room(self, room_number):
        """방 번호로 이동 (엘리베이터 로직 포함)"""
        if room_number not in self.rooms:
            print(f"❌ 잘못된 방 번호: {room_number}")
            print("사용법: python3 go_room.py [1|2|3|4]")
            return False

        room = self.rooms[room_number]
        target_floor = room["floor"]

        # 층간 이동이 필요한 경우 엘리베이터 로직 실행
        if self.current_floor != target_floor:
            print(f"🚀 층간 이동 필요: {self.current_floor}층 → {target_floor}층")

            # 1. 엘리베이터 호출
            self.call_elevator(target_floor)

            # 실제 구현에서는 여기서 엘리베이터 위치로 이동하고 대기해야 함
            # 지금은 시뮬레이션을 위해 사용자 입력을 받음
            input("🛗 엘리베이터에 탑승한 후 Enter를 누르세요...")

            # 2. 탑승 완료 알림
            self.notify_boarded()

            input(f"🛗 {target_floor}층에 도착한 후 Enter를 누르세요...")

            # 3. 하차 완료 알림
            self.notify_alighted(target_floor)

        # NavigateToPose 액션이 준비될 때까지 대기
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            print("❌ NavigateToPose 액션 서버를 찾을 수 없습니다!")
            return False

        # NavigateToPose 액션 목표 생성 (orientation 포함)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(room["x"])
        goal_msg.pose.pose.position.y = float(room["y"])
        goal_msg.pose.pose.position.z = 0.0

        # orientation을 quaternion으로 변환 (z축 회전)
        yaw = room["ori"]
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        print(f"🎯 {room['name']}으로 이동: ({room['x']}, {room['y']}, 각도: {math.degrees(yaw):.1f}°)")

        # 액션 목표 전송
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("❌ 목표가 거부되었습니다!")
            return

        print("✅ 목표가 수락되었습니다. 이동 시작...")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        _ = future.result().result
        print("🎉 목표 도달 완료!")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        # 진행 상황 피드백 (선택사항)
        _ = feedback_msg

def main():
    rclpy.init()

    if len(sys.argv) != 2:
        print("사용법: python3 go_room.py [방번호]")
        print("방번호: 1(1번방), 2(2번방), 3(3번방), 4(4번방)")
        return

    try:
        room_number = int(sys.argv[1])
    except ValueError:
        print("❌ 방 번호는 숫자여야 합니다 (1, 2, 3, 4)")
        return

    navigator = QuickRoomNav()

    if navigator.go_to_room(room_number):
        print("✅ 이동 명령 전송 완료!")
        # 액션이 완료될 때까지 스핀
        rclpy.spin(navigator)

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    #엘리베이터 호출 로직 향후 추가 예정