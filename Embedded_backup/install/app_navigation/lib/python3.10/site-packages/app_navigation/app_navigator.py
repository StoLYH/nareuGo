#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
import json

class AppNavigator(Node):
    def __init__(self):
        super().__init__('app_navigator')
        
        # Nav2 Simple Commander 초기화
        self.navigator = BasicNavigator()
        
        # 초기화 대기
        self.get_logger().info('Nav2 초기화 대기 중...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 활성화됨!')
        
        # 로봇 초기 위치 설정 (gazebo에서 스폰된 위치)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 8.8  # aptcomplexworld의 기본 x 위치
        initial_pose.pose.position.y = -12.0  # aptcomplexworld의 기본 y 위치
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.w = 1.0  # 기본 방향
        
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info('초기 위치 설정됨')
        
        # 앱에서 좌표를 받는 토픽 구독
        # JSON 형식: {"start": [x, y, yaw], "end": [x, y, yaw]}
        self.subscription = self.create_subscription(
            String,
            '/app/navigation_command',
            self.navigation_callback,
            10
        )
        
        self.get_logger().info('App Navigator 시작됨. /app/navigation_command 토픽 대기 중...')

    def navigation_callback(self, msg):
        try:
            # JSON 파싱
            command = json.loads(msg.data)
            
            if 'end' in command:
                end_coords = command['end']
                self.navigate_to_pose(end_coords[0], end_coords[1], end_coords[2])
                
        except json.JSONDecodeError:
            self.get_logger().error('잘못된 JSON 형식')
        except Exception as e:
            self.get_logger().error(f'네비게이션 오류: {str(e)}')

    def navigate_to_pose(self, x, y, yaw):
        # 좌표 유효성 검사 (apt_complex 맵 기준 대략적 범위)
        if not (-15 <= float(x) <= 15 and -15 <= float(y) <= 15):
            self.get_logger().warn(f'목표 좌표가 유효 범위를 벗어남: x={x}, y={y}')
            return
        
        # 목표 위치 생성
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        
        # Yaw를 쿼터니언으로 변환 (tf_transformations 대신 math 사용)
        import math
        yaw_rad = float(yaw)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        
        self.get_logger().info(f'목표 좌표로 이동: x={x}, y={y}, yaw={yaw}')
        
        # 네비게이션 시작
        self.navigator.goToPose(goal_pose)
        
        # 결과 대기 (비블로킹)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            
        result = self.navigator.getResult()
        if result:
            self.get_logger().info('네비게이션 성공!')
        else:
            self.get_logger().warn('네비게이션 실패')
            self.get_logger().info(f'실패 이유: {self.navigator.getResult()}')

def main(args=None):
    rclpy.init(args=args)
    app_navigator = AppNavigator()
    
    try:
        rclpy.spin(app_navigator)
    except KeyboardInterrupt:
        pass
    finally:
        app_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()