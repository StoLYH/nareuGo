#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 런치 아규먼트 선언
        DeclareLaunchArgument(
            'robot_id',
            default_value='1',
            description='Robot ID for backend communication'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='NareuGO',
            description='Robot name for backend communication'
        ),
        DeclareLaunchArgument(
            'api_base_url',
            default_value='http://j13a501.p.ssafy.io/api',
            description='Base URL of the backend API server'
        ),
        DeclareLaunchArgument(
            'access_token',
            default_value='your_access_token',
            description='Access token for API authentication'
        ),
        DeclareLaunchArgument(
            'refresh_token',
            default_value='your_refresh_token',
            description='Refresh token for API authentication'
        ),

        # 로봇 상태 모니터 노드
        Node(
            package='robot_web_interface',
            executable='robot_status_monitor',
            name='robot_status_monitor',
            output='screen'
        ),

        # 로봇 상태 클라이언트 노드 - 작업 가능 여부 확인 API
        Node(
            package='robot_web_interface',
            executable='robot_status_client',
            name='robot_status_client',
            output='screen',
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                'robot_name': LaunchConfiguration('robot_name'),
                'api_base_url': LaunchConfiguration('api_base_url'),
                'access_token': LaunchConfiguration('access_token'),
                'refresh_token': LaunchConfiguration('refresh_token'),
                'status_interval': 15.0,  # 15초마다 상태 확인
            }]
        ),

        # 배송 주소 클라이언트 노드 - 주소 요청 API
        Node(
            package='robot_web_interface',
            executable='delivery_address_client',
            name='delivery_address_client',
            output='screen',
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                'robot_name': LaunchConfiguration('robot_name'),
                'api_base_url': LaunchConfiguration('api_base_url'),
                'access_token': LaunchConfiguration('access_token'),
                'refresh_token': LaunchConfiguration('refresh_token'),
            }]
        ),
    ])