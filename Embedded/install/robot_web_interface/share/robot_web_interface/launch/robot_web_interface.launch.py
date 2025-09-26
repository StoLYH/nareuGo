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
            description='Robot ID to send to server'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='NareuGO',
            description='Robot name to send to server'
        ),
        DeclareLaunchArgument(
            'api_base_url',
            default_value='http://localhost:8080',
            description='Base URL of the API server'
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
        DeclareLaunchArgument(
            'status_interval',
            default_value='10.0',
            description='Interval in seconds to send status updates'
        ),

        # 로봇 상태 모니터 노드
        Node(
            package='robot_web_interface',
            executable='robot_status_monitor',
            name='robot_status_monitor',
            output='screen'
        ),

        # 웹 API 클라이언트 노드
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
                'status_interval': LaunchConfiguration('status_interval'),
            }]
        ),
    ])