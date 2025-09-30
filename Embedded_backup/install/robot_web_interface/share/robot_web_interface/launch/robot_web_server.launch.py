#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 런치 아규먼트 선언
        DeclareLaunchArgument(
            'server_port',
            default_value='8888',
            description='HTTP server port'
        ),
        DeclareLaunchArgument(
            'server_host',
            default_value='0.0.0.0',
            description='HTTP server host'
        ),

        # TurtleBot 상태 모니터 노드 - Gazebo TurtleBot의 실제 상태를 감지
        Node(
            package='robot_web_interface',
            executable='turtlebot_status_monitor',
            name='turtlebot_status_monitor',
            output='screen'
        ),

        # HTTP 서버 노드 - 웹에서의 요청을 받아 처리
        Node(
            package='robot_web_interface',
            executable='robot_http_server',
            name='robot_http_server',
            output='screen',
            parameters=[{
                'server_port': LaunchConfiguration('server_port'),
                'server_host': LaunchConfiguration('server_host'),
            }]
        ),
    ])