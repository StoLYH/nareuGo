#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose', default='6.8')
    y_pose = LaunchConfiguration('y_pose', default='5.0')
    yaw = LaunchConfiguration('yaw', default='3.14')
    
    # Declare launch arguments
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose', default_value='6.8',
        description='X position of the robot')
    
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', default_value='5.0', 
        description='Y position of the robot')
        
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', default_value='3.14',
        description='Yaw orientation of the robot')

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('turtlebot3_gazebo'),
            '/launch/turtlebot3_outerworld.launch.py'
        ]),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Navigation2 launch (5초 후 시작)
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    '/opt/ros/humble/share/nav2_bringup/launch/bringup_launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': 'True',
                    'map': '/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml'
                }.items()
            )
        ]
    )

    # Initial pose setter (10초 후 실행)
    set_initial_pose_cmd = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', '/home/donggun/turtle_ws/src/set_initial_pose.py'],
                output='screen'
            )
        ]
    )

    # RViz2 launch (12초 후 시작)
    rviz_launch = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    '/opt/ros/humble/share/nav2_bringup/launch/rviz_launch.py'
                ])
            )
        ]
    )

    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd) 
    ld.add_action(declare_yaw_cmd)
    
    # Add actions
    ld.add_action(gazebo_launch)
    ld.add_action(nav2_launch)
    ld.add_action(set_initial_pose_cmd)
    ld.add_action(rviz_launch)
    
    return ld