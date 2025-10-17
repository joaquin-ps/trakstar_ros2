#!/usr/bin/env python3

"""
Launch file for Trakstar driver node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'hemisphere_back',
            default_value='false',
            description='Set hemisphere to back (true) or front (false)'
        ),
        DeclareLaunchArgument(
            'frequency',
            default_value='255',
            description='Measurement frequency in Hz'
        ),
        DeclareLaunchArgument(
            'range_72inch',
            default_value='false',
            description='Use 72 inch range (true) or 36 inch range (false)'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='false',
            description='Publish transforms to TF (true) or not (false)'
        ),
        
        # Pivot point parameters for sensor 0
        DeclareLaunchArgument(
            'pivot_x',
            default_value='0.0',
            description='X offset for sensor 0 pivot point'
        ),
        DeclareLaunchArgument(
            'pivot_y',
            default_value='0.0',
            description='Y offset for sensor 0 pivot point'
        ),
        DeclareLaunchArgument(
            'pivot_z',
            default_value='0.0',
            description='Z offset for sensor 0 pivot point'
        ),
        DeclareLaunchArgument(
            'attach_roll',
            default_value='0.0',
            description='Roll angle for sensor 0 attachment'
        ),
        DeclareLaunchArgument(
            'attach_pitch',
            default_value='0.0',
            description='Pitch angle for sensor 0 attachment'
        ),
        DeclareLaunchArgument(
            'attach_yaw',
            default_value='0.0',
            description='Yaw angle for sensor 0 attachment'
        ),
        
        # Pivot point parameters for sensor 1
        DeclareLaunchArgument(
            'pivot_x1',
            default_value='0.0',
            description='X offset for sensor 1 pivot point'
        ),
        DeclareLaunchArgument(
            'pivot_y1',
            default_value='0.0',
            description='Y offset for sensor 1 pivot point'
        ),
        DeclareLaunchArgument(
            'pivot_z1',
            default_value='0.0',
            description='Z offset for sensor 1 pivot point'
        ),
        DeclareLaunchArgument(
            'attach_roll1',
            default_value='0.0',
            description='Roll angle for sensor 1 attachment'
        ),
        DeclareLaunchArgument(
            'attach_pitch1',
            default_value='0.0',
            description='Pitch angle for sensor 1 attachment'
        ),
        DeclareLaunchArgument(
            'attach_yaw1',
            default_value='0.0',
            description='Yaw angle for sensor 1 attachment'
        ),

        # Trakstar driver node
        Node(
            package='trakstar',
            executable='trakstar_driver',
            name='trakstar_driver',
            output='screen',
            parameters=[{
                'hemisphere_back': LaunchConfiguration('hemisphere_back'),
                'frequency': LaunchConfiguration('frequency'),
                'range_72inch': LaunchConfiguration('range_72inch'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'pivot_x': LaunchConfiguration('pivot_x'),
                'pivot_y': LaunchConfiguration('pivot_y'),
                'pivot_z': LaunchConfiguration('pivot_z'),
                'attach_roll': LaunchConfiguration('attach_roll'),
                'attach_pitch': LaunchConfiguration('attach_pitch'),
                'attach_yaw': LaunchConfiguration('attach_yaw'),
                'pivot_x1': LaunchConfiguration('pivot_x1'),
                'pivot_y1': LaunchConfiguration('pivot_y1'),
                'pivot_z1': LaunchConfiguration('pivot_z1'),
                'attach_roll1': LaunchConfiguration('attach_roll1'),
                'attach_pitch1': LaunchConfiguration('attach_pitch1'),
                'attach_yaw1': LaunchConfiguration('attach_yaw1'),
            }]
        ),
    ])
